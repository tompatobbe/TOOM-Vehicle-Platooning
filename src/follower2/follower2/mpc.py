import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import numpy as np
import cvxpy as cp

class MPCFollowerQP:
    """
    QP-based MPC follower controller with actuator dynamics (first-order lag).
    """

    def __init__(self,
                 dt=0.05,
                 horizon=15,
                 tau_act=0.15,
                 Qd=500.0,
                 Ru=0.01,
                 Rdu=10.0,
                 u_min=-6.0,
                 u_max=3.0,
                 safety_distance=0.15,
                 desired_distance=0.50,
                 solver=cp.OSQP,
                 verbose=False):

        self.dt = float(dt)
        self.N = int(horizon)
        self.tau = float(tau_act)
        self.Qd = float(Qd)
        self.Ru = float(Ru)
        self.Rdu = float(Rdu)
        self.u_min = float(u_min)
        self.u_max = float(u_max)
        self.safety_distance = float(safety_distance)
        self.desired_distance = float(desired_distance)
        self.solver = solver
        self.verbose = bool(verbose)

        dt = self.dt
        # State: [pos, vel, a_act]
        A = np.array([
            [1.0, dt, 0.5 * dt**2],
            [0.0, 1.0, dt],
            [0.0, 0.0, 1.0 - dt / self.tau]
        ])
        B = np.array([[0.0], [0.0], [dt / self.tau]])

        self.A = A
        self.B = B

        # Precompute prediction matrices
        nx = 3
        N = self.N
        Sx = np.zeros((nx*N, nx))
        Su = np.zeros((nx*N, N))

        A_pow = np.eye(nx)
        for k in range(N):
            A_pow = A @ A_pow
            Sx[k*nx:(k+1)*nx, :] = A_pow
            for j in range(k + 1):
                Aj = np.linalg.matrix_power(A, k - j)
                Su[k*nx:(k+1)*nx, j] = (Aj @ B).flatten()

        pos_rows = [i * nx for i in range(N)]
        self.Px_pos = Sx[pos_rows, :]
        self.Pu_pos = Su[pos_rows, :]
        self.time_steps = np.array([(k+1) * dt for k in range(N)])

        # --- Build CVXPY Problem ---
        self.U = cp.Variable(N)
        self.x0 = cp.Parameter(nx)
        self.leader_pos0 = cp.Parameter() 
        self.v_leader = cp.Parameter() 
        self.a_leader = cp.Parameter()
        self.u_prev = cp.Parameter()   

        pos_pred = self.Px_pos @ self.x0 + self.Pu_pos @ self.U
        
        leader_vec = (self.leader_pos0 + 
                      self.v_leader * self.time_steps + 
                      0.5 * self.a_leader * cp.square(self.time_steps))
        
        dist = leader_vec - pos_pred

        # Cost Functions
        err = dist - self.desired_distance
        cost_dist = self.Qd * cp.sum_squares(err)
        cost_u = self.Ru * cp.sum_squares(self.U)
        
        # FIX 1: Use hstack for 1D stacking
        Du = cp.hstack([self.U[0] - self.u_prev, self.U[1:] - self.U[:-1]])
        cost_du = self.Rdu * cp.sum_squares(Du)

        cost = cost_dist + cost_u + cost_du

        constraints = [
            dist >= self.safety_distance,
            self.U >= self.u_min,
            self.U <= self.u_max
        ]

        self.problem = cp.Problem(cp.Minimize(cost), constraints)
        self.U_warm = np.zeros(self.N)

    def compute_control(self, x_follower, d_meas, v_follower, leader_throttle, 
                        d_prev=None, u_prev_cmd=0.0):
        pos_f, vel_f, a_act_f = x_follower
        local_x0 = [0.0, vel_f, a_act_f]
        leader_pos0_val = d_meas

        if d_prev is not None:
            d_dot = (d_meas - d_prev) / max(self.dt, 1e-9)
        else:
            d_dot = 0.0
        
        v_lead = d_dot + float(v_follower)

        self.x0.value = np.array(local_x0, dtype=float)
        self.leader_pos0.value = float(leader_pos0_val)
        self.v_leader.value = float(v_lead)
        self.a_leader.value = float(leader_throttle)
        self.u_prev.value = float(u_prev_cmd)

        self.U.value = self.U_warm

        try:
            self.problem.solve(solver=self.solver, verbose=self.verbose, warm_start=True, osqp_param={'verbose': 0})
        except Exception as e:
            fallback = float(np.clip(-1.5 * (self.desired_distance - d_meas), self.u_min, self.u_max))
            # Best-effort debug print since we don't have a ROS logger here
            print(f"MPC compute_control: solver exception: {e}; returning fallback {fallback}")
            return fallback, 'EXCEPTION'

        if self.problem.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            fallback = float(np.clip(-1.5 * (self.desired_distance - d_meas), self.u_min, self.u_max))
            print(f"MPC compute_control: problem status {self.problem.status}; returning fallback {fallback}")
            return fallback, 'INFEASIBLE'

        Uopt = np.array(self.U.value).flatten()
        self.U_warm = np.concatenate((Uopt[1:], [Uopt[-1]]))

        return float(Uopt[0]), 'OPTIMAL'


class PlatoonMPCNode(Node):
    def __init__(self):
        super().__init__('platoon_mpc_node')

        # --- Parameters ---
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('target_dist', 1.0)
        
        self.dt = self.get_parameter('dt').value
        target_dist = self.get_parameter('target_dist').value

        # --- MPC Controller ---
        self.mpc = MPCFollowerQP(
            dt=self.dt,
            horizon=20,
            desired_distance=target_dist,
            safety_distance=0.3,
            verbose=False
        )

        # --- State Variables ---
        self.leader_throttle = 0.0
        self.current_distance = 0.0
        self.current_velocity = 0.0
        self.prev_distance = None
        self.prev_u_cmd = 0.0
        self.current_accel_estimate = 0.0
        
        # Initialize time to NOW so we don't timeout instantly on startup
        self.last_dist_time = self.get_clock().now()

        # --- QoS Profile ---
        # FIX 2: Create a "Best Effort" QoS profile for sensors
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Subscribers ---
        self.sub_leader_u = self.create_subscription(
            Float64, 'leader/motor_throttle', self.leader_throttle_callback, 10)
        
        # FIX 2: Apply QoS profile here
        self.sub_dist = self.create_subscription(
            Float64, 'follower1/sonar_dist', self.distance_callback, qos_sensor)
            
        self.sub_odom = self.create_subscription(
            Odometry, '/ego/odom', self.odom_callback, 10)

        # --- Publishers ---
        self.pub_throttle = self.create_publisher(Float64, 'follower1/motor_throttle', 10)

        # Log subscriptions/publishers
        self.get_logger().info("Subscribed to: leader/motor_throttle, follower1/sonar_dist, /ego/odom")
        self.get_logger().info("Publishing to: follower1/motor_throttle")

        # --- Control Loop ---
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Platoon MPC Node Started")

    def leader_throttle_callback(self, msg):
        self.leader_throttle = msg.data
        t = (self.get_clock().now()).nanoseconds / 1e9
        self.get_logger().info(f"Incoming [leader/motor_throttle]: {self.leader_throttle:.4f} @ {t:.3f}s")

    def distance_callback(self, msg):
        self.prev_distance = self.current_distance
        self.current_distance = msg.data
        self.last_dist_time = self.get_clock().now()
        t = (self.last_dist_time).nanoseconds / 1e9
        self.get_logger().info(f"Incoming [follower1/sonar_dist]: {self.current_distance:.4f} (prev {self.prev_distance}) @ {t:.3f}s")

    def odom_callback(self, msg):
        self.current_velocity = msg.twist.twist.linear.x
        t = (self.get_clock().now()).nanoseconds / 1e9
        self.get_logger().info(f"Incoming [/ego/odom]: vel={self.current_velocity:.4f} @ {t:.3f}s")

    def control_loop(self):
        # 1. Check data freshness (safety)
        time_since_dist = (self.get_clock().now() - self.last_dist_time).nanoseconds / 1e9
        
        # FIX 3: Increased timeout to 1.0s to allow for sensor startup lag
        if time_since_dist > 1.0:
            self.get_logger().warn(f"Distance data stale ({time_since_dist:.2f}s). Stopping.", throttle_duration_sec=2)
            self.stop_vehicle()
            return

        # Debug: log inputs to compute_control
        self.get_logger().info(f"Inputs: distance={self.current_distance:.4f}, prev_distance={self.prev_distance}, vel={self.current_velocity:.4f}, leader_throttle={self.leader_throttle:.4f}, prev_u={self.prev_u_cmd:.4f}, age={time_since_dist:.3f}s")

        # 2. Construct Follower State
        tau = self.mpc.tau
        self.current_accel_estimate += (self.dt / tau) * (self.prev_u_cmd - self.current_accel_estimate)
        
        x_follower = [0.0, self.current_velocity, self.current_accel_estimate]

        # 3. Compute Control
        u_cmd, status = self.mpc.compute_control(
            x_follower=x_follower,
            d_meas=self.current_distance,
            v_follower=self.current_velocity,
            leader_throttle=self.leader_throttle,
            d_prev=self.prev_distance,
            u_prev_cmd=self.prev_u_cmd
        )

        self.get_logger().info(f"MPC output: u_cmd={u_cmd:.4f}, status={status}")

        # 4. Publish
        msg = Float64()
        msg.data = u_cmd
        self.pub_throttle.publish(msg)
        self.get_logger().info(f"Outgoing [follower1/motor_throttle]: {msg.data:.4f}")
        
        # 5. Update history
        self.prev_u_cmd = u_cmd

    def stop_vehicle(self):
        msg = Float64()
        msg.data = -1.0
        self.pub_throttle.publish(msg)
        self.get_logger().info(f"Outgoing [follower1/motor_throttle] STOP: {msg.data:.4f}")

def main(args=None):
    rclpy.init(args=args)
    node = PlatoonMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_vehicle()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()