import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cvxpy as cp

class MPCFollowerQP:
    """
    QP-based MPC follower controller with actuator dynamics.
    REVERSE REMOVED: This controller is strictly non-negative (0.0 to u_max).
    """
    def __init__(self,
                 dt=0.05,
                 horizon=10,
                 tau_act=0.08,
                 Qd=1.0,
                 Ru=0.1,
                 Rdu=0.1,
                 u_min=0.0,
                 u_max=1.0,
                 safety_distance=0.05,
                 desired_distance=0.10, 
                 solver=cp.OSQP,
                 verbose=False):

        # Store parameters
        self.dt = float(dt)
        self.N = int(horizon)
        self.tau = float(tau_act)
        self.Qd = float(Qd)
        self.Ru = float(Ru)
        self.Rdu = float(Rdu)
        
        # --- MODIFICATION 1: STRICTLY ENFORCE NO REVERSE ---
        # Even if the user passes a negative u_min, we force it to 0.0
        self.u_min = max(0.0, float(u_min)) 
        self.u_max = float(u_max)
        
        self.safety_distance = float(safety_distance)
        self.desired_distance = float(desired_distance)
        self.solver = solver
        self.verbose = bool(verbose)

        # Build discrete system matrices (A and B for x_{k+1} = A*x_k + B*u_k)
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

        # Precompute prediction matrices Sx and Su
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

        # Extract predicted positions
        pos_rows = [i * nx for i in range(N)]
        self.Px_pos = Sx[pos_rows, :]
        self.Pu_pos = Su[pos_rows, :]
        self.time_steps = np.array([(k+1) * dt for k in range(N)])

        # --- Build CVXPY Problem ---
        self.U = cp.Variable(N)        # Control sequence
        self.x0 = cp.Parameter(nx)     # Current follower state
        self.leader_pos0 = cp.Parameter() 
        self.v_leader = cp.Parameter() 
        self.a_leader = cp.Parameter() 
        self.u_prev = cp.Parameter()   

        # Predicted follower position trajectory
        pos_pred = self.Px_pos @ self.x0 + self.Pu_pos @ self.U
        
        # Predicted leader trajectory 
        leader_vec = (self.leader_pos0 + 
                      self.v_leader * self.time_steps + 
                      0.5 * self.a_leader * cp.square(self.time_steps))
        
        # Predicted distance trajectory
        dist = leader_vec - pos_pred

        # Cost Functions
        err = dist - self.desired_distance
        cost_dist = self.Qd * cp.sum_squares(err)
        cost_u = self.Ru * cp.sum_squares(self.U)
        Du = cp.hstack([self.U[0] - self.u_prev, self.U[1:] - self.U[:-1]])
        cost_du = self.Rdu * cp.sum_squares(Du)

        cost = cost_dist + cost_u + cost_du

        # Constraints
        constraints = [
            dist >= self.safety_distance,
            # --- MODIFICATION 2: Explicit Hard Constraint >= 0.0 ---
            self.U >= 0.0, 
            self.U >= self.u_min, # Redundant but keeps parameter logic valid
            self.U <= self.u_max
        ]

        self.problem = cp.Problem(cp.Minimize(cost), constraints)
        self.U_warm = np.zeros(self.N)

    def compute_control(self, x_follower, d_meas, v_follower, leader_throttle, 
                        d_prev=None, u_prev_cmd=0.0):
        """
        Computes u_cmd. Returns only values >= 0.0.
        """
        pos_f, vel_f, a_act_f = x_follower
        
        local_x0 = [0.0, vel_f, a_act_f]
        leader_pos0_val = d_meas

        if d_prev is not None:
            d_dot = (d_meas - d_prev) / max(self.dt, 1e-9)
        else:
            d_dot = 0.0
        
        v_lead = d_dot + float(v_follower)

        # Assign CVXPY parameters
        self.x0.value = np.array(local_x0, dtype=float)
        self.leader_pos0.value = float(leader_pos0_val)
        self.v_leader.value = float(v_lead)
        self.a_leader.value = float(leader_throttle)
        self.u_prev.value = float(u_prev_cmd)

        self.U.value = self.U_warm

        # --- Helper for Fallback P-Controller ---
        def get_fallback_command():
            # If we are closer than desired, we want to STOP (0.0), not reverse (<0).
            # If we are further than desired, we want to DRIVE (>0).
            error_gap = d_meas - self.desired_distance
            
            # Simple P-gain
            kp = 1.5 
            u_p = kp * error_gap
            
            # Clip between 0.0 and max. 
            # This ensures if error_gap is negative (too close), we return 0.0.
            return float(np.clip(u_p, 0.0, self.u_max))

        # Solve
        try:
            self.problem.solve(solver=self.solver, verbose=self.verbose, warm_start=True, osqp_param={'verbose': 0})
        except Exception:
            return get_fallback_command()

        if self.problem.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
             return get_fallback_command()

        Uopt = np.array(self.U.value).flatten()
        self.U_warm = np.concatenate((Uopt[1:], [Uopt[-1]]))
        
        # Final safety clip to ensure non-negative output
        return max(0.0, float(Uopt[0]))

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
            # Ensure min is 0 here as well
            u_min=0.0,
            verbose=False
        )

        # --- State Variables ---
        self.leader_throttle = 0.0
        self.current_distance = 0.0
        self.current_velocity = 0.0
        self.prev_distance = None
        self.prev_u_cmd = 0.0
        self.current_accel_estimate = 0.0
        
        self.last_dist_time = self.get_clock().now()

        # --- Subscribers ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )       

        self.sub_dist = self.create_subscription(
            Range, 
            'follower1/sonar_dist', 
            self.distance_callback, 
            sensor_qos
        )
        self.get_logger().info("Subscribed to follower1/sonar_dist with BEST_EFFORT QoS")

        self.sub_leader_u = self.create_subscription(
            Float32, 'leader/motor_throttle', self.leader_throttle_callback, 10)
        
        self.sub_odom = self.create_subscription(
            Float32, 'follower1/encoder_speed_mps', self.odom_callback, 10)

        # --- Publishers ---
        self.pub_throttle = self.create_publisher(Float32, 'follower1/motor_throttle', 10)

        # --- Control Loop ---
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Platoon MPC Node Started - NO REVERSE MODE")

    def leader_throttle_callback(self, msg):
        self.leader_throttle = msg.data

    def distance_callback(self, msg):
        self.get_logger().info(f"Dist: {msg.range:.3f}", throttle_duration_sec=2.0)
        self.prev_distance = self.current_distance
        self.current_distance = msg.range
        self.last_dist_time = self.get_clock().now()

    def odom_callback(self, msg):
        self.current_velocity = msg.data

    def control_loop(self):
        now = self.get_clock().now()
        time_since_dist = (now - self.last_dist_time).nanoseconds / 1e9
        
        if time_since_dist > 0.1:
            self.get_logger().warn(f"Lag: {time_since_dist:.4f}s", throttle_duration_sec=1.0)

        if time_since_dist > 0.5:
            self.get_logger().error("Data Stale. Stopping.", throttle_duration_sec=1.0)
            self.stop_vehicle()
            return

        # Construct Follower State 
        tau = self.mpc.tau
        self.current_accel_estimate += (self.dt / tau) * (self.prev_u_cmd - self.current_accel_estimate)
        x_follower = [0.0, self.current_velocity, self.current_accel_estimate]

        # Compute Control
        u_cmd = self.mpc.compute_control(
            x_follower=x_follower,
            d_meas=self.current_distance,
            v_follower=self.current_velocity,
            leader_throttle=self.leader_throttle,
            d_prev=self.prev_distance,
            u_prev_cmd=self.prev_u_cmd
        )

        # Publish
        msg = Float32()
        msg.data = u_cmd
        self.pub_throttle.publish(msg)
        
        self.get_logger().debug(f"MPC Cmd: {u_cmd:.3f}", throttle_duration_sec=1.0)

        self.prev_u_cmd = u_cmd

    def stop_vehicle(self):
        msg = Float32()
        msg.data = 0.0 # 0.0 is strictly STOP, not reverse
        self.pub_throttle.publish(msg)

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