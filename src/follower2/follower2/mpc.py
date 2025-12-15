import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32  # Added Float32 for speed topic
import numpy as np
import cvxpy as cp

class MPCFollowerQP:
    """
    QP-based MPC follower controller with actuator dynamics (first-order lag).
    Modified to include Leader Acceleration (Throttle) in prediction.
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

        # Store parameters
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
        self.a_leader = cp.Parameter() # NEW: Leader Acceleration Parameter
        self.u_prev = cp.Parameter()   

        # Predicted follower position trajectory
        pos_pred = self.Px_pos @ self.x0 + self.Pu_pos @ self.U
        
        # Predicted leader trajectory (Includes acceleration term now)
        # p(t) = p0 + v*t + 0.5*a*t^2
        leader_vec = (self.leader_pos0 + 
                      self.v_leader * self.time_steps + 
                      0.5 * self.a_leader * cp.square(self.time_steps))
        
        # Predicted distance trajectory
        dist = leader_vec - pos_pred

        # Cost Functions
        err = dist - self.desired_distance
        cost_dist = self.Qd * cp.sum_squares(err)
        cost_u = self.Ru * cp.sum_squares(self.U)
        Du = cp.vstack([self.U[0] - self.u_prev, self.U[1:] - self.U[:-1]])
        cost_du = self.Rdu * cp.sum_squares(Du)

        cost = cost_dist + cost_u + cost_du

        # Constraints
        constraints = [
            dist >= self.safety_distance,
            self.U >= self.u_min,
            self.U <= self.u_max
        ]

        self.problem = cp.Problem(cp.Minimize(cost), constraints)
        self.U_warm = np.zeros(self.N)

    def compute_control(self, x_follower, d_meas, v_follower, leader_throttle, 
                        d_prev=None, u_prev_cmd=0.0):
        """
        Computes u_cmd using leader throttle and distance.
        """
        pos_f, vel_f, a_act_f = x_follower
        
        # We work in a local frame where current follower pos is 0.0
        local_x0 = [0.0, vel_f, a_act_f]
        
        # Leader is at distance d_meas
        leader_pos0_val = d_meas

        # Estimate Leader Velocity
        # v_leader = v_follower + d_dot
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

        # Solve
        try:
            self.problem.solve(solver=self.solver, verbose=self.verbose, warm_start=True, osqp_param={'verbose': 0})
        except Exception:
            return float(np.clip(-1.5 * (self.desired_distance - d_meas), self.u_min, self.u_max))

        if self.problem.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
             return float(np.clip(-1.5 * (self.desired_distance - d_meas), self.u_min, self.u_max))

        Uopt = np.array(self.U.value).flatten()
        self.U_warm = np.concatenate((Uopt[1:], [Uopt[-1]]))
        
        return float(Uopt[0])


class PlatoonMPCNode(Node):
    def __init__(self):
        super().__init__('platoon_mpc_node')

        # --- Parameters ---
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('target_dist', 1.0) # Desired following distance
        
        self.dt = self.get_parameter('dt').value
        target_dist = self.get_parameter('target_dist').value

        # --- MPC Controller ---
        self.mpc = MPCFollowerQP(
            dt=self.dt,
            horizon=20,
            desired_distance=target_dist,
            safety_distance=0.3, # Hard constraint
            verbose=False
        )

        # --- State Variables ---
        self.leader_throttle = 0.0
        self.current_distance = 0.0
        self.current_velocity = 0.0
        self.prev_distance = None
        self.prev_u_cmd = 0.0
        self.current_accel_estimate = 0.0 
        
        # Safety check: Is data fresh?
        self.last_dist_time = self.get_clock().now()

        # --- Subscribers ---
        # 1. Throttle of vehicle in front
        self.sub_leader_u = self.create_subscription(
            Float64, 'follower1/motor_throttle', self.leader_throttle_callback, 10)
        
        # 2. Distance to vehicle in front
        self.sub_dist = self.create_subscription(
            Float64, 'follower2/sonar_dist', self.distance_callback, 10)
            
        # 3. Own Velocity (FROM SPEED NODE, NOT ODOM)
        # Subscribes to the calculated m/s from the previous speed.py code
        self.sub_speed = self.create_subscription(
            Float32, 'follower2/encoder_speed_mps', self.speed_callback, 10)

        # --- Publishers ---
        self.pub_throttle = self.create_publisher(Float64, 'follower2/motor_throttle', 10)

        # --- Control Loop ---
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Platoon MPC Node Started")

    def leader_throttle_callback(self, msg):
        self.leader_throttle = msg.data

    def distance_callback(self, msg):
        self.prev_distance = self.current_distance
        self.current_distance = msg.data
        self.last_dist_time = self.get_clock().now()

    def speed_callback(self, msg):
        """
        Updates current velocity from the hall effect sensor node.
        """
        self.current_velocity = float(msg.data)

    def control_loop(self):
        # 1. Check data freshness (safety)
        time_since_dist = (self.get_clock().now() - self.last_dist_time).nanoseconds / 1e9
        if time_since_dist > 1.0:
            self.get_logger().warn("Distance data stale. Stopping.", throttle_duration_sec=1)
            self.stop_vehicle()
            return

        # 2. Construct Follower State x = [pos, vel, acc]
        tau = self.mpc.tau
        self.current_accel_estimate += (self.dt / tau) * (self.prev_u_cmd - self.current_accel_estimate)
        
        x_follower = [0.0, self.current_velocity, self.current_accel_estimate]

        # 3. Compute Control
        u_cmd = self.mpc.compute_control(
            x_follower=x_follower,
            d_meas=self.current_distance,
            v_follower=self.current_velocity,
            leader_throttle=self.leader_throttle,
            d_prev=self.prev_distance,
            u_prev_cmd=self.prev_u_cmd
        )

        # 4. Publish
        msg = Float64()
        msg.data = u_cmd
        self.pub_throttle.publish(msg)

        # 5. Update history
        self.prev_u_cmd = u_cmd

    def stop_vehicle(self):
        msg = Float64()
        msg.data = -1.0 # Braking
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