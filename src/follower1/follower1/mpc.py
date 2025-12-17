import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy # Import QoS modules
import numpy as np
import cvxpy as cp

class MPCFollowerQP:
    """
    QP-based MPC follower controller with actuator dynamics (first-order lag).
    Modified to include Leader Acceleration (Throttle) in prediction.
    """
    def __init__(self,
                 dt=0.05,
                 horizon=20,          # CHANGED: Increased horizon for smoother planning
                 tau_act=0.08,
                 Qd=300.0,            # CHANGED: Lowered from 1500.0 (Less aggressive catch-up)
                 Ru=5.0,              # CHANGED: Increased from 0.1 (Penalize high throttle usage)
                 Rdu=100.0,           # CHANGED: Increased from 10.0 (Strongly penalize jerky throttle changes)
                 u_min=-1.0,
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
        Du = cp.hstack([self.U[0] - self.u_prev, self.U[1:] - self.U[:-1]])
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
        # This prevents floating point issues with large global coordinates.
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
        self.a_leader.value = float(leader_throttle) # Pass leader throttle here
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
        self.declare_parameter('target_dist', 1.0) 
        
        # CHANGED: Default offset to 0.0. Only increase this if your ESC needs 
        # a specific center value (e.g., 1500 for PWM, but usually 0.0 for ROS floats)
        self.declare_parameter('throttle_offset', 0.0)
        
        # Friction Deadband
        self.declare_parameter('friction_deadband', 0.3) 

        self.dt = self.get_parameter('dt').value
        target_dist = self.get_parameter('target_dist').value
        self.throttle_offset = self.get_parameter('throttle_offset').value
        self.friction_deadband = self.get_parameter('friction_deadband').value

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
        
        self.last_dist_time = self.get_clock().now()

        # --- Subscribers ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )       

        self.sub_dist = self.create_subscription(
            Range, 'follower1/sonar_dist', self.distance_callback, sensor_qos)

        self.sub_leader_u = self.create_subscription(
            Float32, 'leader/motor_throttle', self.leader_throttle_callback, 10)
        
        self.sub_odom = self.create_subscription(
            Float32, 'follower1/encoder_speed_mps', self.odom_callback, 10)

        # --- Publishers ---
        self.pub_throttle = self.create_publisher(Float32, 'follower1/motor_throttle', 10)

        # --- Control Loop ---
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Platoon MPC Node Started")

    def leader_throttle_callback(self, msg):
        self.leader_throttle = msg.data

    def distance_callback(self, msg):
        self.prev_distance = self.current_distance
        self.current_distance = msg.range
        self.last_dist_time = self.get_clock().now()

    def odom_callback(self, msg):
        self.current_velocity = msg.data

    def control_loop(self):
        now = self.get_clock().now()
        time_since_dist = (now - self.last_dist_time).nanoseconds / 1e9
        
        # Safety Check
        if time_since_dist > 0.5:
            self.stop_vehicle()
            return

        # 1. Estimate State
        tau = self.mpc.tau
        self.current_accel_estimate += (self.dt / tau) * (self.prev_u_cmd - self.current_accel_estimate)
        x_follower = [0.0, self.current_velocity, self.current_accel_estimate]

        # 2. Compute MPC Control
        u_cmd = self.mpc.compute_control(
            x_follower=x_follower,
            d_meas=self.current_distance,
            v_follower=self.current_velocity,
            leader_throttle=self.leader_throttle,
            d_prev=self.prev_distance,
            u_prev_cmd=self.prev_u_cmd
        )

        # 3. Apply Deadband & Braking Logic
        compensated_cmd = 0.0

        if u_cmd > 0.01:
            # POSITIVE: Add friction deadband to start moving
            compensated_cmd = u_cmd + self.friction_deadband
        else:
            # NEGATIVE (Braking): Since we have no reverse/brakes, 
            # we simply output 0.0 (Neutral/Coast)
            compensated_cmd = 0.0

        # 4. Apply Offset (Ensure this is 0.0 unless your hardware requires it)
        final_cmd = compensated_cmd + self.throttle_offset
        
        # Clamp between 0.0 (Stop) and u_max (Forward)
        # We clamp min to 0.0 because you said no reverse/braking.
        cmd_out = float(np.clip(final_cmd, 0.0, self.mpc.u_max))

        # Publish
        msg = Float32()
        msg.data = cmd_out
        self.pub_throttle.publish(msg)
        
        # 5. Update History (Save the RAW MPC intent, not the boosted command)
        self.prev_u_cmd = u_cmd

    def stop_vehicle(self):
        msg = Float32()
        msg.data = 0.0 # Force zero output
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