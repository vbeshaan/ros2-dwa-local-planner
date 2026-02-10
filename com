import math  # Trig, atan2, hypot, pi, radians, etc.

import rclpy
from rclpy.node import Node
from rclpy.time import Time  # rclpy Time object used for TF lookups

import tf2_ros
from tf2_ros import TransformException  # raised when TF lookup fails

# ROS message types we use (inputs + outputs + RViz visualization)
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


# ----------------------------
# Math / utility helper funcs
# ----------------------------

def yaw_from_quat(q):
    """
    Convert a quaternion orientation (q.x, q.y, q.z, q.w) into a planar yaw angle (rad).

    In 2D navigation, we usually care only about yaw (rotation about Z axis).
    """
    # These are intermediate terms for a standard quaternion->yaw conversion.
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    # atan2 returns an angle in [-pi, pi] with correct quadrant handling.
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(x, lo, hi):
    """
    Clamp x into [lo, hi].

    Used to enforce velocity limits and saturation (e.g., max angular rate).
    """
    return max(lo, min(hi, x))


def linspace(lo, hi, n):
    """
    Return n evenly spaced values from lo to hi inclusive.

    This is like numpy.linspace but implemented without NumPy.
    Used for sampling candidate v and w values inside the dynamic window.
    """
    if n <= 1:
        # If we only want one sample, use the midpoint.
        return [(lo + hi) * 0.5]

    # step so that: lo + (n-1)*step = hi
    step = (hi - lo) / (n - 1)

    # Return [lo, lo+step, lo+2*step, ..., hi]
    return [lo + i * step for i in range(n)]


def simulate_trajectory(v, w, dt, horizon):
    """
    Simulate (rollout) a trajectory using a simple unicycle/diff-drive kinematic model.

    Inputs:
      v: constant linear velocity command (m/s)
      w: constant angular velocity command (rad/s)
      dt: integration timestep for simulation (s)
      horizon: total simulated time ahead (s)

    Output:
      states: list of (x, y, theta) along the rollout, in the *robot frame*
              (because we start at x=y=theta=0 each time).
    """
    # Start at origin in robot frame.
    x = 0.0
    y = 0.0
    th = 0.0

    # Store the full trajectory (including the starting state).
    states = [(x, y, th)]

    # Number of integration steps for the requested horizon.
    # max(1, ...) ensures we always simulate at least one step.
    steps = max(1, int(horizon / dt))

    # Integrate forward in time.
    for _ in range(steps):
        # Move forward in the direction of current heading th.
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt

        # Update heading based on yaw rate.
        th += w * dt

        # Append the new simulated state.
        states.append((x, y, th))

    return states


def wrap_to_pi(a):
    """
    Wrap an angle (rad) to the interval [-pi, pi].

    This prevents angle differences from being 'too large' due to wraparound.
    Example: +179 degrees and -179 degrees should be treated as a 2-degree error,
    not a 358-degree error.
    """
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# ----------------------------
# Main DWA Planner Node
# ----------------------------

class DwaPlanner(Node):
    def __init__(self):
        # Initialize ROS2 node name.
        super().__init__("dwa_planner")

        # TF buffer/listener lets us query transforms between frames at runtime.
        # We need TF to express goal and scan points in the robot frame.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ----------------------------
        # Subscribers (inputs)
        # ----------------------------

        # Goal pose: user publishes PoseStamped to /goal_pose.
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_cb, 10
        )

        # Odometry: used to read current (v_curr, w_curr) and center the dynamic window.
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_cb, 10
        )

        # LaserScan: used to build obstacle points and detect frontal blocking.
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_cb, 10
        )

        # ----------------------------
        # Publishers (outputs)
        # ----------------------------

        # Velocity command output to robot (or simulation).
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # RViz visualization of candidate trajectories and best trajectory.
        self.marker_pub = self.create_publisher(
            MarkerArray, "/dwa_trajectories", 10
        )

        # ----------------------------
        # Cached state (latest messages)
        # ----------------------------

        # We store the most recent goal/odom/scan messages.
        # The planner uses these cached values each timer tick.
        self.goal = None
        self.odom = None
        self.scan = None

        # ----------------------------
        # ROS parameters (tunable knobs)
        # ----------------------------

        # Timing
        self.declare_parameter("control_dt", 0.1)  # how often planner runs (s)
        self.declare_parameter("dt", 0.08)         # simulation integration step (s)
        self.declare_parameter("horizon", 6.0)     # rollout time (s)

        # Front-cone "blocked" logic
        self.declare_parameter("front_angle_deg", 25.0)       # +/- cone half-angle
        self.declare_parameter("front_blocked_dist", 0.55)    # if obstacle closer than this => blocked
        self.declare_parameter("w_turn_blocked", 2.0)         # penalty if not turning while blocked
        self.declare_parameter("w_min_turn", 0.6)             # minimum |w| considered "turning"

        # Final goal yaw alignment behavior
        self.declare_parameter("yaw_tolerance", 0.25)  # rad; if yaw error smaller -> aligned
        self.declare_parameter("k_yaw", 1.8)           # P-controller gain for yaw alignment
        self.declare_parameter("w_align_max", 1.2)     # saturate yaw alignment angular speed

        # Sampling resolution
        self.declare_parameter("v_samples", 12)
        self.declare_parameter("w_samples", 19)

        # Limits (robot caps)
        self.declare_parameter("v_max", 0.22)
        self.declare_parameter("w_max", 2.84)

        # Dynamic constraints (acceleration limits)
        self.declare_parameter("a_max", 0.5)      # linear accel (m/s^2)
        self.declare_parameter("alpha_max", 5.0)  # angular accel (rad/s^2)

        # Cost weights (behavior shaping)
        self.declare_parameter("w_goal", 1.0)     # goal distance weight
        self.declare_parameter("w_obs", 2.0)      # obstacle avoidance weight (soft clearance penalty)
        self.declare_parameter("w_smooth", 0.2)   # penalize changing v,w too much
        self.declare_parameter("w_speed", 0.6)    # prefer higher v (faster forward progress)
        self.declare_parameter("w_spin", 0.2)     # penalize rotation and oscillation in w
        self.declare_parameter("w_heading", 0.8)  # penalize heading error at end of rollout

        # Safety / collision parameters
        self.declare_parameter("robot_radius", 0.11)        # approximate robot radius (m)
        self.declare_parameter("safety_margin", 0.02)       # extra margin (m)
        self.declare_parameter("preferred_clearance", 0.06) # clearance threshold where obs penalty becomes zero
        self.declare_parameter("goal_tolerance", 0.20)      # when within this distance -> do yaw alignment

        # LaserScan processing
        self.declare_parameter("scan_stride", 1)  # downsample scan points to reduce compute

        # Visualization
        self.declare_parameter("viz_every_n", 1)     # publish markers every n ticks
        self.declare_parameter("max_trajs_viz", 120) # cap number of candidate trajectories considered
        self.declare_parameter("v_min", -0.06)       # allow small reverse (m/s)

        # ----------------------------
        # Read params (store as python variables)
        # ----------------------------
        self.control_dt = float(self.get_parameter("control_dt").value)
        self.dt = float(self.get_parameter("dt").value)
        self.horizon = float(self.get_parameter("horizon").value)
        self.v_min = float(self.get_parameter("v_min").value)

        self.front_angle_deg = float(self.get_parameter("front_angle_deg").value)
        self.front_blocked_dist = float(self.get_parameter("front_blocked_dist").value)
        self.w_turn_blocked = float(self.get_parameter("w_turn_blocked").value)
        self.w_min_turn = float(self.get_parameter("w_min_turn").value)

        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.k_yaw = float(self.get_parameter("k_yaw").value)
        self.w_align_max = float(self.get_parameter("w_align_max").value)

        self.v_samples = int(self.get_parameter("v_samples").value)
        self.w_samples = int(self.get_parameter("w_samples").value)

        self.v_max = float(self.get_parameter("v_max").value)
        self.w_max = float(self.get_parameter("w_max").value)

        self.a_max = float(self.get_parameter("a_max").value)
        self.alpha_max = float(self.get_parameter("alpha_max").value)

        self.w_goal = float(self.get_parameter("w_goal").value)
        self.w_obs = float(self.get_parameter("w_obs").value)
        self.w_smooth = float(self.get_parameter("w_smooth").value)
        self.w_speed = float(self.get_parameter("w_speed").value)
        self.w_spin = float(self.get_parameter("w_spin").value)
        self.w_heading = float(self.get_parameter("w_heading").value)

        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.safety_margin = float(self.get_parameter("safety_margin").value)

        # A combined safety distance (you compute it here, but note: collision check later uses robot_radius)
        self.collision_dist = self.robot_radius + self.safety_margin

        self.preferred_clearance = float(self.get_parameter("preferred_clearance").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)

        self.scan_stride = int(self.get_parameter("scan_stride").value)

        self.viz_every_n = int(self.get_parameter("viz_every_n").value)
        self.max_trajs_viz = int(self.get_parameter("max_trajs_viz").value)

        # ----------------------------
        # Internal state for smoothing/visualization
        # ----------------------------

        # Previous commands (used in spin penalty + smoothness)
        self.prev_v_cmd = 0.0
        self.prev_w_cmd = 0.0

        # Tick counter for throttling marker publishing
        self._tick = 0

        # Used to delete old candidate markers in RViz
        self._last_candidate_count = 0

        # Cached planner outputs (useful for RViz / debugging)
        self.best_traj = None
        self.best_vw = (0.0, 0.0)

        # Main control loop timer
        self.timer = self.create_timer(self.control_dt, self.on_timer)
        self.get_logger().info("DWA planner started (goal input: /goal_pose)")

    # ----------------------------
    # Small TF helper
    # ----------------------------
    def robot_yaw_in_map(self):
        """
        Read robot yaw in the map frame using TF: map -> base_footprint.
        Returns yaw (rad) or None if TF unavailable.
        """
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", Time())
            return yaw_from_quat(t.transform.rotation)
        except TransformException:
            return None

    # ----------------------------
    # Subscriber callbacks (just cache messages)
    # ----------------------------
    def goal_cb(self, msg: PoseStamped):
        """Store the latest goal and log it."""
        self.goal = msg
        self.get_logger().info(
            f"Goal received frame={msg.header.frame_id} "
            f"x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}"
        )

    def odom_cb(self, msg: Odometry):
        """Store the latest odometry."""
        self.odom = msg

    def scan_cb(self, msg: LaserScan):
        """Store the latest laser scan."""
        self.scan = msg

    # ----------------------------
    # LaserScan utilities
    # ----------------------------
    def front_min_range(self):
        """
        Compute the minimum valid scan range within a front cone.

        Used for "front_blocked" logic:
          if front_min_range < front_blocked_dist => blocked => force turning behavior.
        """
        if self.scan is None:
            return float("inf")  # no scan => treat as no obstacle information

        # Convert degrees to radians for comparing to scan angles.
        a = math.radians(self.front_angle_deg)

        rmin = float("inf")
        stride = max(1, self.scan_stride)  # allow scan downsampling

        for i in range(0, len(self.scan.ranges), stride):
            r = float(self.scan.ranges[i])

            # Skip invalid scan readings (NaN/inf)
            if not math.isfinite(r):
                continue

            # Skip readings outside sensor valid range
            if r <= float(self.scan.range_min) or r >= float(self.scan.range_max):
                continue

            # Compute the angle of this ray: angle_min + i*angle_increment
            ang = float(self.scan.angle_min) + i * float(self.scan.angle_increment)

            # Only consider rays in front cone [-a, +a]
            if -a <= ang <= a:
                rmin = min(rmin, r)

        return rmin

    # ----------------------------
    # Goal transform utility
    # ----------------------------
    def goal_in_base_xy(self):
        """
        Return goal position (gx, gy) expressed in base_footprint.

        Why this is useful:
          - Trajectory rollout is done in robot frame (starts at 0,0,0),
          - So costs should be computed in the same frame.
        """
        if self.goal is None:
            return None

        # Source frame for the goal PoseStamped; default to map if empty.
        src = self.goal.header.frame_id or "map"

        try:
            # We want a transform that allows expressing src-frame coordinates in base_footprint.
            t = self.tf_buffer.lookup_transform("base_footprint", src, Time())
        except TransformException as e:
            self.get_logger().warn(f"TF goal transform failed ({src} -> base_footprint): {e}")
            return None

        # Goal position in its own frame (src)
        xs = float(self.goal.pose.position.x)
        ys = float(self.goal.pose.position.y)

        # Use the transform translation + yaw rotation (planar approximation)
        tx = float(t.transform.translation.x)
        ty = float(t.transform.translation.y)
        yaw = yaw_from_quat(t.transform.rotation)

        c = math.cos(yaw)
        s = math.sin(yaw)

        # Rotate+translate (src -> base_footprint) in 2D
        xb = tx + c * xs - s * ys
        yb = ty + s * xs + c * ys

        return (xb, yb)

    # ----------------------------
    # Obstacle transform utility
    # ----------------------------
    def obstacle_points_in_base(self):
        """
        Convert LaserScan readings into (x,y) obstacle points in base_footprint.

        Steps:
          1) For each scan ray: convert polar (r, ang) -> Cartesian (xs, ys) in scan frame.
          2) TF transform scan frame -> base_footprint (rotate+translate).
        """
        if self.scan is None:
            return []

        scan = self.scan
        scan_frame = scan.header.frame_id or "base_scan"

        try:
            tf = self.tf_buffer.lookup_transform("base_footprint", scan_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f"TF scan transform failed ({scan_frame} -> base_footprint): {e}")
            return []

        # Transform parameters
        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        yaw = yaw_from_quat(tf.transform.rotation)

        cy = math.cos(yaw)
        sy = math.sin(yaw)

        pts = []
        stride = max(1, self.scan_stride)

        for i in range(0, len(scan.ranges), stride):
            r = float(scan.ranges[i])

            # Skip invalid ranges
            if not math.isfinite(r):
                continue
            if r <= float(scan.range_min) or r >= float(scan.range_max):
                continue

            # Angle of i-th ray in scan frame
            ang = float(scan.angle_min) + i * float(scan.angle_increment)

            # Convert ray to Cartesian in scan frame
            xs = r * math.cos(ang)
            ys = r * math.sin(ang)

            # Rotate+translate into base_footprint
            xb = tx + cy * xs - sy * ys
            yb = ty + sy * xs + cy * ys

            pts.append((xb, yb))

        return pts

    # ----------------------------
    # Geometry / collision utility
    # ----------------------------
    def min_dist_to_obstacles(self, traj_xy, obs_xy):
        """
        Compute minimum Euclidean distance between ANY trajectory point and ANY obstacle point.

        Note: This is O(N_traj * N_obs). It's simple but can be expensive at high rates.
        """
        if not obs_xy:
            return float("inf")  # no obstacles => infinitely far away

        best = float("inf")
        for (x, y) in traj_xy:
            for (ox, oy) in obs_xy:
                d = math.hypot(x - ox, y - oy)
                if d < best:
                    best = d
        return best

    # ----------------------------
    # Main DWA evaluation
    # ----------------------------
    def evaluate_and_select(self):
        """
        Compute the best (v, w) using DWA-style sampling, rollout, and scoring.

        Returns:
          best_vw: (v, w) floats
          best_traj: list of (x, y, th) for the best candidate
          candidate_markers: list of Marker objects for RViz visualization
        """
        # Must have odometry and scan data to plan safely.
        if self.odom is None or self.scan is None:
            return (0.0, 0.0), None, []

        # Goal in robot frame (base_footprint)
        gxy = self.goal_in_base_xy() if self.goal is not None else None
        if gxy is None:
            return (0.0, 0.0), None, []

        gx, gy = gxy

        # Convert scan into obstacle points in base_footprint
        obs = self.obstacle_points_in_base()

        # Decide if we are "blocked" in front (for forcing a turn)
        front_blocked = (self.front_min_range() < self.front_blocked_dist)

        # Current measured velocities (center of dynamic window)
        v_curr = float(self.odom.twist.twist.linear.x)
        w_curr = float(self.odom.twist.twist.angular.z)

        # (recomputed again in your code; harmless but redundant)
        front_blocked = (self.front_min_range() < self.front_blocked_dist)

        # ----------------------------
        # Dynamic window calculation
        # ----------------------------
        # Limit candidates to what the robot can reach in one control_dt given accel limits.
        v_lo = clamp(v_curr - self.a_max * self.control_dt, self.v_min, self.v_max)
        v_hi = clamp(v_curr + self.a_max * self.control_dt, self.v_min, self.v_max)
        w_lo = clamp(w_curr - self.alpha_max * self.control_dt, -self.w_max, self.w_max)
        w_hi = clamp(w_curr + self.alpha_max * self.control_dt, -self.w_max, self.w_max)

        # If not blocked ahead, disallow reverse by forcing v_lo >= 0.
        if not front_blocked:
            v_lo = clamp(v_lo, 0.0, self.v_max)

        # Sample candidate velocities inside the dynamic window.
        v_list = linspace(v_lo, v_hi, self.v_samples)
        w_list = linspace(w_lo, w_hi, self.w_samples)

        # Marker time: using a 0 stamp (RViz often still displays with frame_locked).
        t0 = TimeMsg(sec=0, nanosec=0)

        # Track best candidate
        best_cost = float("inf")
        best_vw = (0.0, 0.0)
        best_traj = None

        # Candidate visualization markers
        candidate_markers = []

        # idx is a running ID for markers (and used to cap number of candidates)
        idx = 0

        # ----------------------------
        # Evaluate all candidates
        # ----------------------------
        for v in v_list:
            for w in w_list:
                # Cap number of candidates for performance / RViz
                if idx >= self.max_trajs_viz:
                    break

                # Rollout trajectory in robot frame
                traj = simulate_trajectory(v, w, self.dt, self.horizon)
                traj_xy = [(x, y) for (x, y, _) in traj]

                # -------- Goal costs --------
                # Evaluate distance + heading error at trajectory end
                x_f, y_f, th_f = traj[-1]
                goal_dist = math.hypot(gx - x_f, gy - y_f)

                angle_to_goal = math.atan2(gy - y_f, gx - x_f)
                heading_err = abs(wrap_to_pi(angle_to_goal - th_f))

                # -------- Obstacle distance --------
                # Skip the first point so we don't consider the robot "colliding with itself"
                traj_xy_check = traj_xy[1:] if len(traj_xy) > 1 else traj_xy
                min_dist = self.min_dist_to_obstacles(traj_xy_check, obs)

                # Hard collision check: reject if an obstacle point is inside robot radius
                collision = (min_dist < self.robot_radius)

                if collision:
                    total = float("inf")  # invalid candidate
                else:
                    # Clearance from robot body to nearest obstacle point
                    clearance = max(1e-3, min_dist - self.robot_radius)

                    # Soft obstacle cost:
                    # - If clearance >= preferred_clearance => no penalty
                    # - Else quadratic penalty increasing as clearance decreases
                    if clearance >= self.preferred_clearance:
                        obs_cost = 0.0
                    else:
                        d = (self.preferred_clearance - clearance) / self.preferred_clearance
                        obs_cost = d * d

                    # Smoothness cost: prefer not changing v,w too much from current odom
                    smooth_cost = abs(v - v_curr) + abs(w - w_curr)

                    # Speed cost: prefer larger v (because (v_max - v) smaller)
                    speed_cost = (self.v_max - v)

                    # Spin cost: penalize large |w| and changes in angular command (oscillation)
                    spin_cost = abs(w) + 0.5 * abs(w - self.prev_w_cmd)

                    # Progress reward: prefer candidates that reduce goal distance
                    dist_now = math.hypot(gx, gy)  # current goal distance (in base frame)
                    dist_end = goal_dist           # end-of-trajectory goal distance
                    progress = dist_now - dist_end # >0 means improvement
                    progress_cost = -progress      # negative cost => reward

                    # Total cost (weighted sum)
                    total = (
                        self.w_goal * goal_dist
                        + self.w_heading * heading_err
                        + self.w_obs * obs_cost
                        + self.w_smooth * smooth_cost
                        + self.w_speed * speed_cost
                        + self.w_spin * spin_cost
                        + 0.8 * progress_cost
                    )

                    # If blocked ahead, heavily penalize "not turning"
                    if front_blocked and abs(w) < self.w_min_turn:
                        total += self.w_turn_blocked

                # Track best candidate (minimum cost)
                if total < best_cost:
                    best_cost = total
                    best_vw = (float(v), float(w))
                    best_traj = traj

                # ----------------------------
                # RViz candidate marker creation
                # ----------------------------
                m = Marker()
                m.header.frame_id = "base_footprint"
                m.header.stamp = t0
                m.frame_locked = True

                # Namespace and ID uniquely identify the marker in RViz.
                # We use idx so each candidate gets a unique ID in ns="cand".
                m.ns = "cand"
                m.id = idx

                # Draw as a line strip (polyline)
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                m.scale.x = 0.02  # line width

                # Color collision candidates red, valid ones gray
                if collision:
                    m.color.r = 1.0
                    m.color.g = 0.0
                    m.color.b = 0.0
                    m.color.a = 0.35
                else:
                    m.color.r = 0.7
                    m.color.g = 0.7
                    m.color.b = 0.7
                    m.color.a = 0.6

                # Convert the trajectory states into Marker points
                pts = []
                for (x, y, _) in traj:
                    p = Point()
                    p.x = float(x)
                    p.y = float(y)
                    p.z = 0.02
                    pts.append(p)
                m.points = pts

                candidate_markers.append(m)

                # Increment marker/candidate counter
                idx += 1

        # Delete leftover candidate markers from previous cycle so RViz doesn't keep old lines.
        for j in range(idx, self._last_candidate_count):
            dm = Marker()
            dm.header.frame_id = "base_footprint"
            dm.header.stamp = t0
            dm.ns = "cand"
            dm.id = j
            dm.action = Marker.DELETE
            candidate_markers.append(dm)

        # Remember how many candidates we produced this tick.
        self._last_candidate_count = idx

        # Fallback: if no trajectory was selected, turn toward goal direction.
        if best_traj is None and gxy is not None:
            ang = math.atan2(gy, gx)
            best_vw = (0.0, clamp(2.0 * ang, -self.w_max, self.w_max))
            best_traj = simulate_trajectory(best_vw[0], best_vw[1], self.dt, self.horizon)

        return best_vw, best_traj, candidate_markers

    # ----------------------------
    # RViz publishing
    # ----------------------------
    def publish_markers(self, candidate_markers, best_traj):
        """
        Publish a MarkerArray containing:
          - robot arrow (blue)
          - goal arrow (orange)
          - candidate trajectories (gray/red)
          - best trajectory (green)
        """
        arr = MarkerArray()
        t0 = TimeMsg(sec=0, nanosec=0)

        # Robot arrow (visual only)
        robot = Marker()
        robot.header.frame_id = "base_footprint"
        robot.header.stamp = t0
        robot.frame_locked = True
        robot.ns = "robot"
        robot.id = 0
        robot.type = Marker.ARROW
        robot.action = Marker.ADD
        robot.pose.orientation.w = 1.0  # arrow faces forward in base frame
        robot.scale.x = 0.6
        robot.scale.y = 0.15
        robot.scale.z = 0.15
        robot.color.r = 0.0
        robot.color.g = 0.4
        robot.color.b = 1.0
        robot.color.a = 1.0
        robot.pose.position.z = 0.05
        arr.markers.append(robot)

        # Goal arrow: publish in the goal's original frame; RViz will TF it.
        if self.goal is not None:
            goal = Marker()
            goal.header.frame_id = self.goal.header.frame_id or "map"
            goal.header.stamp = t0
            goal.ns = "goal"
            goal.id = 0
            goal.type = Marker.ARROW
            goal.action = Marker.ADD
            goal.pose = self.goal.pose
            goal.scale.x = 0.5
            goal.scale.y = 0.10
            goal.scale.z = 0.10
            goal.color.r = 1.0
            goal.color.g = 0.6
            goal.color.b = 0.0
            goal.color.a = 1.0
            arr.markers.append(goal)

        # Candidate trajectories
        for m in candidate_markers:
            arr.markers.append(m)

        # Best trajectory (green, thicker)
        if best_traj is not None:
            best = Marker()
            best.header.frame_id = "base_footprint"
            best.header.stamp = t0
            best.frame_locked = True
            best.ns = "best"
            best.id = 0
            best.type = Marker.LINE_STRIP
            best.action = Marker.ADD
            best.scale.x = 0.05
            best.color.r = 0.0
            best.color.g = 1.0
            best.color.b = 0.0
            best.color.a = 1.0

            pts = []
            for (x, y, _) in best_traj:
                p = Point()
                p.x = float(x)
                p.y = float(y)
                p.z = 0.03
                pts.append(p)
            best.points = pts
            arr.markers.append(best)

        self.marker_pub.publish(arr)

    # ----------------------------
    # Control loop (runs every control_dt)
    # ----------------------------
    def on_timer(self):
        """
        Periodic planner tick:
          1) Run DWA evaluation and publish RViz markers
          2) If near goal, do yaw alignment
          3) Else publish best (v,w)
        """
        self._tick += 1

        # Compute best velocity command + candidate markers for visualization
        best_vw, best_traj, candidate_markers = self.evaluate_and_select()
        self.best_vw = best_vw
        self.best_traj = best_traj

        # Publish markers at configured rate
        if (self._tick % self.viz_every_n) == 0:
            self.publish_markers(candidate_markers, best_traj)

        # If close enough to goal position: align yaw first, then stop.
        gxy = self.goal_in_base_xy() if self.goal is not None else None
        if gxy is not None:
            gx, gy = gxy
            if math.hypot(gx, gy) < self.goal_tolerance:
                # Robot yaw from TF map->base_footprint
                robot_yaw = self.robot_yaw_in_map()

                # Goal yaw from goal pose quaternion
                goal_yaw = yaw_from_quat(self.goal.pose.orientation) if self.goal is not None else 0.0

                # If we can read robot_yaw, do a P-controller alignment
                if robot_yaw is not None:
                    yaw_err = wrap_to_pi(goal_yaw - robot_yaw)

                    # If not aligned yet, rotate in place
                    if abs(yaw_err) > self.yaw_tolerance:
                        cmd = Twist()
                        cmd.linear.x = 0.0
                        cmd.angular.z = clamp(
                            self.k_yaw * yaw_err,
                            -self.w_align_max,
                            self.w_align_max
                        )
                        self.cmd_pub.publish(cmd)
                        self.prev_v_cmd = cmd.linear.x
                        self.prev_w_cmd = cmd.angular.z
                        return

                # If aligned (or TF unavailable), stop at the goal
                cmd = Twist()
                self.cmd_pub.publish(cmd)
                self.prev_v_cmd = 0.0
                self.prev_w_cmd = 0.0
                return

        # Normal driving: publish the best DWA-selected (v,w)
        cmd = Twist()
        cmd.linear.x = float(best_vw[0])
        cmd.angular.z = float(best_vw[1])
        self.cmd_pub.publish(cmd)

        # Store previous command for next tick's smoothness/spin penalties
        self.prev_v_cmd = cmd.linear.x
        self.prev_w_cmd = cmd.angular.z


def main(args=None):
    """
    ROS2 node entry point.
    """
    rclpy.init(args=args)
    node = DwaPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
