import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
from tf2_ros import TransformException

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def linspace(lo, hi, n):
    if n <= 1:
        return [(lo + hi) * 0.5]
    step = (hi - lo) / (n - 1)
    return [lo + i * step for i in range(n)]


def simulate_trajectory(v, w, dt, horizon):
    x = 0.0
    y = 0.0
    th = 0.0
    states = [(x, y, th)]
    steps = max(1, int(horizon / dt))
    for _ in range(steps):
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w * dt
        states.append((x, y, th))
    return states


def wrap_to_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class DwaPlanner(Node):
    def __init__(self):
        super().__init__("dwa_planner")

        # TF listener (for map->base_footprint, base_scan->base_footprint, etc.)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/dwa_trajectories", 10)

        # State
        self.goal = None
        self.odom = None
        self.scan = None

        # ---------- Params ----------
        self.declare_parameter("control_dt", 0.1)
        self.declare_parameter("dt", 0.08)
        self.declare_parameter("horizon", 6.0)

        self.declare_parameter("front_angle_deg", 25.0)
        self.declare_parameter("front_blocked_dist", 0.55)
        self.declare_parameter("w_turn_blocked", 2.0)
        self.declare_parameter("w_min_turn", 0.6)

        self.declare_parameter("yaw_tolerance", 0.25)  # rad (~14 deg)
        self.declare_parameter("k_yaw", 1.8)
        self.declare_parameter("w_align_max", 1.2)

        self.declare_parameter("v_samples", 12)
        self.declare_parameter("w_samples", 19)

        self.declare_parameter("v_max", 0.22)
        self.declare_parameter("w_max", 2.84)

        self.declare_parameter("a_max", 0.5)
        self.declare_parameter("alpha_max", 5.0)

        # Cost weights
        self.declare_parameter("w_goal", 1.0)
        self.declare_parameter("w_obs", 2.0)
        self.declare_parameter("w_smooth", 0.2)
        self.declare_parameter("w_speed", 0.6)     # prefer higher linear speed
        self.declare_parameter("w_spin", 0.2)      # penalize high |w| and oscillation
        self.declare_parameter("w_heading", 0.8)   # penalize heading error at end of traj

        # Collision / safety
        self.declare_parameter("robot_radius", 0.11)
        self.declare_parameter("safety_margin", 0.02)
        self.declare_parameter("preferred_clearance", 0.06)
        self.declare_parameter("goal_tolerance", 0.20)

        # Scan processing
        self.declare_parameter("scan_stride", 1)

        # Visualization
        self.declare_parameter("viz_every_n", 1)
        self.declare_parameter("max_trajs_viz", 120)
        self.declare_parameter("v_min", -0.06)  # allow small reverse

        # Read params
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
        self.collision_dist = self.robot_radius + self.safety_margin

        self.preferred_clearance = float(self.get_parameter("preferred_clearance").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)

        self.scan_stride = int(self.get_parameter("scan_stride").value)

        self.viz_every_n = int(self.get_parameter("viz_every_n").value)
        self.max_trajs_viz = int(self.get_parameter("max_trajs_viz").value)

        # Internal
        self.prev_v_cmd = 0.0
        self.prev_w_cmd = 0.0
        self._tick = 0
        self._last_candidate_count = 0

        # Planner outputs (cached for markers)
        self.best_traj = None
        self.best_vw = (0.0, 0.0)

        self.timer = self.create_timer(self.control_dt, self.on_timer)
        self.get_logger().info("DWA planner started (goal input: /goal_pose)")

    def robot_yaw_in_map(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", Time())
            return yaw_from_quat(t.transform.rotation)
        except TransformException:
            return None

    def goal_cb(self, msg: PoseStamped):
        self.goal = msg
        self.get_logger().info(
            f"Goal received frame={msg.header.frame_id} "
            f"x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}"
        )

    def odom_cb(self, msg: Odometry):
        self.odom = msg

    def scan_cb(self, msg: LaserScan):
        self.scan = msg

    def front_min_range(self):
        if self.scan is None:
            return float("inf")

        a = math.radians(self.front_angle_deg)
        rmin = float("inf")
        stride = max(1, self.scan_stride)

        for i in range(0, len(self.scan.ranges), stride):
            r = float(self.scan.ranges[i])
            if not math.isfinite(r):
                continue
            if r <= float(self.scan.range_min) or r >= float(self.scan.range_max):
                continue

            ang = float(self.scan.angle_min) + i * float(self.scan.angle_increment)
            if -a <= ang <= a:
                rmin = min(rmin, r)

        return rmin

    def goal_in_base_xy(self):
        """Return (gx, gy) goal position expressed in base_footprint."""
        if self.goal is None:
            return None

        src = self.goal.header.frame_id or "map"

        try:
            t = self.tf_buffer.lookup_transform("base_footprint", src, Time())
        except TransformException as e:
            self.get_logger().warn(f"TF goal transform failed ({src} -> base_footprint): {e}")
            return None

        # Goal position in source frame
        xs = float(self.goal.pose.position.x)
        ys = float(self.goal.pose.position.y)

        # Transform src -> base_footprint
        tx = float(t.transform.translation.x)
        ty = float(t.transform.translation.y)
        yaw = yaw_from_quat(t.transform.rotation)
        c = math.cos(yaw)
        s = math.sin(yaw)

        xb = tx + c * xs - s * ys
        yb = ty + s * xs + c * ys
        return (xb, yb)

    def obstacle_points_in_base(self):
        """Convert LaserScan ranges to (x,y) points in base_footprint."""
        if self.scan is None:
            return []

        scan = self.scan
        scan_frame = scan.header.frame_id or "base_scan"

        try:
            tf = self.tf_buffer.lookup_transform("base_footprint", scan_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f"TF scan transform failed ({scan_frame} -> base_footprint): {e}")
            return []

        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        yaw = yaw_from_quat(tf.transform.rotation)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        pts = []
        stride = max(1, self.scan_stride)
        for i in range(0, len(scan.ranges), stride):
            r = float(scan.ranges[i])
            if not math.isfinite(r):
                continue
            if r <= float(scan.range_min) or r >= float(scan.range_max):
                continue

            ang = float(scan.angle_min) + i * float(scan.angle_increment)
            xs = r * math.cos(ang)
            ys = r * math.sin(ang)

            # Rotate+translate into base_footprint
            xb = tx + cy * xs - sy * ys
            yb = ty + sy * xs + cy * ys
            pts.append((xb, yb))

        return pts

    def min_dist_to_obstacles(self, traj_xy, obs_xy):
        """Return minimum Euclidean distance from any traj point to any obstacle point."""
        if not obs_xy:
            return float("inf")

        best = float("inf")
        for (x, y) in traj_xy:
            for (ox, oy) in obs_xy:
                d = math.hypot(x - ox, y - oy)
                if d < best:
                    best = d
        return best

    def evaluate_and_select(self):
        """Compute best (v,w) using costs in robot frame."""
        if self.odom is None or self.scan is None:
            return (0.0, 0.0), None, []

        gxy = self.goal_in_base_xy() if self.goal is not None else None
        if gxy is None:
            return (0.0, 0.0), None, []

        gx, gy = gxy
        obs = self.obstacle_points_in_base()
        front_blocked = (self.front_min_range() < self.front_blocked_dist)

        v_curr = float(self.odom.twist.twist.linear.x)
        w_curr = float(self.odom.twist.twist.angular.z)
        front_blocked = (self.front_min_range() < self.front_blocked_dist)

        # Dynamic window around current velocity
        v_lo = clamp(v_curr - self.a_max * self.control_dt, self.v_min, self.v_max)
        v_hi = clamp(v_curr + self.a_max * self.control_dt, self.v_min, self.v_max)
        w_lo = clamp(w_curr - self.alpha_max * self.control_dt, -self.w_max, self.w_max)
        w_hi = clamp(w_curr + self.alpha_max * self.control_dt, -self.w_max, self.w_max)
        if not front_blocked:
            v_lo = clamp(v_lo, 0.0, self.v_max)
        v_list = linspace(v_lo, v_hi, self.v_samples)
        w_list = linspace(w_lo, w_hi, self.w_samples)

        t0 = TimeMsg(sec=0, nanosec=0)

        best_cost = float("inf")
        best_vw = (0.0, 0.0)
        best_traj = None

        candidate_markers = []
        idx = 0

        for v in v_list:
            for w in w_list:
                if idx >= self.max_trajs_viz:
                    break

                traj = simulate_trajectory(v, w, self.dt, self.horizon)
                traj_xy = [(x, y) for (x, y, _) in traj]

                # --- Goal costs (robot frame) ---
                x_f, y_f, th_f = traj[-1]
                goal_dist = math.hypot(gx - x_f, gy - y_f)
                angle_to_goal = math.atan2(gy - y_f, gx - x_f)
                heading_err = abs(wrap_to_pi(angle_to_goal - th_f))

                # --- Obstacle distance (skip the start point) ---
                traj_xy_check = traj_xy[1:] if len(traj_xy) > 1 else traj_xy
                min_dist = self.min_dist_to_obstacles(traj_xy_check, obs)

                # Hard collision only if obstacle is inside robot body radius
                collision = (min_dist < self.robot_radius)

                if collision:
                    total = float("inf")
                    
                else:
                    clearance = max(1e-3, min_dist - self.robot_radius)

                    # Softer obstacle cost
                    if clearance >= self.preferred_clearance:
                        obs_cost = 0.0
                    else:
                        d = (self.preferred_clearance - clearance) / self.preferred_clearance
                        obs_cost = d * d

                    # Smoothness
                    smooth_cost = abs(v - v_curr) + abs(w - w_curr)

                    # Encourage forward progress + discourage spinning
                    speed_cost = (self.v_max - v)  # lower is better
                    spin_cost = abs(w) + 0.5 * abs(w - self.prev_w_cmd)
                    
                    dist_now = math.hypot(gx, gy)
                    dist_end = goal_dist
                    progress = dist_now - dist_end         # >0 means improvement
                    progress_cost = -progress

                    total = (
                        self.w_goal * goal_dist
                        + self.w_heading * heading_err
                        + self.w_obs * obs_cost
                        + self.w_smooth * smooth_cost
                        + self.w_speed * speed_cost
                        + self.w_spin * spin_cost + 0.8*progress_cost
                    )

                    # If blocked ahead, penalize not turning
                    if front_blocked and abs(w) < self.w_min_turn:
                        total += self.w_turn_blocked

                # Track best
                if total < best_cost:
                    best_cost = total
                    best_vw = (float(v), float(w))
                    best_traj = traj

                # Candidate marker
                m = Marker()
                m.header.frame_id = "base_footprint"
                m.header.stamp = t0
                m.frame_locked = True
                m.ns = "cand"
                m.id = idx
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                m.scale.x = 0.02  # line width

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

                pts = []
                for (x, y, _) in traj:
                    p = Point()
                    p.x = float(x)
                    p.y = float(y)
                    p.z = 0.02
                    pts.append(p)
                m.points = pts
                candidate_markers.append(m)

                idx += 1

        # Delete leftover candidate markers from previous cycle
        for j in range(idx, self._last_candidate_count):
            dm = Marker()
            dm.header.frame_id = "base_footprint"
            dm.header.stamp = t0
            dm.ns = "cand"
            dm.id = j
            dm.action = Marker.DELETE
            candidate_markers.append(dm)

        self._last_candidate_count = idx

        if best_traj is None and gxy is not None:
            ang = math.atan2(gy, gx)
            best_vw = (0.0, clamp(2.0 * ang, -self.w_max, self.w_max))
            best_traj = simulate_trajectory(best_vw[0], best_vw[1], self.dt, self.horizon)

        return best_vw, best_traj, candidate_markers

    def publish_markers(self, candidate_markers, best_traj):
        arr = MarkerArray()
        t0 = TimeMsg(sec=0, nanosec=0)

        # Robot arrow
        robot = Marker()
        robot.header.frame_id = "base_footprint"
        robot.header.stamp = t0
        robot.frame_locked = True
        robot.ns = "robot"
        robot.id = 0
        robot.type = Marker.ARROW
        robot.action = Marker.ADD
        robot.pose.orientation.w = 1.0
        robot.scale.x = 0.6
        robot.scale.y = 0.15
        robot.scale.z = 0.15
        robot.color.r = 0.0
        robot.color.g = 0.4
        robot.color.b = 1.0
        robot.color.a = 1.0
        robot.pose.position.z = 0.05
        arr.markers.append(robot)

        # Goal arrow (still in its original frame, RViz will TF it)
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

        # Candidates
        for m in candidate_markers:
            arr.markers.append(m)

        # Best trajectory (green)
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

    def on_timer(self):
        self._tick += 1

        best_vw, best_traj, candidate_markers = self.evaluate_and_select()
        self.best_vw = best_vw
        self.best_traj = best_traj

        if (self._tick % self.viz_every_n) == 0:
            self.publish_markers(candidate_markers, best_traj)

        # If close to goal position: align yaw first, otherwise stop.
        gxy = self.goal_in_base_xy() if self.goal is not None else None
        if gxy is not None:
            gx, gy = gxy
            if math.hypot(gx, gy) < self.goal_tolerance:
                robot_yaw = self.robot_yaw_in_map()
                goal_yaw = yaw_from_quat(self.goal.pose.orientation) if self.goal is not None else 0.0

                if robot_yaw is not None:
                    yaw_err = wrap_to_pi(goal_yaw - robot_yaw)
                    if abs(yaw_err) > self.yaw_tolerance:
                        cmd = Twist()
                        cmd.linear.x = 0.0
                        cmd.angular.z = clamp(self.k_yaw * yaw_err, -self.w_align_max, self.w_align_max)
                        self.cmd_pub.publish(cmd)
                        self.prev_v_cmd = cmd.linear.x
                        self.prev_w_cmd = cmd.angular.z
                        return

                cmd = Twist()  # stop when aligned (or if yaw TF unavailable)
                self.cmd_pub.publish(cmd)
                self.prev_v_cmd = 0.0
                self.prev_w_cmd = 0.0
                return

        # Normal driving
        cmd = Twist()
        cmd.linear.x = float(best_vw[0])
        cmd.angular.z = float(best_vw[1])
        self.cmd_pub.publish(cmd)
        self.prev_v_cmd = cmd.linear.x
        self.prev_w_cmd = cmd.angular.z


def main(args=None):
    rclpy.init(args=args)
    node = DwaPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
