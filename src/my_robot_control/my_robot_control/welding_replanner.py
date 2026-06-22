import math
import time
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, PoseStamped, Quaternion, PointStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

from robot_vision_msgs.msg import HandTargetArray


class LiveWeldingReplanner(Node):
    def __init__(self):
        super().__init__("live_welding_replanner")

        # Frames
        self.base_frame = "lbr_link_0"
        self.ee_link = "lbr_link_ee"

        # MoveIt / controller
        self.group_name = "arm"
        self.ik_service = "/lbr/compute_ik"
        self.action_name = "/lbr/joint_trajectory_controller/follow_joint_trajectory"

        self.joint_names = [
            "lbr_A1",
            "lbr_A2",
            "lbr_A3",
            "lbr_A4",
            "lbr_A5",
            "lbr_A6",
            "lbr_A7",
        ]

        # Welding line target
        self.start_xyz = [-0.60, -0.30, 0.45]
        self.end_xyz = [-0.60, 0.30, 0.45]

        # Live replanning settings
        self.num_points = 35
        self.total_time_sec = 6.5

        self.danger_distance = 0.25      # если рука ближе 25 см — перепланировать
        self.detour_distance = 0.15     # насколько уводить траекторию в сторону
        self.hand_marker_radius = 0.16

        self.replan_cooldown_sec = 1.5   # чтобы не отменяло 100 раз в секунду
        self.last_replan_time = None

        # State
        self.latest_joint_state: Optional[JointState] = None
        self.latest_hand: Optional[Point] = None
        self.goal_handle = None
        self.goal_active = False
        self.replanning_now = False

        self.current_path: List[Point] = []

        # ROS
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ik_client = self.create_client(GetPositionIK, self.ik_service)

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.action_name,
        )

        self.create_subscription(
            JointState,
            "/lbr/joint_states",
            self.joint_state_callback,
            10,
        )

        self.create_subscription(
            HandTargetArray,
            "/robot_vision/hands",
            self.hand_callback,
            10,
        )

        self.path_marker_pub = self.create_publisher(
            Marker,
            "/live_replanned_path",
            10,
        )

        self.hand_marker_pub = self.create_publisher(
            Marker,
            "/live_hand_marker",
            10,
        )

        self.timer = self.create_timer(0.10, self.timer_callback)
        self.executor_started = False
        self.get_logger().info("live_welding_replanner started")
        self.get_logger().info("This node cancels and replans trajectory while robot is moving")

    # -------------------------
    # callbacks
    # -------------------------

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def hand_callback(self, msg: HandTargetArray):
        if len(msg.hands) == 0:
            self.latest_hand = None
            return

        hand = msg.hands[0]

        p_cam = PointStamped()
        p_cam.header.frame_id = hand.header.frame_id
        p_cam.header.stamp = rclpy.time.Time().to_msg()
        p_cam.point.x = hand.centroid.x
        p_cam.point.y = hand.centroid.y
        p_cam.point.z = hand.centroid.z

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                p_cam.header.frame_id,
                rclpy.time.Time(),
            )

            p_base = do_transform_point(p_cam, tf)

            p = Point()
            p.x = p_base.point.x
            p.y = p_base.point.y
            p.z = p_base.point.z

            self.latest_hand = p
            self.publish_hand_marker(p)

        except Exception as e:
            self.get_logger().warn(
                f"Cannot transform hand to {self.base_frame}: {e}",
                throttle_duration_sec=2.0,
            )

    # -------------------------
    # main logic
    # -------------------------

    def start(self):
        self.get_logger().info("Waiting for robot data...")

        if not self.wait_for_robot():
            self.get_logger().error("Robot data not ready")
            return

        self.get_logger().info("Sending initial welding trajectory")

        start = self.xyz_to_point(self.start_xyz)
        end = self.xyz_to_point(self.end_xyz)

        path = self.make_straight_path(start, end, self.num_points)
        self.send_path(path)

    def timer_callback(self):
        if not self.goal_active:
            return

        if self.replanning_now:
            return

        if self.latest_hand is None:
            return

        if len(self.current_path) < 2:
            return

        # cooldown
        now = self.get_clock().now()
        if self.last_replan_time is not None:
            dt = (now - self.last_replan_time).nanoseconds / 1e9
            if dt < self.replan_cooldown_sec:
                return

        dist = self.distance_hand_to_path(self.latest_hand, self.current_path)

        if dist > self.danger_distance:
            return

        self.get_logger().warn(
            f"HAND TOO CLOSE: {dist:.3f} m. Live replanning now..."
        )

        self.replanning_now = True
        self.last_replan_time = now

        import threading
        threading.Thread(target=self.cancel_and_replan, daemon=True).start()

    def cancel_and_replan(self):
        if self.goal_handle is None:
            self.get_logger().warn("No goal handle, cannot cancel")
            self.replanning_now = False
            return

        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        try:
            result = future.result()
            self.get_logger().warn(
                f"Cancel requested. Goals canceling: {len(result.goals_canceling)}"
            )
        except Exception as e:
            self.get_logger().error(f"Cancel failed: {e}")
            self.replanning_now = False
            return

        import threading
        threading.Thread(target=self.replan_worker, daemon=True).start()


    def replan_worker(self):
        current_tcp = self.get_current_tcp_point()

        if current_tcp is None:
            self.get_logger().error("Cannot get current TCP, replanning failed")
            self.replanning_now = False
            return

        end = self.xyz_to_point(self.end_xyz)

        if self.latest_hand is None:
            new_path = self.make_straight_path(current_tcp, end, self.num_points)
        else:
            new_path = self.make_avoidance_path(
                current_tcp,
                end,
                self.latest_hand,
            )

        self.get_logger().warn("Sending new live-replanned trajectory")
        self.send_path(new_path)

        self.replanning_now = False

    # -------------------------
    # path generation
    # -------------------------

    def make_avoidance_path(self, start: Point, end: Point, hand: Point) -> List[Point]:
        distance, closest = self.distance_point_to_segment(hand, start, end)

        if distance > self.danger_distance:
            return self.make_straight_path(start, end, self.num_points)

        detour = self.make_detour_point(start, end, hand, closest)

        self.get_logger().warn(
            f"Detour point: x={detour.x:.3f}, y={detour.y:.3f}, z={detour.z:.3f}"
        )

        first = self.make_straight_path(start, detour, self.num_points // 2)
        second = self.make_straight_path(detour, end, self.num_points // 2)

        return first + second[1:]

    def make_detour_point(self, start: Point, end: Point, hand: Point, closest: Point) -> Point:
        dx = end.x - start.x
        dy = end.y - start.y

        length = math.sqrt(dx * dx + dy * dy)

        if length < 1e-6:
            p = Point()
            p.x = closest.x
            p.y = closest.y
            p.z = closest.z + self.detour_distance
            return p

        # перпендикуляр в XY
        px = -dy / length
        py = dx / length

        c1 = Point()
        c1.x = closest.x + px * self.detour_distance
        c1.y = closest.y + py * self.detour_distance
        c1.z = closest.z

        c2 = Point()
        c2.x = closest.x - px * self.detour_distance
        c2.y = closest.y - py * self.detour_distance
        c2.z = closest.z

        d1 = self.distance_points(c1, hand)
        d2 = self.distance_points(c2, hand)

        if d1 > d2:
            return c1

        return c2

    def make_straight_path(self, start: Point, end: Point, n: int) -> List[Point]:
        if n < 2:
            n = 2

        points = []

        for i in range(n):
            t = i / float(n - 1)

            p = Point()
            p.x = start.x + t * (end.x - start.x)
            p.y = start.y + t * (end.y - start.y)
            p.z = start.z + t * (end.z - start.z)

            points.append(p)

        return points

    # -------------------------
    # send trajectory
    # -------------------------

    def send_path(self, path: List[Point]):
        self.current_path = path
        self.publish_path_marker(path)

        joint_positions = self.compute_ik_path(path)

        if joint_positions is None:
            self.get_logger().error("IK failed, path not sent")
            return

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Action server not found: {self.action_name}")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        n = len(joint_positions)

        for i, positions in enumerate(joint_positions):
            t = i / float(n - 1)
            time_sec = t * self.total_time_sec

            sec = int(math.floor(time_sec))
            nanosec = int((time_sec - sec) * 1e9)

            pt = JointTrajectoryPoint()
            pt.positions = positions
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)

            goal.trajectory.points.append(pt)

        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error("Trajectory rejected")
            self.goal_active = False
            return

        self.goal_active = True
        self.get_logger().info("Trajectory accepted")

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.goal_active = False

        try:
            result = future.result().result
            self.get_logger().info(f"Trajectory finished, error_code={result.error_code}")
        except Exception as e:
            self.get_logger().warn(f"Trajectory result error: {e}")

    # -------------------------
    # IK
    # -------------------------

    def compute_ik_path(self, path: List[Point]):
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"IK service not found: {self.ik_service}")
            return None

        orientation = self.get_tool_orientation()
        seed = self.make_seed_from_current()

        result_positions = []

        for i, p in enumerate(path):
            pose = PoseStamped()
            pose.header.frame_id = self.base_frame
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = p.x
            pose.pose.position.y = p.y
            pose.pose.position.z = p.z
            pose.pose.orientation = orientation

            req = GetPositionIK.Request()
            req.ik_request.group_name = self.group_name
            req.ik_request.ik_link_name = self.ee_link
            req.ik_request.pose_stamped = pose
            req.ik_request.robot_state = seed
            req.ik_request.avoid_collisions = True
            req.ik_request.timeout.sec = 1
            req.ik_request.timeout.nanosec = 0

            future = self.ik_client.call_async(req)

            if self.executor_started:
                while rclpy.ok() and not future.done():
                    time.sleep(0.01)
            else:
                rclpy.spin_until_future_complete(self, future)

            res = future.result()

            if res is None:
                self.get_logger().error(f"IK call failed at point {i}")
                return None

            if res.error_code.val != res.error_code.SUCCESS:
                self.get_logger().error(
                    f"IK failed at point {i}: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, code={res.error_code.val}"
                )
                return None

            positions = self.extract_joint_positions(res.solution.joint_state)

            if positions is None:
                self.get_logger().error("IK solution missing LBR joints")
                return None

            result_positions.append(positions)
            seed = self.make_seed_from_positions(positions)

        return result_positions

    def extract_joint_positions(self, js: JointState):
        positions = []

        for name in self.joint_names:
            if name not in js.name:
                return None

            idx = js.name.index(name)
            positions.append(js.position[idx])

        return positions

    def make_seed_from_current(self):
        state = RobotState()
        state.joint_state = self.latest_joint_state
        return state

    def make_seed_from_positions(self, positions):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = positions

        state = RobotState()
        state.joint_state = js
        return state

    # -------------------------
    # helpers
    # -------------------------

    def wait_for_robot(self):
        start_time = self.get_clock().now()

        while rclpy.ok() and self.latest_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)

            dt = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if dt > 10.0:
                self.get_logger().error("No /lbr/joint_states")
                return False

        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f"No IK service: {self.ik_service}")
            return False

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"No trajectory action: {self.action_name}")
            return False

        return True

    def get_current_tcp_point(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_link,
                rclpy.time.Time(),
            )

            p = Point()
            p.x = tf.transform.translation.x
            p.y = tf.transform.translation.y
            p.z = tf.transform.translation.z
            return p

        except Exception as e:
            self.get_logger().error(f"Cannot get current TCP TF: {e}")
            return None

    def get_tool_orientation(self):
        # Такая же ориентация, как в welding_line_ik: инструмент вниз
        q = self.quaternion_from_rpy(0.0, math.pi, 0.0)

        if self.start_xyz[0] < 0.0 and self.end_xyz[0] < 0.0:
            q = self.rotate_orientation_around_z(q, math.pi)

        return q

    def xyz_to_point(self, xyz):
        p = Point()
        p.x = float(xyz[0])
        p.y = float(xyz[1])
        p.z = float(xyz[2])
        return p

    def distance_hand_to_path(self, hand: Point, path: List[Point]):
        min_dist = 999.0

        for i in range(len(path) - 1):
            d, _ = self.distance_point_to_segment(hand, path[i], path[i + 1])
            min_dist = min(min_dist, d)

        return min_dist

    def distance_point_to_segment(self, p: Point, a: Point, b: Point):
        abx = b.x - a.x
        aby = b.y - a.y
        abz = b.z - a.z

        apx = p.x - a.x
        apy = p.y - a.y
        apz = p.z - a.z

        ab_len2 = abx * abx + aby * aby + abz * abz

        if ab_len2 < 1e-9:
            return self.distance_points(p, a), a

        t = (apx * abx + apy * aby + apz * abz) / ab_len2
        t = max(0.0, min(1.0, t))

        closest = Point()
        closest.x = a.x + t * abx
        closest.y = a.y + t * aby
        closest.z = a.z + t * abz

        return self.distance_points(p, closest), closest

    def distance_points(self, a: Point, b: Point):
        return math.sqrt(
            (a.x - b.x) ** 2 +
            (a.y - b.y) ** 2 +
            (a.z - b.z) ** 2
        )

    # -------------------------
    # markers
    # -------------------------

    def publish_path_marker(self, path: List[Point]):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "live_replanned_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.025

        marker.color.r = 0.0
        marker.color.g = 0.2
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.points = path

        self.path_marker_pub.publish(marker)

    def publish_hand_marker(self, p: Point):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "live_hand"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = p.x
        marker.pose.position.y = p.y
        marker.pose.position.z = p.z
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.hand_marker_radius * 2.0
        marker.scale.y = self.hand_marker_radius * 2.0
        marker.scale.z = self.hand_marker_radius * 2.0

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.6

        self.hand_marker_pub.publish(marker)

    # -------------------------
    # quaternions
    # -------------------------

    def quaternion_from_rpy(self, roll, pitch, yaw):
        q = Quaternion()

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    def yaw_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def multiply_quaternions(self, q1, q2):
        q = Quaternion()

        q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        q.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        q.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w

        return q

    def rotate_orientation_around_z(self, orientation, yaw):
        yaw_q = self.yaw_quaternion(yaw)
        return self.multiply_quaternions(yaw_q, orientation)


def main(args=None):
    rclpy.init(args=args)

    node = LiveWeldingReplanner()

    try:
        node.start()
        node.executor_started = True
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
