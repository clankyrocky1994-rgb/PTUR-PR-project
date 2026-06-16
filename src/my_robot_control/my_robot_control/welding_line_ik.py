import math
from typing import List, Optional
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker

from tf2_ros import Buffer, TransformListener


class WeldingLineIK(Node):
    def __init__(self):
        super().__init__("welding_line_ik")

        # -----------------------------
        # Основные настройки робота
        # -----------------------------
        self.base_frame = "lbr_link_0"
        self.ee_link = "lbr_link_ee"      # если IK/TF ругается, попробуем lbr_link_7
        self.group_name = "arm"           # если не сработает, проверим имя planning group

        self.controller_action = (
            "/lbr/joint_trajectory_controller/follow_joint_trajectory"
        )

        self.joint_names = [
            "lbr_A1",
            "lbr_A2",
            "lbr_A3",
            "lbr_A4",
            "lbr_A5",
            "lbr_A6",
            "lbr_A7",
        ]

        # -----------------------------
        # XYZ-точки сварки
        # -----------------------------
        # Координаты в метрах относительно lbr_link_0.
        # Тут задаётся прямой шов в одной горизонтальной плоскости z = const.
        #
        # Пример:
        # x = 0.45 м от базы
        # y идёт от -0.20 до +0.20
        # z = 0.45 м высота
        #
        # Получается прямая линия в плоскости z = 0.45.
        self.start_xyz = [-0.45, -0.20, 0.45]
        self.end_xyz = [-0.45, 0.20, 0.45]

        self.tool_roll = 0.0
        self.tool_pitch = math.pi
        self.tool_yaw = 0.0


        # Количество точек вдоль сварочного шва.
        # Чем больше точек, тем ближе движение к прямой линии.
        self.num_points = 40

        # Общее время движения по шву
        self.total_time_sec = 12.0

        # Выполнять ли движение.
        # Для безопасности сначала False: только рисует и считает IK.
        # Потом запускай с параметром execute:=true.
        self.execute = self.declare_parameter("execute", False).value

        # -----------------------------
        # ROS-интерфейсы
        # -----------------------------
        self.ik_client = self.create_client(GetPositionIK, "/lbr/compute_ik")

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.controller_action,
        )

        marker_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.plan_marker_pub = self.create_publisher(
            Marker,
            "/welding_planned_line",
            marker_qos,
        )

        self.point_marker_pub = self.create_publisher(
            Marker,
            "/welding_points",
            marker_qos,
        )

        self.cartesian_points = []

        self.marker_timer = self.create_timer(0.5, self.republish_markers)

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/lbr/joint_states",
            self.joint_state_callback,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_joint_state: Optional[JointState] = None
        self.current_orientation = None

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

    def multiply_quaternions(self, q1, q2):
        q = Quaternion()

        q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        q.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        q.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w

        return q


    def yaw_quaternion(self, yaw_rad):
        q = Quaternion()

        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw_rad / 2.0)
        q.w = math.cos(yaw_rad / 2.0)

        return q


    def rotate_orientation_around_z(self, orientation, yaw_rad):
        yaw_q = self.yaw_quaternion(yaw_rad)
        return self.multiply_quaternions(yaw_q, orientation)

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg


    def republish_markers(self):
        if len(self.cartesian_points) > 0:
            self.publish_cartesian_markers(self.cartesian_points)

    def run(self):
        self.get_logger().info("Starting welding line planning")

        if not self.wait_for_data():
            self.get_logger().error("Required data was not received")
            return False

        cartesian_points = self.create_cartesian_line()
        self.cartesian_points = cartesian_points
        self.publish_cartesian_markers(cartesian_points)

        joint_points = self.compute_joint_trajectory(cartesian_points)

        if joint_points is None:
            self.get_logger().error("Failed to compute full IK trajectory")
            return False

        self.get_logger().info(
            f"IK trajectory ready: {len(joint_points)} joint points"
        )

        if not self.execute:
            self.get_logger().info(
                "execute:=false, so trajectory is only visualized. "
                "Run with execute:=true to move the robot."
            )
            return True

        self.send_joint_trajectory(joint_points)
        return True

    def wait_for_data(self) -> bool:
        self.get_logger().info("Waiting for /lbr/joint_states...")

        start_time = self.get_clock().now()

        while rclpy.ok() and self.latest_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > 10.0:
                self.get_logger().error("No /joint_states received")
                return False

        self.get_logger().info("Waiting for current end-effector orientation from TF...")

        start_time = self.get_clock().now()

        while rclpy.ok():
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.ee_link,
                    rclpy.time.Time(),
                )

                self.current_orientation = self.quaternion_from_rpy(
                    self.tool_roll,
                    self.tool_pitch,
                    self.tool_yaw,
                )

                # Если шов находится спереди робота в отрицательном X,
                # разворачиваем ориентацию инструмента на 180 градусов вокруг Z.
                if self.start_xyz[0] < 0.0 and self.end_xyz[0] < 0.0:
                    self.current_orientation = self.rotate_orientation_around_z(
                        self.current_orientation,
                        math.pi,
                    )

                    self.get_logger().info(
                        "Welding line is in front side x < 0, tool orientation rotated by 180 deg around Z"
                    )
                else:
                    self.get_logger().info(
                        f"Using current orientation of {self.ee_link} as welding orientation"
                    )
                return True

            except Exception as e:
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9

                if elapsed > 10.0:
                    self.get_logger().error(
                        f"Cannot get TF {self.base_frame} -> {self.ee_link}: {e}"
                    )
                    return False

                rclpy.spin_once(self, timeout_sec=0.1)

        return False

    def create_cartesian_line(self) -> List[Point]:
        points = []

        sx, sy, sz = self.start_xyz
        ex, ey, ez = self.end_xyz

        for i in range(self.num_points):
            t = i / float(self.num_points - 1)

            p = Point()
            p.x = sx + t * (ex - sx)
            p.y = sy + t * (ey - sy)
            p.z = sz + t * (ez - sz)

            points.append(p)

        self.get_logger().info(
            f"Created straight welding line in XYZ: "
            f"start={self.start_xyz}, end={self.end_xyz}, points={len(points)}"
        )

        return points

    def compute_joint_trajectory(self, cartesian_points: List[Point]):
        self.get_logger().info("Waiting for MoveIt /compute_ik service...")

        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                "MoveIt /compute_ik service not found. Is move_group running?"
            )
            return None

        joint_trajectory = []
        seed_state = self.make_seed_robot_state_from_current()

        for idx, p in enumerate(cartesian_points):
            pose = PoseStamped()
            pose.header.frame_id = self.base_frame
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = p.x
            pose.pose.position.y = p.y
            pose.pose.position.z = p.z

            # Ориентация инструмента постоянная.
            # Это важно для "сварки": горелка не должна крутиться на каждой точке.
            pose.pose.orientation = self.current_orientation

            request = GetPositionIK.Request()
            request.ik_request.group_name = self.group_name
            request.ik_request.ik_link_name = self.ee_link
            request.ik_request.pose_stamped = pose
            request.ik_request.robot_state = seed_state
            request.ik_request.avoid_collisions = True
            request.ik_request.timeout.sec = 1
            request.ik_request.timeout.nanosec = 0

            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            response = future.result()

            if response is None:
                self.get_logger().error(f"IK call failed at point {idx}")
                return None

            if response.error_code.val != response.error_code.SUCCESS:
                self.get_logger().error(
                    f"IK failed at point {idx}: "
                    f"x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, "
                    f"error_code={response.error_code.val}"
                )
                return None

            joint_positions = self.extract_lbr_joint_positions(
                response.solution.joint_state
            )

            if joint_positions is None:
                self.get_logger().error(
                    f"IK solution does not contain all required LBR joints at point {idx}"
                )
                return None

            joint_trajectory.append(joint_positions)

            # Следующую IK-точку считаем от предыдущего решения.
            # Так траектория получается более гладкой.
            seed_state = self.make_seed_robot_state_from_positions(joint_positions)

        return joint_trajectory

    def extract_lbr_joint_positions(self, joint_state: JointState):
        result = []

        for joint_name in self.joint_names:
            if joint_name not in joint_state.name:
                return None

            index = joint_state.name.index(joint_name)
            result.append(joint_state.position[index])

        return result

    def make_seed_robot_state_from_current(self) -> RobotState:
        robot_state = RobotState()
        robot_state.joint_state = self.latest_joint_state
        return robot_state

    def make_seed_robot_state_from_positions(self, positions) -> RobotState:
        robot_state = RobotState()

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = positions

        robot_state.joint_state = js
        return robot_state

    def send_joint_trajectory(self, joint_positions_list):
        self.get_logger().info("Waiting for joint trajectory action server...")

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f"Action server not found: {self.controller_action}"
            )
            rclpy.shutdown()
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        n = len(joint_positions_list)

        for i, positions in enumerate(joint_positions_list):
            t = i / float(n - 1)
            time_sec = t * self.total_time_sec

            point = JointTrajectoryPoint()
            point.positions = positions

            sec = int(math.floor(time_sec))
            nanosec = int((time_sec - sec) * 1e9)

            point.time_from_start = Duration(sec=sec, nanosec=nanosec)

            goal.trajectory.points.append(point)

        self.get_logger().info("Sending welding joint trajectory to controller")

        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Welding trajectory rejected")
            rclpy.shutdown()
            return

        self.get_logger().info("Welding trajectory accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f"Welding trajectory finished with error_code: {result.error_code}"
        )
        rclpy.shutdown()

    def publish_cartesian_markers(self, points: List[Point]):
        line_marker = Marker()
        line_marker.header.frame_id = self.base_frame
        line_marker.header.stamp = self.get_clock().now().to_msg()

        line_marker.ns = "welding_planned_line"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD

        line_marker.scale.x = 0.015

        # Зелёная линия — планируемый сварочный шов
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0

        line_marker.points = points

        points_marker = Marker()
        points_marker.header.frame_id = self.base_frame
        points_marker.header.stamp = self.get_clock().now().to_msg()

        points_marker.ns = "welding_points"
        points_marker.id = 1
        points_marker.type = Marker.SPHERE_LIST
        points_marker.action = Marker.ADD

        # Размер точек "как сварочные точки"
        points_marker.scale.x = 0.035
        points_marker.scale.y = 0.035
        points_marker.scale.z = 0.035

        # Оранжевые точки
        points_marker.color.r = 1.0
        points_marker.color.g = 0.45
        points_marker.color.b = 0.0
        points_marker.color.a = 1.0

        points_marker.points = points

        self.plan_marker_pub.publish(line_marker)
        self.point_marker_pub.publish(points_marker)

        self.get_logger().info(
            "Published planned welding line to /welding_planned_line "
            "and welding dots to /welding_points"
        )


def main(args=None):
    rclpy.init(args=args)

    node = WeldingLineIK()

    try:
        keep_alive = node.run()

        if keep_alive and rclpy.ok():
            rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()