import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from tf2_ros import Buffer, TransformListener


class MoveAToB(Node):
    def __init__(self):
        super().__init__("move_a_to_b")

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/lbr/joint_trajectory_controller/follow_joint_trajectory",
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

        # Углы в радианах.
        # Это точка A в пространстве суставов.
        self.point_a = [
            0.0,
            0.25,
            0.0,
            -0.50,
            0.0,
            0.35,
            0.0,
        ]

        # Это точка B в пространстве суставов.
        self.point_b = [
            0.35,
            0.35,
            0.0,
            -0.75,
            0.0,
            0.55,
            0.20,
        ]

        # TF: откуда и до какого звена смотреть траекторию.
        # Если не заработает, надо будет уточнить имена фреймов через:
        # ros2 run tf2_tools view_frames
        self.base_frame = "lbr_link_0"
        self.ee_frame = "lbr_link_ee"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(Marker, "/ee_trajectory", 10)

        self.trajectory_points = []
        self.record_trajectory = False

        self.timer = self.create_timer(0.05, self.update_marker)

    def make_duration(self, sec: int, nanosec: int = 0) -> Duration:
        duration = Duration()
        duration.sec = sec
        duration.nanosec = nanosec
        return duration

    def send_goal(self):
        self.get_logger().info("Waiting for joint trajectory action server...")

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "Action server not found. Is joint_trajectory_controller running?"
            )
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point_1 = JointTrajectoryPoint()
        point_1.positions = self.point_a
        point_1.time_from_start = self.make_duration(3)

        point_2 = JointTrajectoryPoint()
        point_2.positions = self.point_b
        point_2.time_from_start = self.make_duration(7)

        goal_msg.trajectory.points = [point_1, point_2]

        self.trajectory_points.clear()
        self.record_trajectory = True

        self.get_logger().info("Sending trajectory: A -> B")

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected")
            self.record_trajectory = False
            rclpy.shutdown()
            return

        self.get_logger().info("Trajectory goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.record_trajectory = False
        self.publish_marker()

        self.get_logger().info(f"Trajectory finished with error_code: {result.error_code}")
        self.get_logger().info("End-effector trajectory was published to /ee_trajectory")

        rclpy.shutdown()

    def update_marker(self):
        if not self.record_trajectory:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
            )
        except Exception as e:
            self.get_logger().warn(
                f"Cannot get TF {self.base_frame} -> {self.ee_frame}: {e}",
                throttle_duration_sec=2.0,
            )
            return

        p = Point()
        p.x = transform.transform.translation.x
        p.y = transform.transform.translation.y
        p.z = transform.transform.translation.z

        self.trajectory_points.append(p)
        self.publish_marker()

    def publish_marker(self):
        marker = Marker()

        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "end_effector_trajectory"
        marker.id = 0

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Толщина линии
        marker.scale.x = 0.01

        # Цвет линии: красный
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = self.trajectory_points

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    node = MoveAToB()
    node.send_goal()

    rclpy.spin(node)


if __name__ == "__main__":
    main()