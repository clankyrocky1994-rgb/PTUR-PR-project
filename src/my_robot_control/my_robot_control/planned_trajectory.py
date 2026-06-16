import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from tf2_ros import Buffer, TransformListener


class PlannedTrajectory(Node):
    def __init__(self):
        super().__init__("planned_trajectory")

        self.base_frame = "lbr_link_0"
        self.ee_frame = "lbr_link_ee"  # если не работает, замени на lbr_link_7

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(Marker, "/planned_trajectory", 10)

        self.planned_points = []
        self.created = False

        self.timer = self.create_timer(0.2, self.timer_callback)

        self.get_logger().info("Planned trajectory node started")

    def make_point(self, x, y, z):
        p = Point()
        p.x = x
        p.y = y
        p.z = z
        return p

    def interpolate(self, p1, p2, steps=30):
        points = []

        for i in range(steps):
            t = i / float(steps)

            p = Point()
            p.x = p1.x + t * (p2.x - p1.x)
            p.y = p1.y + t * (p2.y - p1.y)
            p.z = p1.z + t * (p2.z - p1.z)

            points.append(p)

        return points

    def timer_callback(self):
        if not self.created:
            self.create_full_planned_trajectory()

        if self.created:
            self.publish_marker()

    def create_full_planned_trajectory(self):
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

        start = self.make_point(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )

        # Вот тут задаём ВСЮ планируемую траекторию через несколько точек.
        # Все координаты относительно текущего положения конца робота.
        p1 = self.make_point(start.x + 0.20, start.y + 0.00, start.z + 0.10)
        p2 = self.make_point(start.x + 0.40, start.y + 0.15, start.z + 0.20)
        p3 = self.make_point(start.x + 0.60, start.y + 0.15, start.z + 0.10)
        p4 = self.make_point(start.x + 0.75, start.y - 0.10, start.z + 0.25)

        waypoints = [start, p1, p2, p3, p4]

        self.planned_points = []

        for i in range(len(waypoints) - 1):
            segment_points = self.interpolate(waypoints[i], waypoints[i + 1], steps=40)
            self.planned_points.extend(segment_points)

        self.planned_points.append(waypoints[-1])

        self.created = True

        self.get_logger().info(
            f"Full planned trajectory created: {len(self.planned_points)} points"
        )

    def publish_marker(self):
        marker = Marker()

        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "planned_trajectory"
        marker.id = 0

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Толщина линии
        marker.scale.x = 0.025

        # Зелёная линия
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = self.planned_points

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    node = PlannedTrajectory()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()