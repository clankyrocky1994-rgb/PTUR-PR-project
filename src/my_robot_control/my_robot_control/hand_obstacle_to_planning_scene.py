import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from robot_vision_msgs.msg import HandTargetArray


class HandObstacleToPlanningScene(Node):
    def __init__(self):
        super().__init__("hand_obstacle_to_planning_scene")

        # ВАЖНО: эти точки должны совпадать с welding_line_ik.py
        self.start_xyz = [-0.45, -0.20, 0.45]
        self.end_xyz = [-0.45, 0.20, 0.45]

        # Радиус безопасной зоны вокруг руки
        self.obstacle_radius = 0.12

        # Если рука ближе этого расстояния к траектории — считаем, что пересекает
        self.warning_distance = 0.18

        self.latest_hand = None
        self.obstacle_active = False

        self.hand_sub = self.create_subscription(
            HandTargetArray,
            "/robot_vision/hands",
            self.hand_callback,
            10,
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/hand_obstacle_marker",
            10,
        )

        self.scene_client = self.create_client(
            ApplyPlanningScene,
            "/lbr/apply_planning_scene",
        )

        self.timer = self.create_timer(0.2, self.timer_callback)

        self.get_logger().info("hand_obstacle_to_planning_scene started")
        self.get_logger().info("Subscribing: /robot_vision/hands")
        self.get_logger().info("Publishing marker: /hand_obstacle_marker")
        self.get_logger().info("Using MoveIt service: /lbr/apply_planning_scene")

    def hand_callback(self, msg: HandTargetArray):
        if len(msg.hands) == 0:
            self.latest_hand = None
            return

        # Берём первую найденную руку
        hand = msg.hands[0]
        self.latest_hand = hand

    def timer_callback(self):
        if self.latest_hand is None:
            return

        hand = self.latest_hand
        # Координаты из камеры
        x_cam = hand.centroid.x
        y_cam = hand.centroid.y
        z_cam = hand.centroid.z

        camera_x = -0.80
        camera_y = 0.00
        camera_z = 0.70

        scale_x = 2.5
        scale_y = 2.5
        scale_z = 4.0

        # z_cam = глубина от камеры
        # x_cam = вправо/влево в кадре
        # y_cam = вверх/вниз в кадре

        x = camera_x - x_cam * 1
        y = camera_y - y_cam * 1
        z = camera_z + z_cam * 1
        

        frame_id = "lbr_link_0"

        distance = self.distance_point_to_segment(
            [x, y, z],
            self.start_xyz,
            self.end_xyz,
        )

        self.publish_hand_marker(frame_id, x, y, z)

        self.add_hand_collision_object(frame_id, x, y, z)

        if distance < self.warning_distance:
            self.get_logger().warn(
                f"РУКА ПЕРЕСЕКАЕТ/БЛИЗКО К ТРАЕКТОРИИ: distance={distance:.3f} m"
            )
        else:
            self.get_logger().info(
                f"hand ok: distance to trajectory = {distance:.3f} m"
            )

    def publish_hand_marker(self, frame_id, x, y, z):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "hand_obstacle"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.obstacle_radius * 2.0
        marker.scale.y = self.obstacle_radius * 2.0
        marker.scale.z = self.obstacle_radius * 2.0

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.6

        self.marker_pub.publish(marker)

    def add_hand_collision_object(self, frame_id, x, y, z):
        if not self.scene_client.service_is_ready():
            if not self.scene_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().warn(
                    "MoveIt service /lbr/apply_planning_scene not available",
                    throttle_duration_sec=2.0,
                )
                return

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.obstacle_radius]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = "hand_obstacle"
        obj.primitives.append(sphere)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)

        req = ApplyPlanningScene.Request()
        req.scene = scene

        future = self.scene_client.call_async(req)
        future.add_done_callback(self.scene_done_callback)

    def scene_done_callback(self, future):
        try:
            result = future.result()
            if not result.success:
                self.get_logger().warn("Failed to apply hand obstacle to PlanningScene")
        except Exception as e:
            self.get_logger().warn(f"PlanningScene call failed: {e}")

    def distance_point_to_segment(self, p, a, b):
        px, py, pz = p
        ax, ay, az = a
        bx, by, bz = b

        ab = [bx - ax, by - ay, bz - az]
        ap = [px - ax, py - ay, pz - az]

        ab_len2 = ab[0] ** 2 + ab[1] ** 2 + ab[2] ** 2

        if ab_len2 < 1e-9:
            return math.sqrt(
                (px - ax) ** 2 +
                (py - ay) ** 2 +
                (pz - az) ** 2
            )

        t = (
            ap[0] * ab[0] +
            ap[1] * ab[1] +
            ap[2] * ab[2]
        ) / ab_len2

        t = max(0.0, min(1.0, t))

        closest = [
            ax + t * ab[0],
            ay + t * ab[1],
            az + t * ab[2],
        ]

        return math.sqrt(
            (px - closest[0]) ** 2 +
            (py - closest[1]) ** 2 +
            (pz - closest[2]) ** 2
        )


def main(args=None):
    rclpy.init(args=args)
    node = HandObstacleToPlanningScene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()