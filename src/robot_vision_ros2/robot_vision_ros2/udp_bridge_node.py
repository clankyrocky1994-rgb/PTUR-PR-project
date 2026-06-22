#!/usr/bin/env python3
"""UDP -> ROS2 bridge for the Industrial Robot Vision Dashboard.

Listens to the JSON telemetry stream emitted by ``robot_vision_v3.py``
(RobotTelemetryStreamer, default 127.0.0.1:9090) and republishes it as ROS2
topics so a KUKA iiwa (or any ROS2 robot) can react to tracked hands.

Decoupled by design: the heavy vision app (camera, YOLO, MediaPipe, GUI) can
run on Windows, while this lightweight node runs on the Linux/robot side.
They only need to share a network — point the vision app's
``telemetry.udp_target`` at the host running this node.

Published topics
----------------
  /robot_vision/hands          robot_vision_msgs/HandTargetArray
  /robot_vision/<hand>/pose    geometry_msgs/PoseStamped   (position + orientation)
  /robot_vision/markers        visualization_msgs/MarkerArray  (RViz)
  + optional TF:  <frame_id> -> hand_<id>

All distances are converted mm -> m, areas mm^2 -> m^2, volumes mm^3 -> m^3.
"""
import json
import math
import socket
import threading

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import (
    Point, Vector3, Quaternion, PoseStamped, TransformStamped,
)
from visualization_msgs.msg import Marker, MarkerArray

from robot_vision_msgs.msg import HandTarget, HandTargetArray

try:
    from tf2_ros import TransformBroadcaster
    HAS_TF = True
except ImportError:
    HAS_TF = False

MM_TO_M = 0.001


def quaternion_from_direction(direction, ref=(1.0, 0.0, 0.0)) -> Quaternion:
    """Quaternion that rotates ``ref`` axis onto the (unit) direction vector.

    By default maps the robot tool's +X axis onto the hand pointing direction,
    so the resulting orientation can be fed straight into a PoseStamped goal.
    """
    d = np.asarray(direction, dtype=float)
    n = np.linalg.norm(d)
    if n < 1e-9:
        return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    d = d / n
    r = np.asarray(ref, dtype=float)
    v = np.cross(r, d)
    c = float(np.dot(r, d))
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        # Parallel (c>0) or anti-parallel (c<0)
        if c > 0:
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return Quaternion(x=0.0, y=0.0, z=1.0, w=0.0)
    axis = v / s
    half = math.atan2(s, c) / 2.0
    sin_h = math.sin(half)
    return Quaternion(
        x=float(axis[0] * sin_h), y=float(axis[1] * sin_h),
        z=float(axis[2] * sin_h), w=float(math.cos(half)),
    )


class HandBridge(Node):
    def __init__(self):
        super().__init__('robot_vision_bridge')

        self.declare_parameter('udp_ip', '0.0.0.0')
        self.declare_parameter('udp_port', 9090)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_markers', True)

        self.frame_id = self.get_parameter('frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value) and HAS_TF
        self.publish_markers = bool(self.get_parameter('publish_markers').value)

        self.hands_pub = self.create_publisher(HandTargetArray, 'robot_vision/hands', 10)
        self.marker_pub = (self.create_publisher(MarkerArray, 'robot_vision/markers', 10)
                           if self.publish_markers else None)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self._pose_pubs = {}

        ip = self.get_parameter('udp_ip').value
        port = int(self.get_parameter('udp_port').value)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((ip, port))
        self.sock.settimeout(0.5)
        self.get_logger().info(f'Listening for vision telemetry on udp://{ip}:{port}')

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ rx
    def _rx_loop(self):
        while not self._stop.is_set() and rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                packet = json.loads(data.decode('utf-8'))
            except (ValueError, UnicodeDecodeError):
                continue
            try:
                self._publish(packet)
            except Exception as exc:  # never kill the rx thread
                self.get_logger().warn(f'Failed to publish packet: {exc}')

    def _pose_pub(self, hand_id):
        if hand_id not in self._pose_pubs:
            safe = hand_id.lower().replace(' ', '_')
            self._pose_pubs[hand_id] = self.create_publisher(
                PoseStamped, f'robot_vision/{safe}/pose', 10)
        return self._pose_pubs[hand_id]

    # ------------------------------------------------------------- publish
    def _publish(self, packet):
        hands = packet.get('hands', {}) or {}
        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id=self.frame_id)

        arr = HandTargetArray()
        arr.header = header
        arr.safety_state = str(packet.get('safety_state', 'RUN'))

        markers = MarkerArray()
        mid = 0
        for hand_id, d in hands.items():
            centroid = Point(
                x=float(d.get('centroid_x_mm', d.get('x_mm', 0.0))) * MM_TO_M,
                y=float(d.get('centroid_y_mm', d.get('y_mm', 0.0))) * MM_TO_M,
                z=float(d.get('centroid_z_mm', d.get('z_mm', 0.0))) * MM_TO_M,
            )
            direction = (
                float(d.get('dir_x', 0.0)),
                float(d.get('dir_y', 0.0)),
                float(d.get('dir_z', 1.0)),
            )
            ttc_raw = d.get('ttc_sec', None)
            try:
                ttc = float(ttc_raw)
                if not math.isfinite(ttc):
                    ttc = -1.0
            except (TypeError, ValueError):
                ttc = -1.0

            t = HandTarget()
            t.header = header
            t.hand_id = str(hand_id)
            t.centroid = centroid
            t.direction = Vector3(x=direction[0], y=direction[1], z=direction[2])
            t.palm_normal = Vector3(
                x=float(d.get('normal_x', 0.0)),
                y=float(d.get('normal_y', 0.0)),
                z=float(d.get('normal_z', -1.0)),
            )
            t.angle_deg = float(d.get('angle_deg', 0.0))
            t.area_m2 = float(d.get('area_mm2', 0.0)) * 1e-6
            t.volume_m3 = float(d.get('volume_mm3', 0.0)) * 1e-9
            t.distance_z = centroid.z
            t.ttc = ttc
            t.confidence = float(d.get('confidence', 0.0))
            t.gesture = str(d.get('gesture', ''))
            arr.hands.append(t)

            orientation = quaternion_from_direction(direction)

            ps = PoseStamped()
            ps.header = header
            ps.pose.position = centroid
            ps.pose.orientation = orientation
            self._pose_pub(hand_id).publish(ps)

            if self.tf_broadcaster:
                tf = TransformStamped()
                tf.header = header
                tf.child_frame_id = f'hand_{t.hand_id.lower().replace(" ", "_")}'
                tf.transform.translation.x = centroid.x
                tf.transform.translation.y = centroid.y
                tf.transform.translation.z = centroid.z
                tf.transform.rotation = orientation
                self.tf_broadcaster.sendTransform(tf)

            if self.marker_pub is not None:
                mid = self._add_markers(markers, mid, header, centroid, direction, t.area_m2)

        self.hands_pub.publish(arr)
        if self.marker_pub is not None:
            self.marker_pub.publish(markers)

    @staticmethod
    def _add_markers(markers, mid, header, centroid, direction, area_m2):
        sphere = Marker()
        sphere.header = header
        sphere.ns = 'hand_com'
        sphere.id = mid
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = centroid
        sphere.pose.orientation.w = 1.0
        r = max(math.sqrt(max(area_m2, 0.0)), 0.03)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = r
        sphere.color.a = 0.6
        sphere.color.r = 1.0
        sphere.color.g = 1.0
        sphere.color.b = 0.0
        markers.markers.append(sphere)

        arrow = Marker()
        arrow.header = header
        arrow.ns = 'hand_dir'
        arrow.id = mid + 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        end = Point(
            x=centroid.x + direction[0] * 0.2,
            y=centroid.y + direction[1] * 0.2,
            z=centroid.z + direction[2] * 0.2,
        )
        arrow.points = [centroid, end]
        arrow.scale.x = 0.01   # shaft diameter
        arrow.scale.y = 0.02   # head diameter
        arrow.scale.z = 0.0
        arrow.color.a = 1.0
        arrow.color.r = 0.0
        arrow.color.g = 1.0
        arrow.color.b = 1.0
        markers.markers.append(arrow)
        return mid + 2

    def destroy_node(self):
        self._stop.set()
        try:
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
