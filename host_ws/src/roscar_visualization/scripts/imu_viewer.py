#!/usr/bin/env python3
# Copied from pi_ws/src/icm20948_ros2/icm20948_ros2/viewer.py.
# Original behavior retained; MIT license is included in the package LICENSE.
"""Display startup-relative IMU attitude using RViz's built-in markers.

imu_view is a visualization reference, not a robot TF frame or position estimate.
Only an isolated identity reference TF is published; no robot TF is changed.
"""

import math
import time

from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def make_visuals(imu, frame='imu_view'):
    if imu.orientation_covariance[0] < 0:
        raise ValueError('IMU message has no orientation; use /imu/data, not /imu/data_raw')
    values = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite IMU quaternion')
    length = math.sqrt(sum(v * v for v in values))
    if length < 1e-6:
        raise ValueError('Zero IMU quaternion')
    q = Quaternion()
    q.x, q.y, q.z, q.w = (v / length for v in values)
    pose = PoseStamped()
    pose.header.stamp = imu.header.stamp
    pose.header.frame_id = frame
    pose.pose.orientation = q
    array = MarkerArray()
    for index in range(4):
        marker = Marker()
        marker.header = pose.header
        marker.ns = 'imu_attitude'
        marker.id = index
        marker.action = Marker.ADD
        marker.pose.orientation = q
        marker.lifetime.nanosec = 300_000_000
        if index == 0:
            marker.type = Marker.CUBE
            marker.scale.x, marker.scale.y, marker.scale.z = .30, .18, .06
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = .65, .65, .7, .7
        else:
            marker.type = Marker.ARROW
            endpoint = [0.0, 0.0, 0.0]
            endpoint[index - 1] = .45
            marker.points = [Point(), Point(x=endpoint[0], y=endpoint[1], z=endpoint[2])]
            marker.scale.x, marker.scale.y, marker.scale.z = .012, .025, .04
            marker.color.r = 1.0 if index == 1 else 0.0
            marker.color.g = 1.0 if index == 2 else 0.0
            marker.color.b = 1.0 if index == 3 else 0.0
            marker.color.a = 1.0
        array.markers.append(marker)
    return pose, array


class ImuViewer(Node):
    def __init__(self):
        super().__init__('imu_viewer')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('display_frame', 'imu_view')
        self.frame = self.get_parameter('display_frame').value
        if not self.frame or self.frame.startswith('/'):
            raise ValueError('display_frame must be nonempty without a leading slash')
        # Register the virtual fixed frame with RViz without pretending this is
        # base_link->imu_link or an estimate of robot odometry.
        self.reference_broadcaster = StaticTransformBroadcaster(self)
        reference = TransformStamped()
        reference.header.stamp = self.get_clock().now().to_msg()
        reference.header.frame_id = self.frame
        reference.child_frame_id = self.frame + '_reference'
        reference.transform.rotation.w = 1.0
        self.reference_broadcaster.sendTransform(reference)
        self.pose_pub = self.create_publisher(PoseStamped, 'imu/pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'imu/markers', 10)
        self.subscription = self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._receive, qos_profile_sensor_data)
        self.last_publish = -math.inf
        self.get_logger().info(
            f'IMU visualization: red X, green Y, blue Z; virtual frame {self.frame}. '
            'Only an isolated display reference TF is published.')

    def _receive(self, message):
        now = time.monotonic()
        if now - self.last_publish < 1.0 / 30:
            return
        try:
            pose, markers = make_visuals(message, self.frame)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return
        self.pose_pub.publish(pose)
        self.marker_pub.publish(markers)
        self.last_publish = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuViewer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
