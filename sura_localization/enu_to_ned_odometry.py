#!/usr/bin/env python3

from __future__ import annotations

from copy import deepcopy
import math

from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


ENU_TO_NED = (
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, -1.0),
)
ENU_TO_NED_QUATERNION = Quaternion(
    x=math.sqrt(0.5),
    y=math.sqrt(0.5),
    z=0.0,
    w=0.0,
)


class EnuToNedOdometry(Node):
    def __init__(self) -> None:
        super().__init__("enu_to_ned_odometry")

        self.declare_parameter("input_topic", "/cirtesub/sensors/gps/odometry")
        self.declare_parameter("output_topic", "/cirtesub/sensors/gps/odometry_ned")
        self.declare_parameter("frame_id", "world_ned")
        self.declare_parameter("child_frame_id", "")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self._publisher = self.create_publisher(Odometry, output_topic, 10)
        self._subscription = self.create_subscription(
            Odometry,
            input_topic,
            self._odometry_callback,
            10,
        )

        self.get_logger().info(f"Converting ENU odometry {input_topic} to NED {output_topic}")

    def _odometry_callback(self, msg: Odometry) -> None:
        converted = Odometry()
        converted.header = msg.header
        converted.header.frame_id = str(self.get_parameter("frame_id").value)
        converted.child_frame_id = str(self.get_parameter("child_frame_id").value)

        converted.pose.pose.position.x = msg.pose.pose.position.y
        converted.pose.pose.position.y = msg.pose.pose.position.x
        converted.pose.pose.position.z = -msg.pose.pose.position.z
        converted.pose.pose.orientation = self._rotate_orientation(msg.pose.pose.orientation)
        converted.pose.covariance = self._transform_covariance(msg.pose.covariance)

        converted.twist = deepcopy(msg.twist)
        converted.twist.twist.angular.x = msg.twist.twist.angular.y
        converted.twist.twist.angular.y = msg.twist.twist.angular.x
        converted.twist.twist.angular.z = -msg.twist.twist.angular.z
        converted.twist.covariance = self._transform_twist_angular_covariance(msg.twist.covariance)

        self._publisher.publish(converted)

    @staticmethod
    def _transform_vector(vector: Vector3) -> Vector3:
        converted = Vector3()
        converted.x = vector.y
        converted.y = vector.x
        converted.z = -vector.z
        return converted

    @staticmethod
    def _rotate_orientation(orientation: Quaternion) -> Quaternion:
        return EnuToNedOdometry._normalize_quaternion(
            EnuToNedOdometry._multiply_quaternions(ENU_TO_NED_QUATERNION, orientation)
        )

    @staticmethod
    def _multiply_quaternions(left: Quaternion, right: Quaternion) -> Quaternion:
        result = Quaternion()
        result.w = (
            left.w * right.w
            - left.x * right.x
            - left.y * right.y
            - left.z * right.z
        )
        result.x = (
            left.w * right.x
            + left.x * right.w
            + left.y * right.z
            - left.z * right.y
        )
        result.y = (
            left.w * right.y
            - left.x * right.z
            + left.y * right.w
            + left.z * right.x
        )
        result.z = (
            left.w * right.z
            + left.x * right.y
            - left.y * right.x
            + left.z * right.w
        )
        return result

    @staticmethod
    def _normalize_quaternion(quaternion: Quaternion) -> Quaternion:
        norm = math.sqrt(
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w
        )
        if norm <= 0.0:
            return Quaternion(w=1.0)

        quaternion.x /= norm
        quaternion.y /= norm
        quaternion.z /= norm
        quaternion.w /= norm
        return quaternion

    @staticmethod
    def _transform_covariance(covariance: list[float]) -> list[float]:
        transform = (
            (0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, -1.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0, 0.0, -1.0),
        )
        return [
            sum(
                transform[row][i] * covariance[i * 6 + j] * transform[col][j]
                for i in range(6)
                for j in range(6)
            )
            for row in range(6)
            for col in range(6)
        ]

    @staticmethod
    def _transform_twist_angular_covariance(covariance: list[float]) -> list[float]:
        transform = (
            (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0, 0.0, -1.0),
        )
        return [
            sum(
                transform[row][i] * covariance[i * 6 + j] * transform[col][j]
                for i in range(6)
                for j in range(6)
            )
            for row in range(6)
            for col in range(6)
        ]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EnuToNedOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
