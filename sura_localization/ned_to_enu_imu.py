#!/usr/bin/env python3

from __future__ import annotations

import math

from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu
import rclpy
from rclpy.node import Node


NED_TO_ENU_QUATERNION = Quaternion(
    x=math.sqrt(0.5),
    y=math.sqrt(0.5),
    z=0.0,
    w=0.0,
)


class NedToEnuImu(Node):
    def __init__(self) -> None:
        super().__init__("ned_to_enu_imu")

        self.declare_parameter("input_topic", "/cirtesub/sensors/imu")
        self.declare_parameter("output_topic", "/cirtesub/sensors/imu_enu")
        self.declare_parameter("frame_id", "cirtesub/IMU")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self._publisher = self.create_publisher(Imu, output_topic, 10)
        self._subscription = self.create_subscription(
            Imu,
            input_topic,
            self._imu_callback,
            10,
        )

        self.get_logger().info(f"Converting NED IMU {input_topic} to ENU {output_topic}")

    def _imu_callback(self, msg: Imu) -> None:
        converted = Imu()
        converted.header = msg.header
        converted.header.frame_id = str(self.get_parameter("frame_id").value)

        converted.orientation = self._rotate_orientation(msg.orientation)
        converted.orientation_covariance = self._transform_3x3_covariance(
            msg.orientation_covariance
        )
        converted.angular_velocity = self._transform_vector(msg.angular_velocity)
        converted.angular_velocity_covariance = self._transform_3x3_covariance(
            msg.angular_velocity_covariance
        )
        converted.linear_acceleration = self._transform_vector(msg.linear_acceleration)
        converted.linear_acceleration_covariance = self._transform_3x3_covariance(
            msg.linear_acceleration_covariance
        )

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
        return NedToEnuImu._normalize_quaternion(
            NedToEnuImu._multiply_quaternions(NED_TO_ENU_QUATERNION, orientation)
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
    def _transform_3x3_covariance(covariance: list[float]) -> list[float]:
        transform = (
            (0.0, 1.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 0.0, -1.0),
        )
        return [
            sum(
                transform[row][i] * covariance[i * 3 + j] * transform[col][j]
                for i in range(3)
                for j in range(3)
            )
            for row in range(3)
            for col in range(3)
        ]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NedToEnuImu()
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
