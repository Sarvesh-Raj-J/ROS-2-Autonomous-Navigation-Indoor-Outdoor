#!/usr/bin/env python3
"""Republish an IMU with a configurable planar yaw correction."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quaternion_to_rpy(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class ImuYawOffsetter(Node):
    def __init__(self):
        super().__init__('imu_yaw_offsetter')
        self.declare_parameter('input_topic', '/gps/imu')
        self.declare_parameter('output_topic', '/gps/imu_heading')
        self.declare_parameter('yaw_offset', 0.0)
        self.declare_parameter('invert_yaw', False)
        self.declare_parameter('frame_id', '')
        self.declare_parameter('drop_zero_stamp', True)

        self.yaw_offset = float(self.get_parameter('yaw_offset').value)
        self.invert_yaw = bool(self.get_parameter('invert_yaw').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.drop_zero_stamp = bool(self.get_parameter('drop_zero_stamp').value)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.publisher = self.create_publisher(Imu, output_topic, 10)
        self.create_subscription(Imu, input_topic, self.imu_callback, 10)
        self.get_logger().info(
            'IMU yaw offsetter started: '
            f'{input_topic} -> {output_topic}, '
            f'invert_yaw={self.invert_yaw}, yaw_offset={self.yaw_offset:.4f} rad, '
            f'drop_zero_stamp={self.drop_zero_stamp}'
        )

    def imu_callback(self, msg: Imu):
        if (
            self.drop_zero_stamp
            and msg.header.stamp.sec == 0
            and msg.header.stamp.nanosec == 0
        ):
            self.get_logger().warn(
                'Dropping IMU message with zero timestamp',
                throttle_duration_sec=5.0,
            )
            return

        roll, pitch, yaw = quaternion_to_rpy(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        if self.invert_yaw:
            yaw = -yaw
        yaw += self.yaw_offset
        qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)

        corrected = Imu()
        corrected.header = msg.header
        if self.frame_id:
            corrected.header.frame_id = self.frame_id
        corrected.orientation.x = qx
        corrected.orientation.y = qy
        corrected.orientation.z = qz
        corrected.orientation.w = qw
        corrected.orientation_covariance = msg.orientation_covariance
        corrected.angular_velocity = msg.angular_velocity
        corrected.angular_velocity_covariance = msg.angular_velocity_covariance
        corrected.linear_acceleration = msg.linear_acceleration
        corrected.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.publisher.publish(corrected)


def main():
    rclpy.init()
    node = ImuYawOffsetter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
