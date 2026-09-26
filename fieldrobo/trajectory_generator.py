#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


def yaw_to_quaternion(yaw):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw


class TrajectoryGenerator(Node):

    def __init__(self):
        super().__init__('trajectory_generator')

        self.declare_parameter('trajectory_type', 'circle')
        self.declare_parameter('publish_frequency', 50.0)
        self.declare_parameter('approach_speed', 0.4)

        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('period', 30.0)

        self.trajectory_type = self.get_parameter(
            'trajectory_type'
        ).value
        self.publish_frequency = self.get_parameter(
            'publish_frequency'
        ).value
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.radius = self.get_parameter('radius').value
        self.period = self.get_parameter('period').value
        self.approach_speed = self.get_parameter('approach_speed').value

        self.reference_publisher = self.create_publisher(
            PoseStamped,
            '/reference_pose',
            10
        )

        self.start_time = self.get_clock().now()

        timer_period = 1.0 / self.publish_frequency
        self.timer = self.create_timer(
            timer_period,
            self.publish_trajectory
        )

        self.get_logger().info(
            f'Trajectory generator started: '
            f'{self.trajectory_type}, '
            f'R={self.radius:.3f} m, '
            f'T={self.period:.3f} s'
        )

    def publish_trajectory(self):
        now = self.get_clock().now()

        elapsed_time = (
            now - self.start_time
        ).nanoseconds / 1e9

        if self.trajectory_type == 'circle':

            # PHASE 1: Straight-line approach from spawn to circle
            approach_time = self.radius / self.approach_speed

            if elapsed_time < approach_time:

                # Move straight along +X from (0,0) to (R,0)
                x = self.approach_speed * elapsed_time
                y = 0.0

                # Keep robot pointing along +X
                yaw = 0.0

            # PHASE 2: Circular trajectory
            else:

                circle_time = elapsed_time - approach_time

                omega = 2.0 * math.pi / self.period
                angle = omega * circle_time

                x = self.center_x + self.radius * math.cos(angle)
                y = self.center_y + self.radius * math.sin(angle)

                # Tangent heading
                yaw = angle + math.pi / 2.0

        elif self.trajectory_type == 'stationary':

            x = self.center_x
            y = self.center_y
            yaw = 0.0

        else:
            self.get_logger().error(
                f'Unsupported trajectory_type: {self.trajectory_type}'
            )
            return

        msg = PoseStamped()

        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.reference_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryGenerator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()