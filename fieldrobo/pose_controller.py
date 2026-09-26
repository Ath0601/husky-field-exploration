#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PoseController(Node):
    """
    Task 1 controller.

    One controller supports both:
      1. stationary pose-to-pose references, and
      2. moving trajectory references.

    For a moving reference, a feed-forward + feedback unicycle controller
    is used:
        v     = v_ref*cos(e_theta) + k_x*e_x
        omega = omega_ref + k_y*v_ref*e_y + k_theta*sin(e_theta)

    where position errors are expressed in the robot body frame.
    """

    def __init__(self):
        super().__init__('pose_controller')

        # Feedback gains
        self.declare_parameter('k_x', 0.8)
        self.declare_parameter('k_y', 1.5)
        self.declare_parameter('k_theta', 2.0)

        # Pose-regulation gains
        self.declare_parameter('k_rho', 0.8)
        self.declare_parameter('k_alpha', 2.0)
        self.declare_parameter('k_beta', 0.8)

        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance', 0.05)

        self.declare_parameter('max_linear_velocity', 0.6)
        self.declare_parameter('max_angular_velocity', 1.5)

        self.declare_parameter('control_frequency', 50.0)

        # Reference velocity estimation
        self.declare_parameter('reference_velocity_timeout', 0.15)
        self.declare_parameter('moving_reference_speed_threshold', 0.03)
        self.declare_parameter('reference_velocity_filter_alpha', 0.35)

        self.k_x = self.get_parameter('k_x').value
        self.k_y = self.get_parameter('k_y').value
        self.k_theta = self.get_parameter('k_theta').value

        self.k_rho = self.get_parameter('k_rho').value
        self.k_alpha = self.get_parameter('k_alpha').value
        self.k_beta = self.get_parameter('k_beta').value

        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value

        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value

        control_frequency = self.get_parameter('control_frequency').value

        self.reference_velocity_timeout = self.get_parameter(
            'reference_velocity_timeout'
        ).value
        self.moving_reference_speed_threshold = self.get_parameter(
            'moving_reference_speed_threshold'
        ).value
        self.reference_velocity_filter_alpha = self.get_parameter(
            'reference_velocity_filter_alpha'
        ).value

        # Current state
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        # Current reference
        self.reference_x = None
        self.reference_y = None
        self.reference_yaw = None

        # Estimated reference motion
        self.reference_v = 0.0
        self.reference_omega = 0.0
        self.previous_reference_x = None
        self.previous_reference_y = None
        self.previous_reference_yaw = None
        self.previous_reference_stamp = None

        self.last_odom_time = None
        self.last_reference_time = None

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/husky_velocity_controller/odom',
            self.odom_callback,
            10
        )

        self.reference_subscriber = self.create_subscription(
            PoseStamped,
            '/reference_pose',
            self.reference_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/husky_velocity_controller/cmd_vel_unstamped',
            10
        )

        control_period = 1.0 / control_frequency
        self.control_timer = self.create_timer(
            control_period,
            self.control_loop
        )

        self.get_logger().info('Task 1 pose/trajectory controller started.')
        self.get_logger().info(
            f'k_x={self.k_x}, k_y={self.k_y}, k_theta={self.k_theta}'
        )

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.last_odom_time = self.get_clock().now()

    def reference_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.orientation)

        # Estimate reference velocity from the PoseStamped timestamps.
        # Using message time makes this independent of callback jitter.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.previous_reference_stamp is not None:
            dt = stamp - self.previous_reference_stamp

            if dt > 1e-5 and dt < 1.0:
                dx = x - self.previous_reference_x
                dy = y - self.previous_reference_y
                dyaw = wrap_to_pi(yaw - self.previous_reference_yaw)

                v_raw = math.hypot(dx, dy) / dt

                # Determine the signed linear speed from the reference
                # heading. For the circular trajectory, this gives the
                # correct positive tangent speed.
                heading_dx = math.cos(yaw)
                heading_dy = math.sin(yaw)
                direction = dx * heading_dx + dy * heading_dy
                v_raw = math.copysign(v_raw, direction) if v_raw > 1e-9 else 0.0

                omega_raw = dyaw / dt

                # Low-pass filter finite-difference noise.
                a = self.reference_velocity_filter_alpha
                self.reference_v = (
                    a * v_raw + (1.0 - a) * self.reference_v
                )
                self.reference_omega = (
                    a * omega_raw + (1.0 - a) * self.reference_omega
                )

        self.reference_x = x
        self.reference_y = y
        self.reference_yaw = yaw

        self.previous_reference_x = x
        self.previous_reference_y = y
        self.previous_reference_yaw = yaw
        self.previous_reference_stamp = stamp
        self.last_reference_time = self.get_clock().now()

    def control_loop(self):
        if (
            self.current_x is None
            or self.current_y is None
            or self.current_yaw is None
        ):
            return

        if (
            self.reference_x is None
            or self.reference_y is None
            or self.reference_yaw is None
        ):
            self.stop_robot()
            return

        # Position error in world/odom frame.
        dx = self.reference_x - self.current_x
        dy = self.reference_y - self.current_y
        rho = math.hypot(dx, dy)

        # Transform position error into robot body frame.
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)

        error_x_body = cos_yaw * dx + sin_yaw * dy
        error_y_body = -sin_yaw * dx + cos_yaw * dy

        heading_error = wrap_to_pi(
            self.reference_yaw - self.current_yaw
        )

        # Stop safely if reference stream becomes stale.
        if self.last_reference_time is not None:
            age = (
                self.get_clock().now() - self.last_reference_time
            ).nanoseconds / 1e9

            if age > self.reference_velocity_timeout:
                self.stop_robot()
                return

        cmd = Twist()

        # MOVING REFERENCE: trajectory tracking
        if abs(self.reference_v) > self.moving_reference_speed_threshold:

            cmd.linear.x = (
                self.reference_v * math.cos(heading_error)
                + self.k_x * error_x_body
            )

            cmd.angular.z = (
                self.reference_omega
                + self.k_y * self.reference_v * error_y_body
                + self.k_theta * math.sin(heading_error)
            )

        # STATIONARY REFERENCE: pose regulation
        else:
            desired_heading = math.atan2(dy, dx)
            alpha = wrap_to_pi(desired_heading - self.current_yaw)
            beta = wrap_to_pi(self.reference_yaw - self.current_yaw)

            if rho > self.position_tolerance:
                if abs(alpha) > math.pi / 2.0:
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.k_alpha * alpha
                else:
                    cmd.linear.x = self.k_rho * rho
                    cmd.angular.z = self.k_alpha * alpha

                    heading_factor = max(0.0, math.cos(alpha))
                    cmd.linear.x *= heading_factor
            else:
                cmd.linear.x = 0.0

                if abs(beta) > self.yaw_tolerance:
                    cmd.angular.z = self.k_beta * beta
                else:
                    cmd.angular.z = 0.0

        # Saturation
        cmd.linear.x = max(
            -self.max_linear_velocity,
            min(cmd.linear.x, self.max_linear_velocity)
        )

        cmd.angular.z = max(
            -self.max_angular_velocity,
            min(cmd.angular.z, self.max_angular_velocity)
        )

        self.cmd_vel_publisher.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_publisher.publish(cmd)

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = PoseController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()