#!/usr/bin/env python3
# Software License Agreement (BSD)
#
# @author    Chris Iverach-Brereton <civerachb@clearpathrobotics.com>
# @copyright (c) 2025, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.types.platform import Platform
from clearpath_generator_common.common import BaseGenerator

from enum import Enum

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

import math
import os

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

from tf_transformations import euler_from_quaternion


class RotationTestNode(Node):
    """
    Uses odometry to rotate 90 degrees one way, then the other, forever

    Test assumes that the robot is on the ground, e-stops are cleared, and the area
    is free of obstacles and obstructions.
    """

    def __init__(self, setup_path='/etc/clearpath'):
        super().__init__('rotation_test')

        self.setup_path = setup_path
        self.config_path = os.path.join(self.setup_path, 'robot.yaml')

        self.clearpath_config = ClearpathConfig(self.config_path)
        self.platform = self.clearpath_config.platform.get_platform_model()
        self.namespace = self.clearpath_config.get_namespace()

        self.enable_drive = self.get_parameter_or('enable_drive', True)
        self.drive_topic = self.get_parameter_or('drive_topic', 'cmd_vel')
        self.odom_topic = self.get_parameter_or('odom_topic', 'platform/odom/filtered')
        self.max_speed = self.get_parameter_or('max_speed', 0.2)  # slightly more than 10 deg/s
        self.publish_rate = self.get_parameter_or('publish_rate', 30)

        if not self.odom_topic.startswith('/'):
            self.odom_topic = f'/{self.namespace}/{self.odom_topic}'

        if not self.drive_topic.startswith('/'):
            self.drive_topic = f'/{self.namespace}/{self.drive_topic}'

        self.current_orientation = None
        self.twist_msg = TwistStamped()
        self.twist_msg.twist.angular.z = self.max_speed

        self.start_time = self.get_clock().now()
        self.odom_timeout = Duration(seconds=10)
        self.initial_yaw = None
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)  # noqa: E501

        self.get_logger().info(f'Waiting for odometry on {self.odom_topic}...')
        self.publisher = self.create_publisher(TwistStamped, self.drive_topic, qos_profile_system_default)  # noqa: E501
        self.publish_timer = self.create_timer(1 / self.publish_rate, self.publish_callback)

    def publish_callback(self):
        if self.initial_yaw is None:
            now = self.get_clock().now()
            if (now - self.start_time) > self.odom_timeout:
                self.get_logger().error('Timed out waiting for odometry. Terminating test')
                raise(TimeoutError('Timed out waiting for odometry'))
        else:
            self.twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().info(f'Current rotation: {self.current_orientation * 180.0 / math.pi:0.2f}')  # noqa: E501

            # rotate 90 degrees one way, then the other
            if self.current_orientation > math.pi/2:
                self.get_logger().info('Starting reverse turn')
                self.twist_msg.twist.angular.z = -self.max_speed
            elif self.current_orientation < -math.pi/2:
                self.get_logger().info('Starting forward turn')
                self.twist_msg.twist.angular.z = self.max_speed

            self.publisher.publish(self.twist_msg)

    def odom_callback(self, msg):
        """
        Check if we've made 2 full rotations.
        """
        xyzw = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        rpy = euler_from_quaternion(xyzw)

        if self.initial_yaw is None:
            self.initial_yaw = rpy[2]
            self.current_orientation = 0.0
        else:
            self.current_orientation = rpy[2] - self.initial_yaw


def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    try:
        rt = RotationTestNode(setup_path)
        try:
            rclpy.spin(rt)
        except KeyboardInterrupt:
            rt.get_logger().info('User aborted! Cleaning up & exiting...')
        finally:
            rt.destroy_node()
    except TimeoutError:
        # This error is already logged when it's raised
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()