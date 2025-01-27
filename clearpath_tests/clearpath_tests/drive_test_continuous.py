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
from clearpath_generator_common.common import BaseGenerator

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

import os

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class DriveTestNode(Node):
    """
    Uses odometry to drive 1m forwards, then backwards, etc... forever

    Test assumes that the robot is on the ground, e-stops are cleared, and the area
    is free of obstacles and obstructions.
    """

    def __init__(self, setup_path='/etc/clearpath'):
        super().__init__('drive_test')

        self.setup_path = setup_path
        self.config_path = os.path.join(self.setup_path, 'robot.yaml')

        self.clearpath_config = ClearpathConfig(self.config_path)
        self.platform = self.clearpath_config.platform.get_platform_model()
        self.namespace = self.clearpath_config.get_namespace()

        self.base_link = self.get_parameter_or('base_link', 'base_link')
        self.enable_drive = self.get_parameter_or('enable_drive', True)
        self.drive_topic = self.get_parameter_or('drive_topic', 'cmd_vel')
        self.odom_topic = self.get_parameter_or('odom_topic', 'platform/odom/filtered')
        self.max_speed = self.get_parameter_or('max_speed', 0.1)  # 10cm/s; nice and safe
        self.publish_rate = self.get_parameter_or('publish_rate', 30)

        if not self.odom_topic.startswith('/'):
            self.odom_topic = f'/{self.namespace}/{self.odom_topic}'

        if not self.drive_topic.startswith('/'):
            self.drive_topic = f'/{self.namespace}/{self.drive_topic}'

        self.current_displacement = None
        self.twist_msg = TwistStamped()
        self.twist_msg.twist.linear.x = self.max_speed

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_time = self.get_clock().now()
        self.odom_timeout = Duration(seconds=10)
        self.initial_position = None
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)  # noqa: E501

        self.get_logger().info(f'Waiting for odometry on {self.odom_topic}...')
        self.publisher = self.create_publisher(TwistStamped, self.drive_topic, qos_profile_system_default)  # noqa: E501
        self.publish_timer = self.create_timer(1 / self.publish_rate, self.publish_callback)

    def publish_callback(self):
        if self.initial_position is None:
            now = self.get_clock().now()
            if (now - self.start_time) > self.odom_timeout:
                self.get_logger().error('Timed out waiting for odometry. Terminating test')
                raise(TimeoutError('Timed out waiting for odometry'))
        else:
            self.twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().info(f'Current position: {self.current_displacement:0.2f}m')

            # drive 1m forwards, then reverse
            if self.current_displacement > 1.0:
                self.get_logger().info('Starting reverse')
                self.twist_msg.twist.linear.x = -self.max_speed
            elif self.current_displacement < -1.0:
                self.get_logger().info('Starting forward')
                self.twist_msg.twist.linear.x = self.max_speed

            self.publisher.publish(self.twist_msg)

    def odom_callback(self, msg):
        odom_frame = msg.header.frame_id

        try:
            transformation = self.tf_buffer.lookup_transform(
                odom_frame,
                self.base_link,
                rclpy.time.Time()
            )
        except TransformException as err:
            self.get_logger().warning(f'TF Lookup failure: {err}')
            return

        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        transformed_pose = do_transform_pose_stamped(pose_stamped, transformation)

        if self.initial_position is None:
            self.initial_position = transformed_pose.pose.position
            self.current_displacement = 0.0
        else:
            self.current_displacement = transformed_pose.pose.position.x - self.initial_position.x  # noqa: E501


def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    try:
        dt = DriveTestNode(setup_path)
        try:
            rclpy.spin(dt)
        except KeyboardInterrupt:
            dt.get_logger().info('User aborted! Cleaning up & exiting...')
        finally:
            dt.destroy_node()
    except TimeoutError:
        # This error is already logged when it's raised
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()