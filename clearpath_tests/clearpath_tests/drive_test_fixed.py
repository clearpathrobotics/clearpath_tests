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

from clearpath_tests.test_node import TestNode, TestResult

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

import os

import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class DriveTestNode(TestNode):
    """
    Uses odometry to drive a fixed distance forwards and then stop

    Test assumes that the robot is on the ground, e-stops are cleared, and the area
    is free of obstacles and obstructions.
    """

    def __init__(self, setup_path='/etc/clearpath'):
        super().__init__('Drive Fixed Distance', 'drive_test')

        self.test_done = False

        self.setup_path = setup_path
        self.config_path = os.path.join(self.setup_path, 'robot.yaml')

        self.clearpath_config = ClearpathConfig(self.config_path)
        self.platform = self.clearpath_config.platform.get_platform_model()
        self.namespace = self.clearpath_config.get_namespace()

        self.goal_distance = self.get_parameter_or('distance', 5.0)
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

    def publish_callback(self):
        if self.initial_position is None:
            now = self.get_clock().now()
            if (now - self.start_time) > self.odom_timeout:
                self.get_logger().error('Timed out waiting for odometry. Terminating test')
                raise(TimeoutError('Timed out waiting for odometry'))
        else:
            self.twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().info(f'Current position: {self.current_displacement:0.2f}m ({self.goal_distance}m)')  # noqa: E501

            # drive 5m forwards, then stop
            if self.current_displacement >= self.goal_distance:
                self.get_logger().info('Reached goal')
                self.twist_msg.twist.linear.x = 0.0

            self.publisher.publish(self.twist_msg)

            if self.twist_msg.twist.linear.x == 0:
                self.test_done = True

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

    def start(self):
        self.start_time = self.get_clock().now()
        self.odom_timeout = Duration(seconds=10)
        self.initial_position = None
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)  # noqa: E501

        self.get_logger().info(f'Waiting for odometry on {self.odom_topic}...')
        self.publisher = self.create_publisher(TwistStamped, self.drive_topic, qos_profile_system_default)  # noqa: E501
        self.publish_timer = self.create_timer(1 / self.publish_rate, self.publish_callback)

    def run_test(self):
        test_name = f'Drive {self.goal_distance:0.1f}m'

        user_response = self.promptYN(f"""The robot will drive forwards approximately {self.goal_distance}m
The robot must be on the ground, all e-stops cleared, and a 2m safety clearance around the robot.
Are all these conditions met?""")
        if user_response == 'N':
            return [TestResult(False, test_name, 'User skipped')]

        self.get_logger().info('Starting drive test')
        self.start()
        while not self.test_done:
            rclpy.spin_once(self)

        user_response = self.promptYN(f"""Test complete.
Measure the robot's actual displacement.
Is it between {self.goal_distance * 0.9:0.2f}m and {self.goal_distance * 1.1:0.2f}m?""")
        if user_response == 'N':
            measured_distance = input('How far did the robot actually drive (in meters)? ')
            return [TestResult(False, test_name, f'Incorrect distance: {measured_distance}')]
        else:
            return [TestResult(True, test_name, None)]

def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    try:
        dt = DriveTestNode(setup_path)
        dt.start()
        try:
            while not dt.test_done:
                rclpy.spin_once(dt)
            dt.get_logger().info('Test complete')
        except KeyboardInterrupt:
            dt.get_logger().info('User aborted! Cleaning up & exiting...')
        dt.destroy_node()
    except TimeoutError:
        # This error is already logged when it's raised
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()