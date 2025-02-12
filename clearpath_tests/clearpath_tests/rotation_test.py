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

from clearpath_tests.test_node import ClearpathTestNode, ClearpathTestResult

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

import math
import os

import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

from tf_transformations import euler_from_quaternion


class RotationTestNode(ClearpathTestNode):
    """
    Uses odometry to rotate n complete rotations and then stops

    Test assumes that the robot is on the ground, e-stops are cleared, and the area
    is free of obstacles and obstructions.
    """

    def __init__(self, setup_path='/etc/clearpath'):
        super().__init__('Rotation in place', 'rotation_test', setup_path)

        self.goal_rotations = self.get_parameter_or('rotations', 2)
        self.enable_drive = self.get_parameter_or('enable_drive', True)
        self.drive_topic = self.get_parameter_or('drive_topic', 'cmd_vel')
        self.odom_topic = self.get_parameter_or('odom_topic', 'platform/odom/filtered')
        self.max_speed = self.get_parameter_or('max_speed', 0.2)  # slightly more than 10 deg/s
        self.publish_rate = self.get_parameter_or('publish_rate', 30)

        if not self.odom_topic.startswith('/'):
            self.odom_topic = f'/{self.namespace}/{self.odom_topic}'

        if not self.drive_topic.startswith('/'):
            self.drive_topic = f'/{self.namespace}/{self.drive_topic}'

        self.num_rotations = 0
        self.current_orientation = 0.0
        self.previous_orientation = 0.0
        self.test_done = False
        self.twist_msg = TwistStamped()
        self.twist_msg.twist.angular.z = self.max_speed

    def publish_callback(self):
        if self.initial_yaw is None:
            now = self.get_clock().now()
            if (now - self.start_time) > self.odom_timeout:
                self.get_logger().error('Timed out waiting for odometry. Terminating test')
                raise(TimeoutError('Timed out waiting for odometry'))
        else:
            self.twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().info(f'Current rotation: {self.current_orientation * 180.0 / math.pi:0.2f} ({self.num_rotations}/{self.goal_rotations})')  # noqa: E501

            # count how many rotations we've done and stop when we reach the right number
            if self.current_orientation >= 0 and self.previous_orientation < 0:
                # basic debouncing to handle odometry noise
                # assume we need at least a few seconds for a complete rotation to avoid incrementing
                # the counter multiple times if there are fluctuations right around 0
                if (self.get_clock().now() - self.last_rotation_complete_at) > self.min_rotation_duration:  # noqa: E501
                    self.num_rotations += 1
                    self.last_rotation_complete_at = self.get_clock().now()

            if self.num_rotations >= self.goal_rotations:
                self.twist_msg.twist.angular.z = 0.0
                self.get_logger().info('Rotated desired times')

            self.publisher.publish(self.twist_msg)

            if self.num_rotations >= self.goal_rotations:
                self.test_done = True

    def odom_callback(self, msg):
        xyzw = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        rpy = euler_from_quaternion(xyzw)

        if self.initial_yaw is None:
            self.initial_yaw = rpy[2]
            self.previous_orientation = 0.0
            self.current_orientation = 0.0
        else:
            self.previous_orientation = self.current_orientation
            self.current_orientation = rpy[2] - self.initial_yaw

    def start(self):
        self.start_time = self.get_clock().now()
        self.odom_timeout = Duration(seconds=10)
        self.initial_yaw = None
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)  # noqa: E501

        self.get_logger().info(f'Waiting for odometry on {self.odom_topic}...')
        self.last_rotation_complete_at = self.get_clock().now()
        self.min_rotation_duration = Duration(seconds=3.0)
        self.publisher = self.create_publisher(TwistStamped, self.drive_topic, qos_profile_system_default)  # noqa: E501
        self.publish_timer = self.create_timer(1 / self.publish_rate, self.publish_callback)

    def run_test(self):
        test_name = f'Rotation {self.goal_rotations}x in place'

        user_response = self.promptYN(f"""The robot will rotate {self.goal_rotations} times
The robot must be on the ground, all e-stops cleared, and a 2m safety clearance around the robot.
Are all these conditions met?""")
        if user_response == 'N':
            return [ClearpathTestResult(False, test_name, 'User skipped')]

        self.get_logger().info('Starting rotation test')
        self.start()
        start_time = self.get_clock().now()
        while not self.test_done:
            rclpy.spin_once(self)
        end_time = self.get_clock().now()

        results = []

        expected_duration = Duration(seconds = self.goal_rotations * math.pi * 2 / self.max_speed)
        test_duration = end_time - start_time

        time_error = (
            min(
                expected_duration.nanoseconds,
                test_duration.nanoseconds) /
            max(
                expected_duration.nanoseconds,
                test_duration.nanoseconds
            )
        )
        if time_error < 0.8:
            results.append(ClearpathTestResult(
                False,
                f'{test_name} (duration)',
                f'Robot took {test_duration.nanoseconds / 1000000000:0.2f}s rotate {self.goal_rotations}x vs {expected_duration.nanoseconds / 1000000000:0.2f}s expected'
            ))
        else:
            results.append(ClearpathTestResult(
                True,
                f'{test_name} (duration)',
                None
            ))

        user_response = self.promptYN("""Test complete.
Measure the robot's actual alignment.
Is it within 10 degrees of its original orientation?""")
        if user_response == 'N':
            measured_alignment = input("How many degrees off is the robot's alignment? ")
            results.append(ClearpathTestResult(
                False,
                f'{test_name} (accuracy)',
                f'Incorrect aligment: {measured_alignment}'
            ))
        else:
            results.append(ClearpathTestResult(
                True,
                f'{test_name} (accuracy)',
                'Alignment OK'
            ))

        return results

def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    try:
        rt = RotationTestNode(setup_path)
        rt.start()
        try:
            while not rt.test_done:
                rclpy.spin_once(rt)
            rt.get_logger().info('Test complete')
        except KeyboardInterrupt:
            rt.get_logger().info('User aborted! Cleaning up & exiting...')
        rt.destroy_node()
    except TimeoutError:
        # This error is already logged when it's raised
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()