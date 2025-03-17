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

from clearpath_generator_common.common import BaseGenerator

from clearpath_tests.mobility_test import MobilityTestNode
from clearpath_tests.test_node import (
    ConfigurableTransformListener,
    ClearpathTestResult
)

from geometry_msgs.msg import PoseStamped, TwistStamped

import rclpy
from rclpy.duration import Duration

from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer


class DriveTestNode(MobilityTestNode):
    """
    Uses odometry to drive a fixed distance forwards and then stop

    Test assumes that the robot is on the ground, e-stops are cleared, and the area
    is free of obstacles and obstructions.
    """

    def __init__(self, setup_path='/etc/clearpath'):
        super().__init__('Drive Fixed Distance', 'drive_test', setup_path)

        self.goal_distance = self.get_parameter_or('distance', 5.0)
        self.max_speed = self.get_parameter_or('max_speed', 0.1)  # 10cm/s; nice and safe
        self.error_margin = self.get_parameter_or('error_margin', 0.05)  # +/-5%

        self.current_displacement = None
        self.twist_msg = TwistStamped()
        self.twist_msg.twist.linear.x = self.max_speed

        self.tf_buffer = Buffer()
        self.tf_listener = ConfigurableTransformListener(
            self.tf_buffer,
            self,
            tf_topic=f'/{self.clearpath_config.get_namespace()}/tf',
            tf_static_topic=f'/{self.clearpath_config.get_namespace()}/tf_static'
        )

    def publish_callback(self):
        if self.initial_position is None:
            now = self.get_clock().now()
            if (now - self.start_time) > self.odom_timeout:
                self.get_logger().error('Timed out waiting for odometry. Terminating test')
                raise(TimeoutError('Timed out waiting for odometry'))
        else:
            if self.current_displacement >= self.goal_distance:
                self.get_logger().info('Reached goal')
                self.cmd_vel.twist.linear.x = 0.0
                self.test_done = True
            else:
                self.cmd_vel.twist.linear.x = self.max_speed
            super().publish_callback()

            self.get_logger().info(f'Current position: {self.current_displacement:0.2f}m ({self.goal_distance}m)')  # noqa: E501

    def odom_callback(self, msg):
        super().odom_callback(msg)

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
            self.current_displacement = abs(transformed_pose.pose.position.x - self.initial_position.x)  # noqa: E501

    def run_test(self):
        self.test_in_progress = True
        test_name = f'Drive {self.goal_distance:0.1f}m'

        user_response = self.promptYN(f"""The robot will drive forwards approximately {self.goal_distance}m
The robot must be on the ground, all e-stops cleared, and a 2m safety clearance around the robot.
Are all these conditions met?""")
        if user_response == 'N':
            return [ClearpathTestResult(False, test_name, 'User skipped')]

        self.get_logger().info('Starting drive test')
        self.start()
        start_time = self.get_clock().now()
        while not self.test_done and not self.test_error:
            rclpy.spin_once(self)
        end_time = self.get_clock().now()

        # ensure we stop the robot
        # the publising timer is still running
        # we need to support omni robots, so set linear X and Y!
        self.cmd_vel.twist.linear.x = 0.0
        self.cmd_vel.twist.linear.y = 0.0
        self.cmd_vel.twist.angular.z = 0.0

        results = self.test_results

        if self.test_error:
            self.get_logger().warning(f'Test aborted due to an error: {self.test_error_msg}')
        else:
            expected_duration = Duration(seconds=self.goal_distance / self.max_speed)
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
                    f'Robot took {test_duration.nanoseconds / 1000000000:0.2f}s to drive {self.goal_distance}m vs {expected_duration.nanoseconds / 1000000000:0.2f}s expected (err={time_error:0.4f})'
                ))
            else:
                results.append(ClearpathTestResult(
                    True,
                    f'{test_name} (duration)',
                    None
                ))

            user_response = self.promptYN(f"""Test complete.
    Measure the robot's actual displacement.
    Is it between {self.goal_distance * (1.0 - self.error_margin):0.2f}m and {self.goal_distance * (1.0 + self.error_margin):0.2f}m?""")
            if user_response == 'N':
                measured_distance = input('How far did the robot actually drive (in meters)? ')
                results.append(ClearpathTestResult(
                    False,
                    f'{test_name} (accuracy)',
                    f'Incorrect distance: {measured_distance}'
                ))
            else:
                results.append(ClearpathTestResult(
                    True,
                    f'{test_name} (accuracy)',
                    None
                ))

        return results

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