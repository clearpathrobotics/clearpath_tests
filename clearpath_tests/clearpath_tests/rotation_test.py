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
import math
from threading import Lock

from clearpath_generator_common.common import BaseGenerator
from clearpath_tests.mobility_test import MobilityTestNode
from clearpath_tests.test_node import ClearpathTestResult
import rclpy
from rclpy.duration import Duration
from tf_transformations import euler_from_quaternion


class RotationTestNode(MobilityTestNode):
    """
    Use odometry to rotate n complete rotations and then stops.

    Test assumes that the robot is on the ground, e-stops are cleared, and the area
    is free of obstacles and obstructions.
    """

    def __init__(self, setup_path='/etc/clearpath'):
        super().__init__('Rotation in place', 'rotation_test', setup_path)

        self.orientation_lock = Lock()

        self.goal_rotations = self.get_parameter_or('rotations', 2)
        self.max_speed = self.get_parameter_or('max_speed', 0.2)  # slightly more than 10 deg/s
        self.error_margin = self.get_parameter_or('error_margin', 10.0)  # +/-10 degrees

        # to debounce noisy odometry data, we assign a minimum duration to one rotation
        # this doesn't need to be accurate, just an absolute lower-bound on the time
        # the robot can take to do 1 full rotation at the requested speed
        # in practice the robot should take longer than this
        self.min_rotation_duration = Duration(seconds=math.pi / self.max_speed / 2.0)

        self.initial_yaw = None
        self.num_rotations = 0
        self.current_orientation = 0.0
        self.previous_orientation = 0.0
        self.test_done = False

    def publish_callback(self):
        if self.initial_yaw is None:
            now = self.get_clock().now()
            if (now - self.start_time) > self.odom_timeout:
                self.get_logger().error('Timed out waiting for odometry. Terminating test')
                raise TimeoutError('Timed out waiting for odometry')
        else:
            # publish the desired velocity
            if self.num_rotations >= self.goal_rotations:
                self.cmd_vel.twist.angular.z = 0.0
                self.test_done = True
            else:
                self.cmd_vel.twist.angular.z = self.max_speed
            super().publish_callback()

            # count how many rotations we've done and stop when we reach the right number
            self.orientation_lock.acquire()
            if self.current_orientation >= 0 and self.previous_orientation < 0:
                time_taken = self.get_clock().now() - self.last_rotation_complete_at

                # basic debouncing to handle odometry noise
                # assume we need at least a few seconds for a complete rotation to avoid
                # incrementing the counter multiple times if there are fluctuations around 0
                if time_taken >= self.min_rotation_duration:
                    self.num_rotations += 1
                    self.last_rotation_complete_at = self.get_clock().now()
                else:
                    self.get_logger().warning(f'Detected possible rotation completion, but only took {time_taken}. False positive?')  # noqa: E501

            self.orientation_lock.release()

    def odom_callback(self, msg):
        super().odom_callback(msg)

        xyzw = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        rpy = euler_from_quaternion(xyzw)

        if self.initial_yaw is None:
            self.initial_yaw = rpy[2]
            self.orientation_lock.acquire()
            self.previous_orientation = 0.0
            self.current_orientation = 0.0
            self.orientation_lock.release()
        else:
            self.orientation_lock.acquire()
            self.previous_orientation = self.current_orientation
            self.current_orientation = rpy[2] - self.initial_yaw
            self.orientation_lock.release()

    def start(self):
        super().start()

    def run_test(self):
        self.test_in_progress = True
        self.last_rotation_complete_at = self.get_clock().now()
        test_name = f'Rotation {self.goal_rotations}x in place'

        user_response = self.promptYN("""The robot will rotate on the spot
The robot must be on the ground, all e-stops cleared, and a 2m safety clearance around the robot.
Are all these conditions met?""")
        if user_response == 'N':
            return [ClearpathTestResult(False, test_name, 'User skipped')]

        self.get_logger().info('Starting rotation test')
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
            expected_duration = Duration(seconds=self.goal_rotations * math.pi * 2 / self.max_speed)  # noqa: E501
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
                    f'Robot took {test_duration.nanoseconds / 1000000000:0.2f}s rotate {self.goal_rotations}x vs {expected_duration.nanoseconds / 1000000000:0.2f}s expected (err={time_error:0.4f})'  # noqa: E501
                ))
            else:
                results.append(ClearpathTestResult(
                    True,
                    f'{test_name} (duration)',
                    None
                ))

            user_response = self.promptYN(f"""Test complete.
    Measure the robot's actual alignment.
    Is it within {self.error_margin:0.1f} degrees of its original orientation?""")
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
