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
from clearpath_tests.test_node import ClearpathTestResult
from clearpath_tests.tf import ConfigurableTransformListener
from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_geometry_msgs import do_transform_vector3
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer


class RotationTestNode(MobilityTestNode):
    """
    Rotate anticlockwise at a fixed rate to verify that the IMU is aligned correctly.

    The IMU should read positive angular velocity around the Z axis.
    """

    def __init__(self, imu_num=0, setup_path='/etc/clearpath'):
        super().__init__('Rotation in place', 'rotation_test', setup_path)

        self.max_speed = self.get_parameter_or(
            'max_speed',
            0.3490658503988659,  # 20 deg/s
        )
        self.record_data = False

        self.imu_num = imu_num

        self.base_link = 'base_link'
        self.tf_buffer = Buffer()
        self.tf_listener = ConfigurableTransformListener(
            self.tf_buffer,
            self,
            tf_topic=f'/{self.clearpath_config.get_namespace()}/tf',
            tf_static_topic=f'/{self.clearpath_config.get_namespace()}/tf_static'
        )
        self.gyro_samples = []

    def imu_callback(self, imu_data):
        super().odom_callback(imu_data)

        imu_frame = imu_data.header.frame_id

        try:
            transformation = self.tf_buffer.lookup_transform(
                imu_frame,
                self.base_link,
                rclpy.time.Time()
            )
        except TransformException as err:
            self.get_logger().warning(f'TF Lookup failure: {err}')
            return

        accel_vector = Vector3Stamped()
        accel_vector.header = imu_data.header
        accel_vector.vector.x = imu_data.linear_acceleration.x
        accel_vector.vector.y = imu_data.linear_acceleration.y
        accel_vector.vector.z = imu_data.linear_acceleration.z
        transformed_accel = do_transform_vector3(accel_vector, transformation)

        gyro_vector = Vector3Stamped()
        gyro_vector.header = imu_data.header
        gyro_vector.vector.x = imu_data.angular_velocity.x
        gyro_vector.vector.y = imu_data.angular_velocity.y
        gyro_vector.vector.z = imu_data.angular_velocity.z
        transformed_gyro = do_transform_vector3(gyro_vector, transformation)

        if not self.test_in_progress:
            self.get_logger().info(f'a ({transformed_accel.vector.x}, {transformed_accel.vector.y}, {transformed_accel.vector.z})')  # noqa: E501
            self.get_logger().info(f'g ({transformed_gyro.vector.x}, {transformed_gyro.vector.y}, {transformed_gyro.vector.z})')  # noqa: E501
            self.get_logger().info('---')

        if self.record_data:
            self.gyro_samples.append(transformed_gyro)

    def start(self):
        super().start()

        # Subscribe to our default IMU
        imu_topic = f'/{self.namespace}/sensors/imu_{self.imu_num}/data'
        self.get_logger().info(f'Subscribing to IMU data on {imu_topic}...')
        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            qos_profile=qos_profile_sensor_data,
        )

    def run_test(self):
        self.cmd_vel.twist.linear.x = 0.0
        self.cmd_vel.twist.linear.y = 0.0
        self.cmd_vel.twist.linear.z = 0.0
        self.cmd_vel.twist.angular.x = 0.0
        self.cmd_vel.twist.angular.y = 0.0
        self.cmd_vel.twist.angular.z = self.max_speed

        self.test_in_progress = True
        self.last_rotation_complete_at = self.get_clock().now()

        user_response = self.promptYN("""The robot will rotate on the spot
The robot must be on the ground, all e-stops cleared, and a 2m safety clearance around the robot.
Are all these conditions met?""")
        if user_response == 'N':
            return [ClearpathTestResult(False, self.test_name, 'User skipped')]

        # start rotating but don't record data for 1s to remove noise
        self.get_logger().info('Starting rotation test')
        self.start()
        startup_wait = Duration(seconds=1.0)
        start_time = self.get_clock().now()
        while (
            not self.test_error
            and self.get_clock().now() - start_time <= startup_wait
        ):
            rclpy.spin_once(self)
        self.record_data = True

        if self.test_error:
            self.record_data = False
            self.cmd_vel.twist.angular.z = 0.0
            self.get_logger().warning(f'Test aborted due to an error: {self.test_error_msg}')
            return self.test_results

        # record data for 10s
        test_wait = Duration(seconds=10)
        start_time = self.get_clock().now()
        while (
            not self.test_error
            and self.get_clock().now() - start_time <= test_wait
        ):
            rclpy.spin_once(self)

        if self.test_error:
            self.record_data = False
            self.cmd_vel.twist.angular.z = 0.0
            self.get_logger().warning(f'Test aborted due to an error: {self.test_error_msg}')
            return self.test_results

        # rotate for another 1s before stopping to remove noise
        self.record_data = False
        end_wait = Duration(seconds=1.0)
        start_time = self.get_clock().now()
        while (
            not self.test_error
            and self.get_clock().now() - start_time <= end_wait
        ):
            rclpy.spin_once(self)

        # stop turning
        self.cmd_vel.twist.angular.z = 0.0

        if self.test_error:
            self.get_logger().warning(f'Test aborted due to an error: {self.test_error_msg}')
            return self.test_results

        # process the results
        results = self.test_results

        if len(self.gyro_samples) <= 10:
            results.append(ClearpathTestResult(
                False,
                self.test_name,
                f'Insufficient IMU data recorded ({len(self.gyro_samples)}): is the IMU publishing at the correct rate?',  # noqa: E501
            ))
        else:
            avg_vel = sum(gyro.z for gyro in self.gyro_samples) / len(self.gyro_samples)
            allowed_error = 0.8
            measured_error = min(avg_vel, self.max_speed) / max(avg_vel, self.max_speed)
            results.append(ClearpathTestResult(
                measured_error >= allowed_error,
                self.test_name,
                f'Recorded angular velocity: {avg_vel}rad/s (err: {measured_error:0.2f})'
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
