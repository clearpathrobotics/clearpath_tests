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
from clearpath_tests.test_node import(
    ClearpathTestNode,
    ClearpathTestResult,
    ConfigurableTransformListener
)
from clearpath_generator_common.common import BaseGenerator

from typing import Optional
from typing import Union

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu

from tf2_geometry_msgs import do_transform_vector3
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class ImuTestNode(ClearpathTestNode):
    """
    Checks that the IMU is publishing, EKF is active, and the IMU orientation is sane

    Will fail of any of the above is not correct
    """

    def __init__(self, imu_num=0, setup_path='/etc/clearpath' ):
        super().__init__(f'IMU Test (imu_{imu_num})', f'imu_{imu_num}_test', setup_path)
        self.test_in_progress = False

        self.base_link = 'base_link'
        self.tf_buffer = Buffer()
        self.tf_listener = ConfigurableTransformListener(
            self.tf_buffer,
            self,
            tf_topic=f'/{self.clearpath_config.get_namespace()}/tf',
            tf_static_topic=f'/{self.clearpath_config.get_namespace()}/tf_static'
        )

    def imu_raw_callback(self, imu_data:Imu):
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

        gyro_vector = Vector3Stamped
        gyro_vector.header = imu_data.header
        gyro_vector.vector.x = imu_data.angular_velocity.x
        gyro_vector.vector.y = imu_data.angular_velocity.y
        gyro_vector.vector.z = imu_data.angular_velocity.z
        transformed_gyro = do_transform_vector3(gyro_vector, transformation)

        self.get_logger().info(f'a ({transformed_accel.vector.x}, {transformed_accel.vector.y}, {transformed_accel.vector.z})')
        self.get_logger().info(f'g ({transformed_gyro.vector.x}, {transformed_gyro.vector.y}, {transformed_gyro.vector.z})')
        self.get_logger().info('---')

    def start(self):
        self.imu_sub = self.create_subscription(
            Imu,
            f'/{self.namespace}/sensors/imu_0/data_raw',
            self.imu_raw_callback,
            qos_profile=qos_profile_sensor_data,
        )

    def run_test(self):
        self.test_in_progress = True
        self.start()

        while True:
            pass

def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    it = ImuTestNode(imu_num=0, setup_path=setup_path)

    try:
        it.start()
        rclpy.spin(it)
    except KeyboardInterrupt:
        pass

    it.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()