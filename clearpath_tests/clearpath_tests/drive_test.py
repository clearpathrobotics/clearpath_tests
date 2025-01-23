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

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

class DriveTest(Node):

    def __init__(self, setup_path='/etc/clearpath'):
        super().__init__('drive_test')
        self.setup_path = setup_path

        # Define paths
        self.config_path = os.path.join(self.setup_path, 'robot.yaml')

        # Parse YAML into config
        self.clearpath_config = ClearpathConfig(self.config_path)
        self.platform = self.clearpath_config.platform.get_platform_model()

        # Params
        self.enable_drive = self.get_parameter_or('enable_drive', True)
        self.drive_topic = self.get_parameter_or('drive_topic', 'cmd_vel')
        self.max_speed = self.get_parameter_or('max_speed', 1.0)
        self.flip_frequency = self.get_parameter_or('flip_frequency', 0.5)
        self.publish_rate = self.get_parameter_or('publish_rate', 30)

        self.twist_msg = TwistStamped()
        self.twist_msg.twist.linear.x = self.max_speed
        self.publisher = self.create_publisher(TwistStamped, self.drive_topic, qos_profile_system_default)
        self.publish_count = 0
        self.flip_count = self.publish_rate / self.flip_frequency
        self.publish_timer = self.create_timer(1 / self.publish_rate, self.publish_callback)

    def publish_callback(self):
        self.twist_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.twist_msg)
        self.publish_count += 1
        if self.publish_count == self.flip_count:
            self.twist_msg.twist.linear.x = -self.twist_msg.twist.linear.x
            self.publish_count = 0


def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    dt = DriveTest(setup_path)

    try:
        rclpy.spin(dt)
    except KeyboardInterrupt:
        pass

    dt.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()