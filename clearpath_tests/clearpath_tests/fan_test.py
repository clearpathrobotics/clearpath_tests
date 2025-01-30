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
from clearpath_platform_msgs.msg import Fans
from clearpath_tests.test_node import ClearpathTestNode, ClearpathTestResult

import rclpy
from rclpy.qos import qos_profile_system_default

class FanTestNode(ClearpathTestNode):

    def __init__(self, n_fans=4, setup_path='/etc/clearpath'):
        super().__init__('Fans', 'fan_test', setup_path)

        # Params
        self.n_fans = self.get_parameter_or('num_fans', n_fans)
        self.fans_topic = self.get_parameter_or('fans_topic', 'platform/mcu/cmd_fans')
        self.publish_rate = self.get_parameter_or('publish_rate', 2)

        self.publisher = self.create_publisher(Fans, self.fans_topic, qos_profile_system_default)
        self.value = 255

    def publish_callback(self):
        # Define the message to be sent
        msg = Fans()
        for _ in range(self.n_fans):
            msg.fans.append(self.value)
        # Publish the message
        self.publisher.publish(msg)

    def start(self):
        self.publish_timer = self.create_timer(1 / self.publish_rate, self.publish_callback)

    def run_test(self):
        results = []

        msg = Fans()
        for _ in range(self.n_fans):
            msg.fans.append(0)

        self.publisher.publish(msg)
        user_input = self.promptYN('Are all fans stopped?')
        if user_input == 'Y':
            results.append(ClearpathTestResult(True, 'All fans off', None))
        else:
            results.append(ClearpathTestResult(False, 'All fans off', None))

        for i in range(self.n_fans):
            for j in range(self.n_fans):
                msg.fans[j] = 0
            msg.fans[i] = 255
            self.publisher.publish(msg)

            user_input = self.promptYN(f'Is ONLY fan {i+1} on?')
            if user_input == 'Y':
                results.append(ClearpathTestResult(True, f'Fan {i+1} only', None))
            else:
                results.append(ClearpathTestResult(False, f'Fan {i+1} only', None))

        for _ in range(self.n_fans):
            msg.fans[i] = 255
        self.publisher.publish(msg)

        user_input = self.promptYN('Are all fans running?')
        if user_input == 'Y':
            results.append(ClearpathTestResult(True, 'All fans on', None))
        else:
            results.append(ClearpathTestResult(False, 'All fans on', None))

        return results


def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    fan_test = FanTestNode(setup_path)

    try:
        rclpy.spin(fan_test)
    except KeyboardInterrupt:
        pass

    fan_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()