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

from clearpath_config.common.types.platform import Platform
from clearpath_generator_common.common import BaseGenerator
from clearpath_platform_msgs.msg import RGB, Lights
from clearpath_tests.test_node import TestNode, TestResult

import rclpy
from rclpy.qos import qos_profile_system_default


class LightTestNode(TestNode):

    def __init__(self, light_zones=4, setup_path='/etc/clearpath'):
        super().__init__('Lights', 'light_test', setup_path)

        self.light_zones = self.get_parameter_or('n_lights', light_zones)

        if self.platform == Platform.A300:
            self.front_left = [Lights.A300_LIGHTS_FRONT_LEFT]
            self.front_right = [Lights.A300_LIGHTS_FRONT_RIGHT]
            self.back_left = [Lights.A300_LIGHTS_REAR_LEFT]
            self.back_right = [Lights.A300_LIGHTS_REAR_RIGHT]
        elif self.platform == Platform.W200:
            self.front_left = [Lights.W200_LIGHTS_FRONT_LEFT]
            self.front_right = [Lights.W200_LIGHTS_FRONT_RIGHT]
            self.back_left = [Lights.W200_LIGHTS_REAR_LEFT]
            self.back_right = [Lights.W200_LIGHTS_REAR_RIGHT]
        elif self.platform == Platform.R100:
            self.front_left = [
                Lights.R100_LIGHTS_FRONT_PORT_UPPER,
                Lights.R100_LIGHTS_FRONT_PORT_LOWER
            ]
            self.front_right = [
                Lights.R100_LIGHTS_FRONT_STARBOARD_UPPER,
                Lights.R100_LIGHTS_FRONT_STARBOARD_LOWER
            ]
            self.back_left = [
                Lights.R100_LIGHTS_REAR_PORT_UPPER,
                Lights.R100_LIGHTS_REAR_PORT_LOWER
            ]
            self.back_right = [
                Lights.R100_LIGHTS_REAR_STARBOARD_UPPER,
                Lights.R100_LIGHTS_REAR_STARBOARD_LOWER
            ]
        elif (
            self.platform == Platform.DD100 or
            self.platform == Platform.DO100
        ):
            self.front_left = [Lights.D100_LIGHTS_FRONT_LEFT]
            self.front_right = [Lights.D100_LIGHTS_FRONT_RIGHT]
            self.back_left = [Lights.D100_LIGHTS_REAR_LEFT]
            self.back_right = [Lights.D100_LIGHTS_REAR_RIGHT]
        elif (
            self.platform == Platform.DD150 or
            self.platform == Platform.DO150
        ):
            self.front_left = [Lights.D150_LIGHTS_FRONT_LEFT]
            self.front_right = [Lights.D150_LIGHTS_FRONT_RIGHT]
            self.back_left = [Lights.D150_LIGHTS_REAR_LEFT]
            self.back_right = [Lights.D150_LIGHTS_REAR_RIGHT]
        else:
            self.get_logger().warning(
                f'Unknown platform {self.platform}: unable to determine lighting indices'
            )
            self.front_left = []
            self.front_right = []
            self.back_left = []
            self.back_right = []


        # Params
        self.lights_topic = self.get_parameter_or('lights_topic', f'/{self.namespace}/platform/cmd_lights')
        self.publish_rate = self.get_parameter_or('publish_rate', 2)

        # Publisher and message
        self.msg = Lights()
        for i in range(self.light_zones):
            self.msg.lights.append(RGB())
        self.publisher = self.create_publisher(Lights, self.lights_topic, qos_profile_system_default)

        self.test_in_progress = False
        self.colour = 0

    def publish_callback(self):
        # Define the message to be sent


        # cycle through blue/green/red/white/off forever
        if not self.test_in_progress:
            if self.colour == 0:
                for i in range(self.light_zones):
                    self.msg.lights[i].red = int(0)
                    self.msg.lights[i].blue = int(10)
                    self.msg.lights[i].green = int(0)
                self.colour = 1
            elif self.colour == 1:
                for i in range(self.light_zones):
                    self.msg.lights[i].red = int(0)
                    self.msg.lights[i].blue = int(0)
                    self.msg.lights[i].green = int(10)
                self.colour = 2
            elif self.colour == 2:
                for i in range(self.light_zones):
                    self.msg.lights[i].red = int(10)
                    self.msg.lights[i].blue = int(0)
                    self.msg.lights[i].green = int(0)
                self.colour = 3
            elif self.colour == 3:
                for i in range(self.light_zones):
                    self.msg.lights[i].red = int(5)
                    self.msg.lights[i].blue = int(5)
                    self.msg.lights[i].green = int(5)
                self.colour = 4
            elif self.colour == 4:
                for i in range(self.light_zones):
                    self.msg.lights[i].red = int(0)
                    self.msg.lights[i].blue = int(0)
                    self.msg.lights[i].green = int(0)
                self.colour = 0

        # Publish the message
        self.publisher.publish(self.msg)

    def start(self):
        self.publish_timer = self.create_timer(1 / self.publish_rate, self.publish_callback)

    def run_test(self):
        results = []
        self.test_in_progress = True

        self.start()

        # turn off all lights
        for i in range(self.light_zones):
            self.msg.lights[i].red = 0
            self.msg.lights[i].green = 0
            self.msg.lights[i].blue = 0
        self.publisher.publish(self.msg)
        user_input = self.promptYN('Are all lights off?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(TestResult(False, 'All lights off', notes))
        else:
            results.append(TestResult(True, 'All lights off', None))

        # turn all lights red
        for i in range(self.light_zones):
            self.msg.lights[i].red = 255
            self.msg.lights[i].green = 0
            self.msg.lights[i].blue = 0
        self.publisher.publish(self.msg)
        user_input = self.promptYN('Are all lights red?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(TestResult(False, 'All lights red', notes))
        else:
            results.append(TestResult(True, 'All lights red', None))

        # turn all lights green
        for i in range(self.light_zones):
            self.msg.lights[i].red = 0
            self.msg.lights[i].green = 255
            self.msg.lights[i].blue = 0
        self.publisher.publish(self.msg)
        user_input = self.promptYN('Are all lights green?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(TestResult(False, 'All lights green', notes))
        else:
            results.append(TestResult(True, 'All lights green', None))

        # turn all lights blue
        for i in range(self.light_zones):
            self.msg.lights[i].red = 0
            self.msg.lights[i].green = 0
            self.msg.lights[i].blue = 255
        self.publisher.publish(self.msg)
        user_input = self.promptYN('Are all lights blue?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(TestResult(False, 'All lights blue', notes))
        else:
            results.append(TestResult(True, 'All lights blue', None))

        # turn all lights white
        for i in range(self.light_zones):
            self.msg.lights[i].red = 255
            self.msg.lights[i].green = 255
            self.msg.lights[i].blue = 255
        self.publisher.publish(self.msg)
        user_input = self.promptYN('Are all lights white?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(TestResult(False, 'All lights white', notes))
        else:
            results.append(TestResult(True, 'All lights white', None))


        # test each corner
        for i in range(self.light_zones):
            if i in self.front_left:
                self.msg.lights[i].red = 255
                self.msg.lights[i].green = 255
                self.msg.lights[i].blue = 255
            else:
                self.msg.lights[i].red = 0
                self.msg.lights[i].green = 0
                self.msg.lights[i].blue = 0
        self.publisher.publish(self.msg)
        user_input = self.promptYN('Is the front-left light white and all other lights off?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(TestResult(False, 'Front left white', notes))
        else:
            results.append(TestResult(True, 'Front left white', None))

        for i in range(self.light_zones):
            if i in self.front_right:
                self.msg.lights[i].red = 255
                self.msg.lights[i].green = 255
                self.msg.lights[i].blue = 255
            else:
                self.msg.lights[i].red = 0
                self.msg.lights[i].green = 0
                self.msg.lights[i].blue = 0
        self.publisher.publish(self.msg)
        user_input = self.promptYN('Is the front-right light white and all other lights off?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(TestResult(False, 'Front right white', notes))
        else:
            results.append(TestResult(True, 'Front right white', None))

        for i in range(self.light_zones):
            if i in self.back_left:
                self.msg.lights[i].red = 255
                self.msg.lights[i].green = 255
                self.msg.lights[i].blue = 255
            else:
                self.msg.lights[i].red = 0
                self.msg.lights[i].green = 0
                self.msg.lights[i].blue = 0
        self.publisher.publish(self.msg)
        user_input = self.promptYN('Is the back-left light white and all other lights off?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(TestResult(False, 'Back left white', notes))
        else:
            results.append(TestResult(True, 'Back left white', None))

        for i in range(self.light_zones):
            if i in self.back_right:
                self.msg.lights[i].red = 255
                self.msg.lights[i].green = 255
                self.msg.lights[i].blue = 255
            else:
                self.msg.lights[i].red = 0
                self.msg.lights[i].green = 0
                self.msg.lights[i].blue = 0
        self.publisher.publish(self.msg)
        user_input = self.promptYN('Is the back-right light white and all other lights off?')
        if user_input == 'N':
            notes = input('Briefly describe the problem > ')
            results.append(TestResult(False, 'Back right white', notes))
        else:
            results.append(TestResult(True, 'Back right white', None))

        # TODO: Ridgeback has 2 lights per corner (top and bottom)
        # tests should be expanded to test those individually too
        # But for now just checking each corner is sufficient

        return results


def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    lt = LightTestNode(setup_path)

    try:
        lt.start()
        rclpy.spin(lt)
    except KeyboardInterrupt:
        pass

    lt.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()