#!/usr/bin/env python3

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.types.platform import Platform
from clearpath_generator_common.common import BaseGenerator
from clearpath_platform_msgs.msg import Fans

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

class FanTest(Node):

    def __init__(self, setup_path='/etc/clearpath'):
        super().__init__('fan_test')
        # self.setup_path = setup_path

        # # Define paths
        # self.config_path = os.path.join(self.setup_path, 'robot.yaml')

        # # Parse YAML into config
        # self.clearpath_config = ClearpathConfig(self.config_path)
        # self.platform = self.clearpath_config.platform.get_platform_model()

        # Params
        self.fans_topic = self.get_parameter_or('fans_topic', 'platform/mcu/cmd_fans')
        self.publish_rate = self.get_parameter_or('publish_rate', 2)

        self.publisher = self.create_publisher(Fans, self.fans_topic, qos_profile_system_default)
        self.publish_timer = self.create_timer(1 / self.publish_rate, self.publish_callback)
        self.value = 255

    def publish_callback(self):
        # Define the message to be sent
        msg = Fans()
        for _ in range(4):
            msg.fans.append(self.value)
        # Publish the message
        self.publisher.publish(msg)


def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    fan_test = FanTest(setup_path)

    try:
        rclpy.spin(fan_test)
    except KeyboardInterrupt:
        pass

    fan_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()