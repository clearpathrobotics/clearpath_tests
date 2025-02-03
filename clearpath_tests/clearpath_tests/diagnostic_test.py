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
from clearpath_tests.test_node import ClearpathTestNode, ClearpathTestResult

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

import rclpy
from rclpy.time import Duration
from rclpy.qos import qos_profile_system_default


class DiagnosticTestNode(ClearpathTestNode):
    """
    Monitors diagnostic topics to make sure there aren't any warnings
    """
    def __init__(self, light_zones=4, setup_path='/etc/clearpath'):
        super().__init__('Diagnostics', 'diagnostic_test', setup_path)
        self.test_in_progress = False
        self.warnings = {}
        self.errors = {}

    def diagnostic_callback(self, diagnostic_array):
        """
        Check the statuses in the array for warnings & errors.

        @param diagnostic_array  The message received on the diagnostic topic
        """
        for status in diagnostic_array.status:
            key = f'{status.name}/{status.message}'

            if status.level == DiagnosticStatus.OK:
                pass
            elif status.level == DiagnosticStatus.WARN:
                if key not in self.warnings:
                    self.warnings[key] = status
                    if not self.test_in_progress:
                        self.get_logger().info(
                            f'Diagnostic warning: {status.name} {status.message}'
                        )
            elif status.level == DiagnosticStatus.ERROR:
                if key not in self.errors:
                    self.errors[key] = status
                    if not self.test_in_progress:
                        self.get_logger().info(
                            f'Diagnostic error: {status.name} {status.message}'
                        )
            elif status.level == DiagnosticStatus.STALE:
                pass

    def start(self):
        self.diagnostc_sub = self.create_subscription(
            DiagnosticArray,
            f'/{self.namespace}/diagnostics',
            self.diagnostic_callback,
            qos_profile_system_default
        )

    def run_test(self):
        results = []

        self.test_in_progress = True
        self.start()

        # collect 30s worth of data
        start_time = self.get_clock().now()
        end_time = start_time + Duration(seconds=30.0)

        self.get_logger().info('Collecting 30s of diagnostic data...')
        while self.get_clock().now() < end_time:
            rclpy.spin_once(self)

        if len(self.warnings) == 0 and len(self.errors) == 0:
            results.append(ClearpathTestResult(True, 'Diagnostics', 'No errors, no warnings'))
        elif len(self.errors) == 0:
            results.append(ClearpathTestResult(True, 'Diagnostics', f'No errors, {len(self.warnings)} warnings'))
        else:
            results.append(ClearpathTestResult(False, 'Diagnostics', f'{len(self.errors)} errors, {len(self.warnings)} warnings'))

        return results

    def get_test_result_details(self):
        details = None

        if len(self.errors) > 0:
            details = ''
            details += '\n#### Errors recorded\n\n'
            for err in self.errors.values():
                details += f'* {err.name}: {err.message}\n'

        if len(self.warnings) > 0:
            if details is None:
                details = ''
            details += '\n#### Warnings recorded\n\n'
            for warn in self.warnings.values():
                details += f'* {warn.name}: {warn.message}\n'

        return details


def main():
    setup_path = BaseGenerator.get_args()
    rclpy.init()

    dt = DiagnosticTestNode(setup_path)

    try:
        dt.start()
        rclpy.spin(dt)
    except KeyboardInterrupt:
        pass

    dt.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()