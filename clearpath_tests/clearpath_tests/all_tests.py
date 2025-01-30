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

"""
This file is the main entrypoint for production tests.

This script will run through each of the tests needed for the configured platform
and log the results to a specified output file.

The output file is in markdown format. Markdown files are human-readable, but
you can also install markdown renderers for Chrome, Firefox, VSCode, and other
browsers & editors.
"""

import rclpy
from rclpy.node import Node

from datetime import datetime
import os

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.types.platform import Platform
from clearpath_config.common.utils.yaml import read_yaml

from clearpath_tests import (
    drive_test,
    fan_test,
    light_test,
    rotation_test,
    wifi_test,
)
from clearpath_tests.test_node import ClearpathTestResult


class TestingNode(Node):

    def __init__(self, node_name='clearpath_production_test_node'):
        super().__init__(node_name)

        self.setup_path = self.get_parameter_or('setup_path', '/etc/clearpath')
        self.setup_file = os.path.join(self.setup_path, 'robot.yaml')
        self.clearpath_config = ClearpathConfig(read_yaml(self.setup_file))

        self.platform = self.clearpath_config.get_platform_model()

        self.common_tests = [
            wifi_test.WifiTestNode(),
            rotation_test.RotationTestNode(),
            drive_test.DriveTestNode(),
        ]

        self.tests_for_platform = []

        if self.platform == Platform.A200:
            pass
        elif self.platform == Platform.A300:
            self.tests_for_platform.append(fan_test.FanTestNode(4))
            self.tests_for_platform.append(light_test.LightTestNode(4))
        elif(
            self.platform == Platform.DD100 or
            self.platform == Platform.DD150 or
            self.platform == Platform.DO100 or
            self.platform == Platform.DO150
        ):
            self.tests_for_platform.append(light_test.LightTestNode(4))
        elif self.platform == Platform.GENERIC:
            pass
        elif self.platform == Platform.J100:
            pass
        elif self.platform == Platform.R100:
            self.tests_for_platform.append(light_test.LightTestNode(8))
        elif self.platform == Platform.W200:
            self.tests_for_platform.append(light_test.LightTestNode(4))

        if os.environ['HOME']:
            default_log_dir = os.environ['HOME']
        else:
            self.get_logger().warning('$HOME is undefined; using /tmp as default report location')
            default_log_dir = '/tmp'

        self.report_file = self.get_parameter_or(
            'report_file',
            os.path.join(
                default_log_dir,
                f'clearpath_test_results.{datetime.now().strftime("%Y%m%d%H%M")}.md'
            )
        )

        self.test_results = []

    def destroy_node(self):
        """
        Clean up any in-progress tests.

        Also make sure the log file is closed.
        """
        return super().destroy_node()

    def create_summary(self):
        """
        Prints the summary table of all tests and appends the same table to the report.

        Also prints out a one/two line summary of the number of tests passed/failed.
        """
        longest_test_name = 'Test'  # initialize t row header
        longest_test_message = 'Notes'  # initialize to row header
        for result in self.test_results:
            if len(result.name) > len(longest_test_name):
                longest_test_name = result.name
            if result.message and len(result.message) > len(longest_test_message):
                longest_test_message = result.message

        test_column_width = len(longest_test_name)
        result_column_width = len('Result')
        message_column_width = len(longest_test_message)

        table_md = f'| {"Test".ljust(test_column_width)} | Result | {"Notes".ljust(message_column_width)} |\n'
        table_md = table_md + f'|-{"-"*test_column_width}-|-{"-"*result_column_width}-|-{"-"*message_column_width}-|\n'

        n_passed = 0
        n_failed = 0
        for result in self.test_results:
            if result.success is None:
                pass_fail = 'n/a'
            elif result.success:
                pass_fail = 'pass'
                n_passed += 1
            else:
                pass_fail = 'fail'
                n_failed += 1

            table_md = table_md + f'| {result.name.ljust(test_column_width)} | {pass_fail.ljust(result_column_width)} | {(result.message if result.message else "").ljust(message_column_width)} |\n'

        with open(self.report_file, 'a') as report:
            report.write('\n## Summary\n')
            report.write(table_md)
            report.write('\n')
            if n_failed == 0:
                report.write('- all tests passed')
            else:
                report.write(f'- {n_passed} tests passed\n')
                report.write(f'- {n_failed} tests failed\n')

        print(f'\n\nSummary:\n{table_md}')
        if n_failed == 0:
            print('\nAll tests passed!\n')
        else:
            print(f'\n{n_passed} tests passed\n{n_failed} tests failed\n')

    def write_header(self):
        """
        Initialize the report file.

        This dumps some initial meta-data and a copy of robot.yaml
        """
        with open(self.report_file, 'w') as report:
            report.write(f"""# Clearpath Test Report

{datetime.now().strftime('%Y-%m-%d %H:%M')}

Report generated by user {os.getlogin()}

Setup path: {self.setup_path}

Platform (serial): {self.clearpath_config.get_platform_model()} ({self.clearpath_config.get_serial_number()})

## robot.yaml
""")
        self.copy_file_contents(self.setup_file, 'yaml')

        with open(self.report_file, 'a') as report:
            report.write('\n## Test results\n\n')

    def copy_file_contents(self, path, format=None):
        """
        Copy the raw contents of a file into a code block

        @param path  The path of the file to copy
        @param format  Optional code formatting parameter (e.g. python, bash, yaml, xml, ...)
        """
        with open(path, 'r') as setup_file:
            file_contents = setup_file.readlines()
        self.write_code(file_contents, format)

    def write_code(self, code, format=None):
        """
        Add a code block to the report

        The code block will always have a newline added before and after it

        @param code  Either a string or an array of strings representing the lines of code
        @param format  The markdown-compatible language (e.g. python, bash, yaml, xml, ...)
        """

        with open(self.report_file, 'a') as report:
            if format:
                report.write(f'\n```{format}')
            else:
                report.write('\n```')

            if type(code) is str:
                if not code.startswith('\n'):
                    report.write('\n')
                report.write(code)
                if not code.endswith('\n'):
                    report.write('\n')
            else:
                if not code[0].startswith('\n'):
                    report.write('\n')
                for line in code:
                    report.write(line)
                if not code[-1].endswith('n'):
                    report.write('\n')

            report.write('```\n')

    def log_result(self, test_result: ClearpathTestResult):
        """
        Log the results of a test to the report

        @param test_result  The result we want to log
        """
        self.test_results.append(test_result)

        with open(self.report_file, 'a') as report:
            report.write(f'{test_result}\n\n')

    def run_tests(self):
        """
        Go through the tests one at a time, logging the results as we go

        The exact tests executed depends on the configured platform
        """
        self.write_header()

        tests_to_run = []
        for test in self.tests_for_platform:
            tests_to_run.append(test)
        for test in self.common_tests:
            tests_to_run.append(test)

        n = 1
        for node in tests_to_run:
            self.get_logger().info(f'Starting ({node.test_name}) (test {n} of {len(tests_to_run)})')
            user_input = node.promptYN(f'Run test {node.test_name}?')
            if user_input == 'N':
                self.get_logger().info(f'User skipped test {node.test_name}')
                with open(self.report_file, 'a') as report:
                    report.write(f'### {node.test_name}\n\n')
                self.log_result(ClearpathTestResult(None, node.test_name, 'Skipped'))
            else:
                try:
                    results = node.run_test()
                except Exception as err:
                    results = [ClearpathTestResult(False, node.test_name, str(err))]

                with open(self.report_file, 'a') as report:
                    report.write(f'### {node.test_name}\n\n')

                for result in results:
                    self.log_result(result)

            n += 1

        self.create_summary()


def main(args=None):
    rclpy.init(args=args)
    test_node = TestingNode()

    try:
        test_node.run_tests()
        print(f'\nTests complete. See {test_node.report_file} for full results')
    except FileNotFoundError as err:
        test_node.get_logger().error(f'Failed to write report: {err}')
    except PermissionError as err:
        test_node.get_logger().error(f'Insufficient permissions to write report: {err}')
    except KeyboardInterrupt:
        test_node.get_logger().info('User aborted')
    rclpy.shutdown()

if __name__ == '__main__':
    main()