#!/usr/bin/env python3

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
"""
Data Collector
"""

import argparse
import os
import signal
import sys
import threading
import time

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.common_msgs.chassis_msgs import chassis_pb2
from modules.common_msgs.control_msgs import control_cmd_pb2
from modules.common_msgs.localization_msgs import localization_pb2


class DataCollector(object):
    """
    DataCollector Class
    """

    def __init__(self, node, output_dir="."):
        self.sequence_num = 0
        self.control_pub = node.create_writer('/apollo/control',
                                              control_cmd_pb2.ControlCommand)
        time.sleep(0.3)
        self.controlcmd = control_cmd_pb2.ControlCommand()

        self.canmsg_received = False
        self.localization_received = False

        self.case = 'a'
        self.in_session = False

        self.outfile = ""
        self.output_dir = output_dir
        self.file_lock = threading.Lock()

    def run(self, cmd):
        signal.signal(signal.SIGINT, self.signal_handler)

        # Validate command format
        if len(cmd) != 3:
            print("Error: Invalid command format. Expected 3 values: throttle speed_limit brake")
            return

        try:
            throttle = float(cmd[0])
            speed_limit = float(cmd[1])
            brake = float(cmd[2])
        except ValueError:
            print("Error: Invalid command values. All values must be numbers.")
            return

        # Validate command values according to new requirements
        if throttle < 0 or brake < 0:
            print("Error: Invalid command values. Throttle and brake must be positive values.")
            return

        if speed_limit <= 0:
            print("Error: Invalid command values. Speed limit must be positive.")
            return

        self.in_session = True
        self.cmd = [throttle, speed_limit, brake]

        # Generate filename with simplified format: throttle_speedlimit_brake
        out = f"t{int(throttle)}_s{int(speed_limit)}_b{int(brake)}"
        i = 0
        self.outfile = os.path.join(self.output_dir, out + f"_run{i}" + '_recorded.csv')
        while os.path.exists(self.outfile):
            i += 1
            self.outfile = os.path.join(self.output_dir, out + f"_run{i}" + '_recorded.csv')
        self.file = open(self.outfile, 'w')
        self.file.write(
            "time,io,ctlmode,ctlbrake,ctlthrottle,ctlgear_location," +
            "vehicle_speed,engine_rpm,driving_mode,throttle_percentage," +
            "brake_percentage,gear_location,imu\n"
        )

        print('Send Reset Command.')
        self.controlcmd.header.module_name = "control"
        self.controlcmd.header.sequence_num = self.sequence_num
        self.sequence_num = self.sequence_num + 1
        self.controlcmd.header.timestamp_sec = cyber_time.Time.now().to_sec()
        self.controlcmd.pad_msg.action = 2
        self.control_pub.write(self.controlcmd)

        time.sleep(0.2)
        # Set Default Message
        print('Send Default Command.')
        self.controlcmd.pad_msg.action = 1
        self.controlcmd.throttle = 0
        self.controlcmd.brake = 0
        self.controlcmd.steering_rate = 100
        self.controlcmd.steering_target = 0
        self.controlcmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
        self.controlcmd.motion_mode = chassis_pb2.Chassis.MOTION_ACKERMANN

        self.canmsg_received = False
        self.case = 'a'

        while self.in_session:
            now = cyber_time.Time.now().to_sec()
            self.publish_control()
            sleep_time = 0.01 - (cyber_time.Time.now().to_sec() - now)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def signal_handler(self, signal, frame):
        self.in_session = False
        # Ensure file is closed safely
        try:
            with self.file_lock:
                if hasattr(self, 'file') and not self.file.closed:
                    self.file.close()
        except:
            pass  # Ignore errors during shutdown

    def callback_localization(self, data):
        """
        New Localization
        """
        self.acceleration = data.pose.linear_acceleration_vrf.y
        self.localization_received = True

    def callback_canbus(self, data):
        """
        New CANBUS
        """
        if not self.localization_received:
            print('No Localization Message Yet')
            return
        timenow = data.header.timestamp_sec
        self.vehicle_speed = data.speed_mps
        self.engine_rpm = data.engine_rpm
        self.throttle_percentage = data.throttle_percentage
        self.brake_percentage = data.brake_percentage
        self.gear_location = data.gear_location
        self.driving_mode = data.driving_mode

        self.canmsg_received = True
        if self.in_session:
            self.write_file(timenow, 0)

    def publish_control(self):
        """
        New Control Command
        """
        if not self.canmsg_received:
            print('No CAN Message Yet')
            return

        self.controlcmd.header.sequence_num = self.sequence_num
        self.sequence_num += 1

        # Simplified logic with strict requirements:
        # - throttle and brake are always positive values
        # - always follow accelerate then decelerate pattern
        if self.case == 'a':
            # Acceleration phase: use throttle, no brake
            self.controlcmd.throttle = self.cmd[0]
            self.controlcmd.brake = 0
            if self.vehicle_speed >= self.cmd[1]:
                self.case = 'd'
        elif self.case == 'd':
            # Deceleration phase: use brake, no throttle
            self.controlcmd.throttle = 0
            self.controlcmd.brake = self.cmd[2]
            if self.vehicle_speed == 0:
                self.in_session = False

        self.controlcmd.header.timestamp_sec = cyber_time.Time.now().to_sec()
        self.control_pub.write(self.controlcmd)
        self.write_file(self.controlcmd.header.timestamp_sec, 1)
        if self.in_session == False:
            with self.file_lock:
                if hasattr(self, 'file') and not self.file.closed:
                    self.file.close()

    def write_file(self, time, io):
        """
        Write Message to File
        """
        with self.file_lock:
            self.file.write(
                "%.4f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n" %
                (time, io, 1, self.controlcmd.brake, self.controlcmd.throttle,
                 self.controlcmd.gear_location, self.vehicle_speed, self.engine_rpm,
                 self.driving_mode, self.throttle_percentage, self.brake_percentage,
                 self.gear_location, self.acceleration))
            self.file.flush()  # Ensure data is written to disk immediately


def main():
    """
    Main function
    """
    parser = argparse.ArgumentParser(description='Data Collector for Vehicle Dynamics Calibration')
    parser.add_argument('-o', '--output-dir', default='.',
                        help='Output directory for recorded data files (default: current directory)')
    parser.add_argument('-f', '--commands-file',
                        help='File containing commands to execute in batch mode')

    args = parser.parse_args()

    # Create output directory if it doesn't exist
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    node = cyber.Node("data_collector")

    data_collector = DataCollector(node, args.output_dir)
    node.create_reader('/apollo/localization/pose',
                       localization_pb2.LocalizationEstimate,
                       data_collector.callback_localization)
    node.create_reader('/apollo/canbus/chassis', chassis_pb2.Chassis,
                       data_collector.callback_canbus)

    # Batch mode: execute commands from file
    if args.commands_file:
        if os.path.exists(args.commands_file):
            with open(args.commands_file, 'r') as f:
                commands = f.readlines()

            for line_num, line in enumerate(commands, 1):
                line = line.strip()
                if not line or line.startswith('#'):  # Skip empty lines and comments
                    continue

                cmd = line.split()
                if len(cmd) == 3:
                    print(f"Executing command from line {line_num}: {line}")
                    # The run method will handle validation according to new requirements
                    data_collector.run(cmd)
                else:
                    print(f"Warning: Invalid command on line {line_num}: {line}")

            print("Batch execution completed.")
            return
        else:
            print(f"Error: Commands file '{args.commands_file}' not found.")
            return

    # Interactive mode
    print('Enter q to quit.')
    print('Enter x to remove result from last run.')
    print('Enter x y z, where x is throttle value (positive), ' +
          'y is speed limit (positive), z is brake value (positive).')
    print('All values must be positive numbers. The system will automatically ' +
          'follow accelerate then decelerate pattern.')
    print(f'Output directory: {args.output_dir}')

    while True:
        cmd = input("Enter commands: ").split()
        if len(cmd) == 0:
            print('Quiting.')
            break
        elif len(cmd) == 1:
            if cmd[0] == "q":
                break
            elif cmd[0] == "x":
                print('Removing last result.')
                if hasattr(data_collector, 'outfile') and os.path.exists(data_collector.outfile):
                    os.remove(data_collector.outfile)
                else:
                    print('File does not exist: %s' % (getattr(data_collector, 'outfile', 'Unknown')))
        elif len(cmd) == 3:
            data_collector.run(cmd)


if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()
