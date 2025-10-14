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

import argparse
import sys
import os

import numpy as np

from modules.control.proto import calibration_table_pb2
from modules.control.proto.control_conf_pb2 import ControlConf
from util import get_pb_from_text_file


def load_calibration_raw_data(fn):
    speed_table = {}
    with open(fn, 'r') as f:
        for line in f:
            items = line.split(',')
            cmd = round(float(items[0]))
            speed = float(items[1])
            acc = round(float(items[2]), 2)
            if speed in speed_table:
                cmd_table = speed_table[speed]
                if cmd in cmd_table:
                    cmd_table[cmd].append(acc)
                else:
                    cmd_table[cmd] = [acc]
            else:
                cmd_table = {}
                cmd_table[cmd] = [acc]
                speed_table[speed] = cmd_table

    for speed in speed_table:
        cmd_table = speed_table[speed]
        for cmd in cmd_table:
            cmd_table[cmd] = round(np.mean(cmd_table[cmd]), 2)
    # After this the acc_list converted to an average float number.

    speed_table2 = {}
    for speed in speed_table:
        cmd_table = speed_table[speed]
        acc_table = {}
        for cmd in cmd_table:
            acc = cmd_table[cmd]
            if acc in acc_table:
                acc_table[acc].append(cmd)
            else:
                acc_table[acc] = [cmd]
        speed_table2[speed] = acc_table

    return speed_table2


def load_calibration_raw_data_old(fn):
    speed_table = {}
    with open(fn, 'r') as f:
        for line in f:
            items = line.split(',')
            cmd = round(float(items[0]))
            speed = float(items[1])
            acc = round(float(items[2]), 2)
            if speed in speed_table:
                acc_table = speed_table[speed]
                if acc in acc_table:
                    acc_table[acc].append(cmd)
                else:
                    acc_table[acc] = [cmd]
            else:
                acc_table = {}
                acc_table[acc] = [cmd]
                speed_table[speed] = acc_table

    return speed_table


def get_calibration_table_pb(speed_table):
    calibration_table_pb = calibration_table_pb2.ControlCalibrationTable()
    speeds = list(speed_table.keys())
    speeds.sort()
    for speed in speeds:
        acc_table = speed_table[speed]
        accs = list(acc_table.keys())
        accs.sort()
        for acc in accs:
            cmds = acc_table[acc]
            cmd = np.mean(cmds)
            item = calibration_table_pb.calibration.add()
            item.speed = speed
            item.acceleration = acc
            item.command = cmd
    return calibration_table_pb


def main():
    parser = argparse.ArgumentParser(
        description="Convert processed calibration data to Protocol Buffer format.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s -i old_control_conf.pb.txt -d result.csv -o control_conf.pb.txt
  %(prog)s --input-config old_control_conf.pb.txt --data-file result.csv --output control_conf.pb.txt
  %(prog)s -i old_control_conf.pb.txt -d result.csv --output-table calibration_table.pb.txt
  %(prog)s -i old_control_conf.pb.txt -d result.csv -o control_conf.pb.txt --output-table calibration_table.pb.txt
        """
    )

    parser.add_argument(
        "-i", "--input-config",
        help="Input control configuration file (old control conf pb text file)"
    )

    parser.add_argument(
        "-d", "--data-file",
        required=True,
        help="Input calibration data file (result.csv)"
    )

    parser.add_argument(
        "-o", "--output",
        default="control_conf.pb.txt",
        help="Output control configuration file (default: control_conf.pb.txt)"
    )

    parser.add_argument(
        "--output-table",
        help="Output calibration table to a separate file"
    )

    # For backward compatibility with old usage
    if len(sys.argv) == 3 and not any(arg.startswith('-') for arg in sys.argv[1:]):
        # If exactly two arguments without dashes, assume it's the old usage
        input_config = sys.argv[1]
        data_file = sys.argv[2]
        output_file = "control_conf.pb.txt"
        output_table_file = None
    else:
        args = parser.parse_args()
        data_file = args.data_file
        input_config = args.input_config
        output_file = args.output
        output_table_file = args.output_table

    # Check if input files exist
    if input_config and not os.path.exists(input_config):
        print(f"Error: Input control configuration file '{input_config}' does not exist.")
        sys.exit(1)

    if not os.path.exists(data_file):
        print(f"Error: Input data file '{data_file}' does not exist.")
        sys.exit(1)

    # Process the data
    print(f"Loading calibration data from '{data_file}'...")
    speed_table_dict = load_calibration_raw_data(data_file)

    print("Generating calibration table...")
    calibration_table_pb = get_calibration_table_pb(speed_table_dict)

    # Output calibration table if requested
    if output_table_file:
        print(f"Writing calibration table to '{output_table_file}'...")
        with open(output_table_file, 'w') as f:
            f.write(str(calibration_table_pb))
        print(f"Success! Calibration table saved to '{output_table_file}'")

    # Output control configuration if requested
    if input_config and output_file:
        print(f"Loading control configuration from '{input_config}'...")
        ctl_conf_pb = get_pb_from_text_file(input_config, ControlConf())

        print(f"Updating control configuration with new calibration table...")
        ctl_conf_pb.lon_controller_conf.calibration_table.CopyFrom(calibration_table_pb)

        print(f"Writing result to '{output_file}'...")
        with open(output_file, 'w') as f:
            f.write(str(ctl_conf_pb))

        print(f"Success! Control configuration saved to '{output_file}'")
    elif not input_config and output_file:
        print("Warning: --input-config is required to generate control configuration file.")


if __name__ == '__main__':
    main()