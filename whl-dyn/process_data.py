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
This module provides functions to process vehicle dynamics data from CSV files.
It can process either a single CSV file or all CSV files in a directory.
"""

import argparse
import glob
import os
import sys

import numpy as np

from process import preprocess
from process import process


class Plotter(object):
    """
    Process and save vehicle dynamics data
    """

    def __init__(self, output_dir="./"):
        """
        Initialize the processor
        """
        np.set_printoptions(precision=3)
        self.output_dir = output_dir
        self.result_file_path = os.path.join(self.output_dir, 'result.csv')

        # Ensure output directory exists
        os.makedirs(self.output_dir, exist_ok=True)

    def process_data(self, filename):
        """
        Load the file and preprocess the data
        """
        print(f"Processing {filename}")
        self.data = preprocess(filename)

        self.tablecmd, self.tablespeed, self.tableacc, self.speedsection, self.accsection, self.timesection = process(
            self.data)

    def save_data(self, filename=None):
        """
        Save processed data to files
        """
        # Create individual result file if filename is provided
        if filename:
            individual_result_path = filename + ".result"
            with open(individual_result_path, 'w') as file_one:
                for i in range(len(self.tablecmd)):
                    for j in range(len(self.tablespeed[i])):
                        file_one.write("%s, %s, %s\n" %
                                      (self.tablecmd[i], self.tablespeed[i][j],
                                       self.tableacc[i][j]))
            print('Saved result to:', individual_result_path)

        # Append to main result file
        with open(self.result_file_path, 'a') as file:
            for i in range(len(self.tablecmd)):
                for j in range(len(self.tablespeed[i])):
                    file.write("%s, %s, %s\n" %
                              (self.tablecmd[i], self.tablespeed[i][j],
                               self.tableacc[i][j]))

    def clear_result_file(self):
        """
        Clear the main result file
        """
        if os.path.exists(self.result_file_path):
            os.remove(self.result_file_path)
            print(f"Cleared existing result file: {self.result_file_path}")


def process_single_file(filepath, output_dir="./"):
    """
    Process a single CSV file
    """
    plotter = Plotter(output_dir)
    plotter.process_data(filepath)
    plotter.save_data(filepath)
    return plotter.result_file_path


def process_directory(directory, output_dir="./", clear=False):
    """
    Process all _recorded.csv files in a directory
    """
    plotter = Plotter(output_dir)

    # Clear result file if requested
    if clear:
        plotter.clear_result_file()

    # Find all _recorded.csv files in the directory
    pattern = os.path.join(directory, "*_recorded.csv")
    files = glob.glob(pattern)

    if not files:
        print(f"No *_recorded.csv files found in {directory}")
        return plotter.result_file_path

    print(f"Found {len(files)} files to process")

    # Process each file
    for filepath in sorted(files):
        plotter.process_data(filepath)
        plotter.save_data(filepath)

    return plotter.result_file_path


def main():
    """
    Main function with command line argument parsing
    """
    parser = argparse.ArgumentParser(
        description="Process vehicle dynamics data from CSV files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s data.csv                           # Process a single file
  %(prog)s data/                              # Process all *_recorded.csv files in directory
  %(prog)s data/ -o results/ --clear          # Process with custom output and clear previous results
        """
    )

    parser.add_argument(
        "input_path",
        help="Input CSV file or directory containing CSV files"
    )

    parser.add_argument(
        "-o", "--output-dir",
        default="./",
        help="Output directory for results (default: current directory)"
    )

    parser.add_argument(
        "--clear",
        action="store_true",
        help="Clear existing result.csv file before processing"
    )

    # For backward compatibility with old usage
    if len(sys.argv) == 2 and not sys.argv[1].startswith('-'):
        # If only one argument and it doesn't start with '-', assume it's the old usage
        input_path = sys.argv[1]
    else:
        args = parser.parse_args()
        input_path = args.input_path

    # Check if input is a file or directory
    if os.path.isfile(input_path):
        # Process single file
        result_file = process_single_file(input_path, args.output_dir if 'args' in locals() else "./")
        print(f'Processing complete. Result saved to: {result_file}')
    elif os.path.isdir(input_path):
        # Process directory
        clear = args.clear if 'args' in locals() else False
        output_dir = args.output_dir if 'args' in locals() else "./"
        result_file = process_directory(input_path, output_dir, clear)
        print(f'Processing complete. Combined result saved to: {result_file}')
    else:
        print(f"Error: {input_path} is neither a file nor a directory")
        sys.exit(1)


if __name__ == '__main__':
    main()