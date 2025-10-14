#!/usr/bin/env bash

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

# This script is deprecated. Please use the new process_data.py script instead.
# The new script provides all the same functionality with a better interface.

echo "WARNING: This script is deprecated."
echo "Please use the new process_data.py script instead:"
echo ""
echo "  # Process a single file:"
echo "  python process_data.py data.csv"
echo ""
echo "  # Process all *_recorded.csv files in a directory:"
echo "  python process_data.py data/"
echo ""
echo "  # Process with custom output directory and clear previous results:"
echo "  python process_data.py data/ -o results/ --clear"
echo ""

# For backward compatibility, we still support the old usage
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
python -W ignore "$DIR/process_data.py" "$@"