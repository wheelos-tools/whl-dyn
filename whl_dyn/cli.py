#!/usr/bin/env python3
"""
Command-line interface for whl-dyn.

This module provides the entry point for the 'whl-dyn' command.
It correctly launches streamlit without triggering warnings.
"""

import sys
import os
from pathlib import Path


def main():
    """Launch the whl-dyn Streamlit application."""
    # Get the package root directory
    # This file is at whl_dyn/cli.py
    # The app.py is at whl_dyn/ui/app.py
    package_root = Path(__file__).parent

    # Path to app.py
    app_py = package_root / "ui" / "app.py"

    # Build streamlit command
    cmd = [
        sys.executable,
        "-m",
        "streamlit",
        "run",
        str(app_py),
        "--server.headless",
        "true",
        "--server.runOnSave",
        "false",
        "--server.fileWatcherType",
        "none",
    ] + sys.argv[1:]

    # Replace current process with streamlit
    # This prevents multiple processes from being created
    os.execvp(sys.executable, cmd)


if __name__ == "__main__":
    main()
