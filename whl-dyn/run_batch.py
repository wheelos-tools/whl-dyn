#!/usr/bin/env python3

import argparse
import yaml
import numpy as np
import time
import subprocess
from itertools import product
from pathlib import Path

def parse_args():
    parser = argparse.ArgumentParser(
        description="Batch-run DataCollector over a grid of throttle/speed/brake settings")
    parser.add_argument(
        "--config", "-c",
        default="params.yaml",
        help="Path to the YAML parameter file")
    parser.add_argument(
        "--name", "-n",
        required=True,
        help="Unique name for this test batch (used to create output folder)")
    parser.add_argument(
        "--settling", "-s",
        type=float,
        help="Override the settling_time (seconds) from the config file")
    parser.add_argument(
        "--outroot", "-o",
        help="Override the output_root directory from the config file")
    return parser.parse_args()

def main():
    args = parse_args()

    # 1) Load configuration
    cfg = yaml.safe_load(open(args.config, "r"))

    # 2) Determine settling time and output root (allow overrides)
    settling = args.settling if args.settling is not None else cfg.get("settling_time", 5.0)
    outroot = Path(args.outroot) if args.outroot else Path(cfg.get("output_root", "./test_results"))

    # 3) Create batch output directory
    batch_dir = outroot / args.name
    batch_dir.mkdir(parents=True, exist_ok=True)
    print(f"Output root directory: {batch_dir}")

    # 4) Build the list of (throttle, speed, brake) tuples
    th_values = np.arange(
        cfg["throttle"]["start"],
        cfg["throttle"]["stop"] + 1e-6,
        cfg["throttle"]["step"]
    )
    br_values = np.arange(
        cfg["brake"]["start"],
        cfg["brake"]["stop"] + 1e-6,
        cfg["brake"]["step"]
    )
    speed_values = cfg["target_speed"]["values"]

    cmds = list(product(
        [round(x, 3) for x in th_values],
        [round(x, 2) for x in speed_values],
        [round(x, 3) for x in br_values]
    ))

    # 5) Run each test in series
    total = len(cmds)
    for idx, (th, sp, br) in enumerate(cmds, start=1):
        tag = f"t{th}_s{sp}_b{br}"
        outdir = batch_dir / tag
        outdir.mkdir(exist_ok=True)
        print(f"[{idx}/{total}] Testing {tag} → results in {outdir}")

        # Invoke your DataCollector wrapper script
        cmd = [
            "python", "run_data_collector.py",
            "--throttle", str(th),
            "--speed",    str(sp),
            "--brake",    str(br),
            "--outdir",   str(outdir)
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode != 0:
            print(f"ERROR: Test {tag} failed:\n{result.stderr}")
            return

        print(f"PASSED: {tag}. Waiting {settling} seconds before next test.")
        time.sleep(settling)

    print(f"\nBatch '{args.name}' completed: {total} tests run successfully.")

if __name__ == "__main__":
    main()
