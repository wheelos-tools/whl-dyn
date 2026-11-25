"""Persistent, collision-free storage for vehicle-dynamics collection runs."""

import json
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

import pandas as pd
import yaml


class RunStorage:
    """Write one collected case into its own timestamped directory."""

    def __init__(self, output_root, case_name, metadata):
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        safe_name = "".join(
            character if character.isalnum() or character in "-_" else "_"
            for character in str(case_name)
        ).strip("_") or "unnamed_case"
        self.path = Path(output_root) / "{0}_{1}_{2}".format(
            timestamp, safe_name, uuid4().hex[:8])
        self.path.mkdir(parents=True, exist_ok=False)
        self.metadata_path = self.path / "metadata.yaml"
        self.samples_path = self.path / "samples.csv"
        with self.metadata_path.open("w") as metadata_file:
            yaml.safe_dump(metadata, metadata_file, sort_keys=False)

    def write_samples(self, samples):
        frame = pd.DataFrame(samples)
        if frame.empty:
            raise ValueError("cannot persist an empty collection")
        frame.to_csv(self.samples_path, index=False)
        return self.samples_path

    def write_metadata(self, metadata):
        with self.metadata_path.open("w") as metadata_file:
            yaml.safe_dump(metadata, metadata_file, sort_keys=False)

    def write_status(self, status):
        with (self.path / "status.json").open("w") as status_file:
            json.dump(status, status_file, indent=2, sort_keys=True)
