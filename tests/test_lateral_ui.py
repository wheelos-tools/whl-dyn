import tempfile
from pathlib import Path
import pandas as pd
import yaml

from whl_dyn.ui.lateral import (
    _load_plan_cases,
    _find_case_runs,
    _get_case_status_info,
    _save_case_temp_plan,
    _build_lateral_plan_df,
)


def test_lateral_ui_plan_loading_and_temp_generation():
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_path = Path(tmp_dir)
        plan_path = tmp_path / "test_plan.yaml"
        sample_cases = [
            {"case_name": "case_chirp_1", "duration_sec": 10.0, "test_type": "chirp", "speed_gate": {"target_mps": 2.0}},
            {"case_name": "case_chirp_2", "duration_sec": 20.0, "test_type": "chirp", "speed_gate": {"target_mps": 2.0}},
        ]
        with open(plan_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(sample_cases, f)

        loaded = _load_plan_cases(plan_path)
        assert len(loaded) == 2
        assert loaded[0]["case_name"] == "case_chirp_1"

        df = _build_lateral_plan_df(loaded)
        assert len(df) == 2
        assert df.iloc[0]["case_name"] == "case_chirp_1"
        assert df.iloc[0]["target_speed_mps"] == 2.0

        temp_plan = _save_case_temp_plan(loaded[0], "case_chirp_1", tmp_path / "runtime")
        assert temp_plan.exists()
        with open(temp_plan, "r", encoding="utf-8") as f:
            single = yaml.safe_load(f)
        assert len(single) == 1
        assert single[0]["case_name"] == "case_chirp_1"


def test_lateral_ui_status_tracking():
    with tempfile.TemporaryDirectory() as tmp_dir:
        out_dir = Path(tmp_dir) / "runs"
        out_dir.mkdir()

        # Case 1: pending
        st_pending = _get_case_status_info("case_a", out_dir, None, None)
        assert st_pending["status_code"] == "pending"

        # Case 2: completed run
        run_a = out_dir / "case_a_20260820_120000"
        run_a.mkdir()
        df = pd.DataFrame({"t": range(50), "steer": range(50)})
        df.to_csv(run_a / "samples.csv", index=False)

        st_completed = _get_case_status_info("case_a", out_dir, None, None)
        assert st_completed["status_code"] == "completed"
        assert st_completed["rows"] == 50

        # Find runs
        runs = _find_case_runs(out_dir, "case_a")
        assert len(runs) == 1
        assert runs[0] == run_a
