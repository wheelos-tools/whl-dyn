import select
import subprocess
import sys
import time
from argparse import Namespace
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import streamlit as st
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from whl_dyn.planning.generator import generate_calibration_plan
from whl_dyn.processing.config import CalibrationConfig
from whl_dyn.processing.data_core import DataCore
from whl_dyn.processing.exporter import Exporter
from whl_dyn.processing.metrics import MetricsEvaluator


PROJECT_ROOT = Path(__file__).resolve().parents[2]
RUNTIME_DIR = PROJECT_ROOT / ".whl_dyn_runtime"
RUNTIME_DIR.mkdir(parents=True, exist_ok=True)


def resolve_path(path_str: str) -> Path:
    path = Path(path_str)
    return path if path.is_absolute() else PROJECT_ROOT / path


def init_state():
    st.session_state.setdefault("case_state", {})
    st.session_state.setdefault(
        "collector_runtime",
        {
            "proc": None,
            "mode": "idle",
            "active_case": None,
            "batch_cases": [],
            "batch_index": 0,
            "logs": [],
            "last_returncode": None,
            "temp_plan": "",
            "output_dir": "",
            "quality": None,
            "awaiting_confirmation": False,
            "batch_done": False,
        },
    )


def load_plan(plan_path: Path):
    if not plan_path.exists():
        return []
    with open(plan_path, "r", encoding="utf-8") as f:
        plan = yaml.safe_load(f) or []
    return plan if isinstance(plan, list) else []


def save_single_case_plan(case: dict, case_name: str) -> Path:
    temp_plan_path = RUNTIME_DIR / f"{case_name}.yaml"
    with open(temp_plan_path, "w", encoding="utf-8") as f:
        yaml.safe_dump([case], f, sort_keys=False, default_flow_style=False, allow_unicode=True)
    return temp_plan_path


def parse_case_summary(case: dict) -> dict:
    name = case.get("case_name", "unknown")
    steps = case.get("steps", [])
    cmd0 = steps[0].get("command", {}) if steps else {}
    trg0 = steps[0].get("trigger", {}) if steps else {}
    case_type = "throttle" if name.startswith("throttle_") else "brake" if name.startswith("brake_") else "mixed"
    return {
        "case_name": name,
        "type": case_type,
        "step_count": len(steps),
        "cmd_throttle": float(cmd0.get("throttle", 0.0)),
        "cmd_brake": float(cmd0.get("brake", 0.0)),
        "trigger": trg0.get("type", ""),
        "trigger_value": float(trg0.get("value", 0.0)),
    }


def build_plan_df(plan: list) -> pd.DataFrame:
    columns = ["case_name", "type", "step_count", "cmd_throttle", "cmd_brake", "trigger", "trigger_value"]
    if not plan:
        return pd.DataFrame(columns=columns)
    return pd.DataFrame([parse_case_summary(case) for case in plan], columns=columns)


def get_case_state(case_name: str):
    st.session_state.case_state.setdefault(
        case_name,
        {
            "status": "pending",
            "retry": 0,
            "manual_confirmed": False,
            "last_check": "not_checked",
            "last_file": "",
            "rows": 0,
            "returncode": None,
        },
    )
    return st.session_state.case_state[case_name]


def set_case_status(case_name: str, **kwargs):
    state = get_case_state(case_name)
    state.update(kwargs)
    return state


def find_case_logs(output_dir: Path, case_name: str):
    return sorted(output_dir.glob(f"{case_name}_*.csv"), key=lambda path: path.stat().st_mtime)


def check_csv_sanity(csv_path: Path | None) -> dict:
    if csv_path is None or not csv_path.exists():
        return {"ok": False, "reason": "no_log", "rows": 0, "speed_span": 0.0, "command_span": 0.0}

    try:
        df = pd.read_csv(csv_path)
    except Exception as exc:
        return {"ok": False, "reason": f"read_failed: {exc}", "rows": 0, "speed_span": 0.0, "command_span": 0.0}

    required_cols = {"time", "speed_mps", "imu_accel_y", "ctl_throttle", "ctl_brake"}
    if not required_cols.issubset(df.columns):
        return {"ok": False, "reason": "missing_required_columns", "rows": int(len(df)), "speed_span": 0.0, "command_span": 0.0}

    speed_span = float(df["speed_mps"].max() - df["speed_mps"].min()) if len(df) else 0.0
    command_span = float((df["ctl_throttle"] - df["ctl_brake"]).abs().max()) if len(df) else 0.0

    if len(df) < 50:
        return {"ok": False, "reason": "too_few_rows", "rows": int(len(df)), "speed_span": speed_span, "command_span": command_span}
    if speed_span < 0.3:
        return {"ok": False, "reason": "speed_span_too_small", "rows": int(len(df)), "speed_span": speed_span, "command_span": command_span}
    if command_span < 5.0:
        return {"ok": False, "reason": "command_span_too_small", "rows": int(len(df)), "speed_span": speed_span, "command_span": command_span}

    return {"ok": True, "reason": "ok", "rows": int(len(df)), "speed_span": speed_span, "command_span": command_span}


def build_collector_command(plan_path: Path, output_dir: Path):
    return [
        sys.executable,
        "-m",
        "whl_dyn.collection.collector",
        "--plan",
        str(plan_path),
        "--output-dir",
        str(output_dir),
        "--auto-start",
    ]


def start_collection(case: dict, output_dir: Path, mode: str, batch_cases=None, batch_index: int = 0):
    runtime = st.session_state.collector_runtime
    case_name = case["case_name"]
    temp_plan = save_single_case_plan(case, case_name)
    cmd = build_collector_command(temp_plan, output_dir)
    proc = subprocess.Popen(
        cmd,
        cwd=str(PROJECT_ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    runtime.update(
        {
            "proc": proc,
            "mode": mode,
            "active_case": case_name,
            "batch_cases": batch_cases or [],
            "batch_index": batch_index,
            "logs": ["$ " + " ".join(cmd)],
            "last_returncode": None,
            "temp_plan": str(temp_plan),
            "output_dir": str(output_dir),
            "quality": None,
            "awaiting_confirmation": False,
            "batch_done": False,
        }
    )
    set_case_status(case_name, status="collecting", manual_confirmed=False, last_check="collecting", last_file="", rows=0, returncode=None)



def stop_collection():
    runtime = st.session_state.collector_runtime
    proc = runtime.get("proc")
    if proc and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
    runtime["last_returncode"] = proc.returncode if proc else None
    runtime["proc"] = None
    active_case = runtime.get("active_case")
    if active_case:
        set_case_status(active_case, status="stopped", last_check="stopped_by_user", returncode=runtime["last_returncode"])



def drain_logs(proc, log_lines: list[str]):
    if proc is None or proc.stdout is None:
        return
    while True:
        ready, _, _ = select.select([proc.stdout], [], [], 0)
        if not ready:
            break
        line = proc.stdout.readline()
        if not line:
            break
        log_lines.append(line.rstrip())



def finalize_current_case():
    runtime = st.session_state.collector_runtime
    active_case = runtime.get("active_case")
    output_dir = Path(runtime.get("output_dir", output_default()))
    logs = find_case_logs(output_dir, active_case) if active_case else []
    latest_log = logs[-1] if logs else None
    quality = check_csv_sanity(latest_log)
    runtime["quality"] = quality
    if active_case:
        set_case_status(
            active_case,
            status="quality_pass" if quality["ok"] else "quality_fail",
            last_check=quality["reason"],
            last_file=str(latest_log) if latest_log else "",
            rows=int(quality["rows"]),
            returncode=runtime.get("last_returncode"),
        )
    runtime["awaiting_confirmation"] = bool(quality["ok"])



def poll_runtime():
    runtime = st.session_state.collector_runtime
    proc = runtime.get("proc")
    if proc is None:
        return False

    drain_logs(proc, runtime["logs"])
    if proc.poll() is None:
        return True

    runtime["last_returncode"] = proc.returncode
    finalize_current_case()
    runtime["proc"] = None
    return False



def approve_and_continue(plan_lookup: dict):
    runtime = st.session_state.collector_runtime
    case_name = runtime.get("active_case")
    if case_name:
        set_case_status(case_name, status="approved", manual_confirmed=True)
    runtime["awaiting_confirmation"] = False

    if runtime.get("mode") != "batch":
        return

    next_index = runtime.get("batch_index", 0) + 1
    batch_cases = runtime.get("batch_cases", [])
    runtime["batch_index"] = next_index
    if next_index >= len(batch_cases):
        runtime["batch_done"] = True
        runtime["mode"] = "idle"
        return

    next_case_name = batch_cases[next_index]
    next_case = plan_lookup[next_case_name]
    start_collection(next_case, Path(runtime["output_dir"]), "batch", batch_cases=batch_cases, batch_index=next_index)



def retry_current_case(plan_lookup: dict):
    runtime = st.session_state.collector_runtime
    case_name = runtime.get("active_case")
    if not case_name:
        return
    state = get_case_state(case_name)
    state["retry"] += 1
    state["manual_confirmed"] = False
    case = plan_lookup[case_name]
    start_collection(case, Path(runtime["output_dir"]), runtime.get("mode", "single"), batch_cases=runtime.get("batch_cases", []), batch_index=runtime.get("batch_index", 0))



def output_default() -> str:
    return str(PROJECT_ROOT / "calibration_data_logs")


def results_default() -> str:
    return str(PROJECT_ROOT / "calibration_results")


def build_lookup_table_frames(speed_grid, command_grid, grid_z):
    if speed_grid is None or command_grid is None or grid_z is None:
        return pd.DataFrame(), pd.DataFrame()

    speed_labels = [f"{float(speed):.2f}" for speed in speed_grid]
    throttle_mask = command_grid >= 0
    brake_mask = command_grid <= 0

    throttle_df = pd.DataFrame(
        grid_z[throttle_mask, :],
        index=[f"{float(cmd):.1f}" for cmd in command_grid[throttle_mask]],
        columns=speed_labels,
    )
    brake_df = pd.DataFrame(
        grid_z[brake_mask, :],
        index=[f"{float(cmd):.1f}" for cmd in command_grid[brake_mask]],
        columns=speed_labels,
    )
    throttle_df.index.name = "command_pct"
    brake_df.index.name = "command_pct"
    return throttle_df, brake_df


def build_speed_slice_figure(speed_grid, command_grid, grid_z):
    fig = go.Figure()
    if speed_grid is None or command_grid is None or grid_z is None or len(speed_grid) == 0:
        return fig

    slice_count = min(5, len(speed_grid))
    slice_indices = np.linspace(0, len(speed_grid) - 1, num=slice_count, dtype=int)
    for index in slice_indices:
        fig.add_trace(
            go.Scatter(
                x=command_grid,
                y=grid_z[:, index],
                mode="lines+markers",
                name=f"speed={speed_grid[index]:.1f} m/s",
            )
        )
    fig.update_layout(
        height=420,
        xaxis_title="Command (%)",
        yaxis_title="Acceleration (m/s^2)",
        title="Calibration Response Curves by Speed Slice",
    )
    return fig


@st.cache_data(show_spinner=False, hash_funcs={CalibrationConfig: lambda _: None})
def load_and_process(config: CalibrationConfig, directory: str):
    core = DataCore(config)
    core.load_data(directory)
    if not core.raw_dfs:
        return None, None, None, None, None, None
    core.process_signals()
    speed_grid, command_grid, grid_z = core.build_calibration_table()
    metrics = MetricsEvaluator.evaluate(speed_grid, command_grid, grid_z)
    return core.unified_df, core.processed_df, speed_grid, command_grid, grid_z, metrics


init_state()
st.set_page_config(page_title="Chassis Dynamics Calibration", layout="wide", page_icon="🚗")
st.title("Chassis Dynamics Calibration Dashboard")
st.caption("Plan generation, real collection control, online QA, and WYSIWYG calibration analysis.")

plan_default = str(PROJECT_ROOT / "calibration_plan.yaml")
plan_tab, collect_tab, analysis_tab = st.tabs([
    "1. Plan Generation",
    "2. Data Collection Workflow",
    "3. Processing & Analytics",
])

with plan_tab:
    st.subheader("Generate Calibration Plan")
    col1, col2, col3 = st.columns(3)
    with col1:
        t_min = st.number_input("Throttle min (%)", 0, 100, 0)
        t_max = st.number_input("Throttle max (%)", 0, 100, 80)
        t_steps = st.number_input("Throttle steps", 1, 30, 5)
    with col2:
        b_min = st.number_input("Brake min (%)", 0, 100, 0)
        b_max = st.number_input("Brake max (%)", 0, 100, 50)
        b_steps = st.number_input("Brake steps", 1, 30, 5)
    with col3:
        speed_targets = st.text_input("Speed targets (m/s, comma)", "1.0,2.0")
        default_brake = st.number_input("Default stop brake (%)", 0.0, 100.0, 30.0)

    plan_path_text = st.text_input("Plan file", value=plan_default)
    plan_path = resolve_path(plan_path_text)

    if st.button("Generate plan", type="primary"):
        targets = [float(x.strip()) for x in speed_targets.split(",") if x.strip()]
        args = Namespace(
            output=str(plan_path),
            throttle_min=int(t_min),
            throttle_max=int(t_max),
            throttle_num_steps=int(t_steps),
            brake_min=int(b_min),
            brake_max=int(b_max),
            brake_num_steps=int(b_steps),
            speed_targets=targets,
            default_brake=float(default_brake),
            accel_timeout=30.0,
            decel_timeout=30.0,
        )
        generate_calibration_plan(args)
        st.success(f"Plan generated: {plan_path}")

    plan = load_plan(plan_path)
    plan_df = build_plan_df(plan)
    view_mode = st.radio("Plan view", ["Table", "Source"], horizontal=True)
    if view_mode == "Table":
        st.dataframe(plan_df, use_container_width=True)
    else:
        if plan_path.exists():
            st.code(plan_path.read_text(encoding="utf-8"), language="yaml")
        else:
            st.warning("Plan file not found.")

with collect_tab:
    running = poll_runtime()

    st.subheader("Real Collection Workflow")
    plan_path_text = st.text_input("Plan file for collection", value=plan_default, key="collect_plan")
    out_dir_text = st.text_input("Output data directory", value=output_default())
    plan_path = resolve_path(plan_path_text)
    output_dir = resolve_path(out_dir_text)
    output_dir.mkdir(parents=True, exist_ok=True)

    plan = load_plan(plan_path)
    plan_df = build_plan_df(plan)
    plan_lookup = {case["case_name"]: case for case in plan}

    st.dataframe(plan_df, use_container_width=True)

    if plan_df.empty:
        st.warning("No plan cases available. Generate or load a plan first.")
    else:
        selected_case = st.selectbox("Selected case", plan_df["case_name"].tolist())
        selected_state = get_case_state(selected_case)
        runtime = st.session_state.collector_runtime

        c1, c2, c3, c4 = st.columns(4)
        if c1.button("Start collection", disabled=running):
            start_collection(plan_lookup[selected_case], output_dir, "single")
            st.rerun()
        if c2.button("Stop collection", disabled=not running):
            stop_collection()
            st.rerun()
        if c3.button("Start batch executor", disabled=running):
            batch_cases = plan_df["case_name"].tolist()
            start_collection(plan_lookup[batch_cases[0]], output_dir, "batch", batch_cases=batch_cases, batch_index=0)
            st.rerun()
        if c4.button("Retry current case", disabled=running or not st.session_state.collector_runtime.get("active_case")):
            retry_current_case(plan_lookup)
            st.rerun()

        active_case = runtime.get("active_case") or selected_case
        active_state = get_case_state(active_case)
        st.markdown(f"**Active case:** {active_case}")
        st.markdown(f"**Status:** {active_state['status']}")
        st.markdown(f"**Last file:** {active_state['last_file'] or 'N/A'}")
        st.markdown(f"**Rows:** {active_state['rows']}")
        st.markdown(f"**Last check:** {active_state['last_check']}")
        st.markdown(f"**Retry count:** {active_state['retry']}")

        if runtime.get("quality"):
            quality = runtime["quality"]
            qc1, qc2, qc3 = st.columns(3)
            qc1.metric("Rows", quality.get("rows", 0))
            qc2.metric("Speed span", f"{quality.get('speed_span', 0.0):.2f}")
            qc3.metric("Command span", f"{quality.get('command_span', 0.0):.2f}")

        if runtime.get("awaiting_confirmation"):
            st.warning("Current case passed automatic checks. Manual confirmation is required before next step.")
            cc1, cc2 = st.columns(2)
            if cc1.button("Approve and continue"):
                approve_and_continue(plan_lookup)
                st.rerun()
            if cc2.button("Reject and retry"):
                retry_current_case(plan_lookup)
                st.rerun()

        if runtime.get("batch_done"):
            st.success("Batch execution completed.")

        status_rows = []
        for case_name in plan_df["case_name"].tolist():
            state = get_case_state(case_name)
            status_rows.append(
                {
                    "case_name": case_name,
                    "status": state["status"],
                    "manual_confirmed": state["manual_confirmed"],
                    "retries": state["retry"],
                    "last_check": state["last_check"],
                    "rows": state["rows"],
                }
            )
        st.subheader("Execution Progress")
        st.dataframe(pd.DataFrame(status_rows), use_container_width=True)

        st.subheader("Collector Command")
        if active_case in plan_lookup:
            temp_preview = RUNTIME_DIR / f"{active_case}.yaml"
            st.code(" ".join(build_collector_command(temp_preview, output_dir)), language="bash")

        st.subheader("Live Logs")
        log_text = "\n".join(runtime.get("logs", [])[-400:])
        st.text_area("collector_stdout", value=log_text, height=320, label_visibility="collapsed")

        if running:
            st.info("Collector process is running. Logs refresh automatically.")
            time.sleep(1)
            st.rerun()

with analysis_tab:
    st.subheader("WYSIWYG Processing and Analytics")
    data_dir_text = st.text_input("Data directory", value=output_default(), key="analysis_data_dir")
    data_dir = resolve_path(data_dir_text)
    export_dir_text = st.text_input("Export directory", value=results_default(), key="analysis_export_dir")
    export_dir = resolve_path(export_dir_text)

    config = CalibrationConfig()
    left, right = st.columns([1, 2])
    with left:
        config.speed_source = st.selectbox("Speed source", ["chassis", "localization"])
        config.accel_source = st.selectbox("Acceleration source", ["imu", "derivative"])
        config.lowpass_cutoff = st.slider("Low-pass cutoff (Hz)", 0.1, 5.0, 1.0, 0.1)
        config.throttle_latency_ms = st.number_input("Throttle latency (ms)", 0, 500, 60)
        config.brake_latency_ms = st.number_input("Brake latency (ms)", 0, 500, 60)
        config.enable_lof = st.checkbox("Enable LOF", True)
        config.lof_neighbors = st.slider("LOF neighbors", 5, 100, 30)
        config.lof_contamination = st.slider("LOF contamination", 0.0, 0.1, 0.02, 0.01)
        config.command_resolution = st.slider("Command resolution", 1.0, 10.0, 5.0, 1.0)
        config.speed_resolution = st.slider("Speed resolution", 0.1, 1.0, 0.2, 0.1)

    raw_df, clean_df, speed_grid, cmd_grid, accel_grid, metrics = load_and_process(config, str(data_dir))

    with right:
        if raw_df is None:
            st.warning(f"No valid CSV data in: {data_dir}")
        else:
            top1, top2, top3, top4 = st.columns(4)
            top1.metric("Raw rows", len(raw_df))
            top2.metric("Processed rows", len(clean_df))
            top3.metric("Throttle deadzone", f"{metrics.get('throttle_deadzone_pct', 0):.1f}%")
            top4.metric("Smoothness", f"{metrics.get('smoothness_score_100', 0):.1f}/100")

            export_col1, export_col2, export_col3 = st.columns(3)
            if export_col1.button("Export calibration tables"):
                export_dir.mkdir(parents=True, exist_ok=True)
                Exporter.save_unified_csv(speed_grid, cmd_grid, accel_grid, export_dir)
                Exporter.save_protobuf(speed_grid, cmd_grid, accel_grid, export_dir)
                Exporter.save_metrics(metrics, export_dir)
                st.success(f"Calibration tables exported to {export_dir}")
            if export_col2.button("Export step responses"):
                export_dir.mkdir(parents=True, exist_ok=True)
                core = DataCore(config)
                core.load_data(str(data_dir))
                core.process_signals()
                Exporter.save_step_responses(core.raw_dfs, export_dir)
                st.success(f"Step responses exported to {export_dir / 'step_responses'}")
            export_col3.caption(str(export_dir))

            if speed_grid is not None and len(speed_grid) > 0:
                fig = go.Figure()
                fig.add_trace(go.Surface(z=accel_grid.T, x=cmd_grid, y=speed_grid, colorscale="RdBu", opacity=0.85, name="surface"))
                scatter_df = clean_df.sample(min(2000, len(clean_df)), random_state=0)
                fig.add_trace(go.Scatter3d(
                    x=scatter_df["command"],
                    y=scatter_df["final_speed"],
                    z=scatter_df["accel_aligned"],
                    mode="markers",
                    marker={"size": 2, "color": "black"},
                    name="samples",
                ))
                fig.update_layout(scene={"xaxis_title": "Command (%)", "yaxis_title": "Speed (m/s)", "zaxis_title": "Accel (m/s^2)"}, height=620)
                st.plotly_chart(fig, use_container_width=True)

                throttle_table, brake_table = build_lookup_table_frames(speed_grid, cmd_grid, accel_grid)
                table_tab1, table_tab2, table_tab3 = st.tabs(["Throttle Table", "Brake Table", "Response Curves"])
                with table_tab1:
                    st.dataframe(throttle_table.style.format("{:.3f}"), use_container_width=True)
                with table_tab2:
                    st.dataframe(brake_table.style.format("{:.3f}"), use_container_width=True)
                with table_tab3:
                    st.plotly_chart(build_speed_slice_figure(speed_grid, cmd_grid, accel_grid), use_container_width=True)

            if "source_file" in clean_df.columns and len(clean_df) > 0:
                selected_file = st.selectbox("Step response file", sorted(clean_df["source_file"].unique().tolist()))
                step_df = clean_df[clean_df["source_file"] == selected_file].sort_values("time")
                if not step_df.empty:
                    fig2 = go.Figure()
                    fig2.add_trace(go.Scatter(x=step_df["time"], y=step_df["command"], name="command"))
                    fig2.add_trace(go.Scatter(x=step_df["time"], y=step_df["raw_accel"] * 10.0, name="raw_accel*10"))
                    fig2.add_trace(go.Scatter(x=step_df["time"], y=step_df["accel_aligned"] * 10.0, name="aligned_accel*10"))
                    fig2.update_layout(height=420, xaxis_title="time", yaxis_title="scaled value")
                    st.plotly_chart(fig2, use_container_width=True)

st.sidebar.markdown("### Runtime")
st.sidebar.text(str(Path(__file__).resolve()))
st.sidebar.text(time.strftime("%Y-%m-%d %H:%M:%S"))
