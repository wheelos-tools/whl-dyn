import dataclasses
import html
import select
import subprocess
import sys
import time
from argparse import Namespace
from pathlib import Path
from typing import List, Optional

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


def check_csv_sanity(csv_path: Optional[Path]) -> dict:
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



def drain_logs(proc, log_lines: List[str]):
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


def get_metric_rating(value: float, metric_type: str) -> dict:
    """
    Get rating (grade 0-100, color, label) for a metric value.
    Returns position 0-100 for vertical scale marker.
    """
    if metric_type == 'mae':
        # MAE: 越小越好 (业界标准)
        if value < 0.10:
            return {'grade': 95, 'color': '#00C853', 'label': '优秀'}
        elif value < 0.20:
            return {'grade': 75, 'color': '#64DD17', 'label': '良好'}
        elif value < 0.35:
            return {'grade': 55, 'color': '#FFD600', 'label': '一般'}
        else:
            return {'grade': 25, 'color': '#FF3D00', 'label': '需改进'}

    elif metric_type == 'rmse':
        if value < 0.15:
            return {'grade': 95, 'color': '#00C853', 'label': '优秀'}
        elif value < 0.30:
            return {'grade': 75, 'color': '#64DD17', 'label': '良好'}
        elif value < 0.50:
            return {'grade': 55, 'color': '#FFD600', 'label': '一般'}
        else:
            return {'grade': 25, 'color': '#FF3D00', 'label': '需改进'}

    elif metric_type == 'r2':
        # R²: 参考指标，非主要评价标准
        if value >= 0.90:
            return {'grade': 95, 'color': '#00C853', 'label': '优秀'}
        elif value >= 0.80:
            return {'grade': 75, 'color': '#64DD17', 'label': '良好'}
        elif value >= 0.70:
            return {'grade': 55, 'color': '#FFD600', 'label': '一般'}
        else:
            return {'grade': 25, 'color': '#FF3D00', 'label': '需改进'}

    elif metric_type == 'deadzone':
        if value < 5:
            return {'grade': 95, 'color': '#00C853', 'label': '优秀'}
        elif value < 10:
            return {'grade': 75, 'color': '#64DD17', 'label': '良好'}
        elif value < 15:
            return {'grade': 55, 'color': '#FFD600', 'label': '一般'}
        else:
            return {'grade': 25, 'color': '#FF3D00', 'label': '偏大'}

    elif metric_type == 'smoothness':
        if value >= 90:
            return {'grade': 95, 'color': '#00C853', 'label': '优秀'}
        elif value >= 75:
            return {'grade': 75, 'color': '#64DD17', 'label': '良好'}
        elif value >= 60:
            return {'grade': 55, 'color': '#FFD600', 'label': '一般'}
        else:
            return {'grade': 25, 'color': '#FF3D00', 'label': '需改进'}

    elif metric_type == 'tolerance_pct':
        if value >= 90:
            return {'grade': 95, 'color': '#00C853', 'label': '优秀'}
        elif value >= 80:
            return {'grade': 75, 'color': '#64DD17', 'label': '良好'}
        elif value >= 70:
            return {'grade': 55, 'color': '#FFD600', 'label': '一般'}
        else:
            return {'grade': 25, 'color': '#FF3D00', 'label': '需改进'}

    elif metric_type == 'monotonicity':
        # 单调性违反次数：0为通过，>0为不通过
        if value == 0:
            return {'grade': 95, 'color': '#00C853', 'label': '通过'}
        else:
            return {'grade': 25, 'color': '#FF3D00', 'label': '违反'}

    return {'grade': 0, 'color': '#9E9E9E', 'label': '未知'}


def render_metric_compact(label: str, value: str, rating: dict, help_text: str = None):
    """Render a compact metric with color badge."""
    safe_label = html.escape(label)
    safe_value = html.escape(value)
    safe_rating_label = html.escape(rating['label'])
    help_attr = f' title="{html.escape(help_text, quote=True)}"' if help_text else ''
    st.markdown(f"""
    <div style="display: flex; justify-content: space-between; align-items: center; padding: 4px 0;">
        <span style="font-size: 13px; color: #CCC;"{help_attr}>{safe_label}</span>
        <div style="display: flex; align-items: center; gap: 8px;">
            <span style="font-size: 14px; font-weight: 600; color: {rating['color']};">{safe_value}</span>
            <span style="font-size: 10px; padding: 2px 6px; border-radius: 3px; background: {rating['color']}; color: #000;">{safe_rating_label}</span>
        </div>
    </div>
    """, unsafe_allow_html=True)


def render_metrics_grid(metrics_data: list, columns: int = 3):
    """Render multiple metrics in a grid layout with hover tooltip showing vertical scale."""
    # 添加CSS样式
    st.markdown("""
    <style>
    .metric-container {
        position: relative;
        padding: 6px 8px;
        background: #1A1A1A;
        border-radius: 6px;
        cursor: help;
    }
    .metric-name {
        font-size: 11px;
        color: #999;
        margin-bottom: 4px;
        white-space: nowrap;
        overflow: hidden;
        text-overflow: ellipsis;
    }
    .metric-value-row {
        display: flex;
        justify-content: space-between;
        align-items: center;
    }
    .metric-tooltip {
        display: none;
        position: absolute;
        left: 105%;
        top: 50%;
        transform: translateY(-50%);
        background: #2A2A2A;
        border: 1px solid #444;
        border-radius: 6px;
        padding: 8px 10px;
        z-index: 1000;
        box-shadow: 0 4px 12px rgba(0,0,0,0.5);
        white-space: nowrap;
    }
    .metric-container:hover .metric-tooltip {
        display: block;
    }
    .vertical-scale {
        display: flex;
        align-items: center;
        gap: 6px;
        height: 80px;
    }
    .scale-bar {
        width: 12px;
        height: 80px;
        border-radius: 6px;
        position: relative;
        background: linear-gradient(to top,
            #FF3D00 0%,   #FF3D00 40%,   /* 差: 0-40分 */
            #FFD600 40%,  #FFD600 70%,   /* 一般: 40-70分 */
            #64DD17 70%,  #64DD17 90%,   /* 良好: 70-90分 */
            #00C853 90%,  #00C853 100%   /* 优秀: 90-100分 */
        );
    }
    .scale-marker {
        position: absolute;
        left: -4px;
        width: 20px;
        height: 2px;
        background: #FFF;
        box-shadow: 0 0 6px rgba(0,0,0,0.9);
        border-radius: 1px;
    }
    .scale-labels {
        display: flex;
        flex-direction: column;
        justify-content: space-between;
        height: 80px;
        font-size: 9px;
        color: #888;
    }
    .tooltip-value {
        font-size: 11px;
        color: #FFF;
        margin-bottom: 4px;
        font-weight: bold;
    }
    .tooltip-hint {
        font-size: 10px;
        color: #AAA;
        margin-top: 4px;
        font-style: italic;
    }
    </style>
    """, unsafe_allow_html=True)

    for i in range(0, len(metrics_data), columns):
        cols = st.columns(columns)
        for j in range(columns):
            if i + j < len(metrics_data):
                with cols[j]:
                    item = metrics_data[i + j]
                    # Support both 3-tuple and 4-tuple
                    if len(item) == 4:
                        label, value, rating, help_text = item
                    else:
                        label, value, rating = item
                        help_text = None

                    # 计算marker位置 (grade 0-100，0在底部/差，100在顶部/优秀)
                    marker_pos = max(2, min(98, rating['grade']))

                    # Build tooltip HTML without escaping (since values are controlled)
                    tooltip_html = f'<div class="tooltip-value">{value}</div>'
                    if help_text:
                        tooltip_html += f'<div class="tooltip-hint">{help_text}</div>'
                    tooltip_html += f'''<div class="vertical-scale"><div class="scale-bar"><div class="scale-marker" style="bottom: {marker_pos}%;"></div></div><div class="scale-labels"><span>优秀</span><span>良好</span><span>一般</span><span>差</span></div></div>'''

                    html_content = f'''<div class="metric-container"><div class="metric-name">{label}</div><div class="metric-value-row"><span style="font-size: 13px; font-weight: 600; color: {rating['color']};">{value}</span><span style="font-size: 8px; padding: 1px 3px; border-radius: 2px; background: {rating['color']}; color: #000;">{rating['label']}</span></div><div class="metric-tooltip">{tooltip_html}</div></div>'''
                    st.markdown(html_content, unsafe_allow_html=True)


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


def _hash_config(config: CalibrationConfig):
    """Hash function for CalibrationConfig."""
    return hash(tuple(sorted(dataclasses.asdict(config).items())))


@st.cache_data(show_spinner=False, hash_funcs={CalibrationConfig: _hash_config})
def load_and_process(config: CalibrationConfig, directory: str):
    core = DataCore(config)
    core.load_data(directory)
    if not core.raw_dfs:
        return None, None, None, None, None, None
    core.process_signals()
    speed_grid, command_grid, grid_z = core.build_calibration_table()
    metrics = MetricsEvaluator.evaluate(speed_grid, command_grid, grid_z, core.processed_df)
    return core.unified_df, core.processed_df, speed_grid, command_grid, grid_z, metrics


init_state()
st.set_page_config(page_title="Chassis Dynamics Calibration", layout="wide", page_icon="🚗")

# 永久隐藏顶部菜单，节省空间
st.markdown("""
<style>
/* 隐藏顶部菜单 */
.stApp header {display: none !important;}

/* 隐藏deploy按钮 */
.stDeployButton {display: none !important;}

/* 减少顶部空白 */
.block-container {padding-top: 0.3rem !important;}

/* 压缩标题间距 */
.stTitle {padding: 0.1rem 1rem 0.2rem !important; font-size: 1.5rem !important;}
.stCaption {padding: 0 1rem 0.3rem 0 !important; font-size: 0.85rem !important;}

/* 隐藏running info等 */
.stStatusWidget {display: none !important;}
</style>
""", unsafe_allow_html=True)

st.title("Chassis Dynamics Calibration Dashboard")

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
        hold_duration = st.number_input("Hold after trigger (ms)", 0, 5000, 0, help="触发条件满足后保持时间(毫秒), 0表示立即切换")

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
            hold_duration_ms=int(hold_duration),
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
    # === 顶部路径输入 ===
    col_path1, col_path2 = st.columns(2)
    with col_path1:
        data_dir_text = st.text_input("Data directory", value=output_default(), key="analysis_data_dir")
        data_dir = resolve_path(data_dir_text)
    with col_path2:
        export_dir_text = st.text_input("Export directory", value=results_default(), key="analysis_export_dir")
        export_dir = resolve_path(export_dir_text)

    config = CalibrationConfig()

    # === 三列布局：参数 | 可视化 | 指标 ===
    param_col, viz_col, metrics_col = st.columns([1, 2.5, 1])

    # === 左列：参数调节（双列紧凑布局） ===
    with param_col:
        st.markdown("##### ⚙️ 参数")

        # 双列布局减少高度
        p1, p2 = st.columns(2)
        with p1:
            config.speed_source = st.selectbox("Speed", ["chassis", "localization"], help="速度数据来源: chassis=底盘, localization=定位")
        with p2:
            config.accel_source = st.selectbox("Accel", ["imu", "derivative"], help="加速度来源: imu=IMU传感器, derivative=速度微分")

        p3, p4 = st.columns(2)
        with p3:
            config.throttle_latency_ms = st.number_input("Throttle ms", 0, 500, 60, help="油门响应延迟补偿(毫秒)")
        with p4:
            config.brake_latency_ms = st.number_input("Brake ms", 0, 500, 60, help="刹车响应延迟补偿(毫秒)")

        p_stab1, p_stab2 = st.columns(2)
        with p_stab1:
            config.throttle_stability_window_ms = st.number_input("油门稳定窗口(ms)", 0, 1000, 200, help="命令切换后丢弃的时间窗口(毫秒)")
        with p_stab2:
            config.brake_stability_window_ms = st.number_input("刹车稳定窗口(ms)", 0, 1000, 300, help="命令切换后丢弃的时间窗口(毫秒)")

        # 速度范围过滤：双滑块分别控制油门和刹车
        st.markdown("**速度范围过滤**")
        throttle_min, throttle_max = st.slider(
            "油门速度范围(m/s)",
            0.0, 10.0, (0.0, 5.0),
            help="油门数据的速度过滤范围：低于最小值或高于最大值的数据将被丢弃"
        )
        config.min_throttle_speed_mps = throttle_min
        config.max_throttle_speed_mps = throttle_max

        brake_min, brake_max = st.slider(
            "刹车速度范围(m/s)",
            0.0, 10.0, (0.0, 5.0),
            help="刹车数据的速度过滤范围：低于最小值或高于最大值的数据将被丢弃"
        )
        config.min_brake_speed_mps = brake_min
        config.max_brake_speed_mps = brake_max

        config.lowpass_cutoff = st.slider("Filter Hz", 0.1, 5.0, 1.0, 0.1, help="低通滤波截止频率: 去除高频噪声,越小越平滑")

        p5, p6 = st.columns(2)
        with p5:
            config.command_resolution = st.slider("Cmd %", 1.0, 10.0, 5.0, 0.5, help="命令轴分辨率(%): 标定表步长, 越小越精细")
        with p6:
            config.speed_resolution = st.slider("Speed %", 0.1, 1.0, 0.2, 0.05, help="速度轴分辨率(m/s): 标定表步长, 越小越精细")

        config.enable_lof = st.checkbox("LOF", True, help="启用异常值检测: 自动过滤传感器噪声和异常数据点")
        if config.enable_lof:
            p7, p8 = st.columns(2)
            with p7:
                config.lof_neighbors = st.slider("Neighbors", 5, 100, 30, help="LOF邻居数: 判断异常值时参考的邻近点数量")
            with p8:
                config.lof_contamination = st.slider("Contam", 0.0, 0.1, 0.02, 0.005, format="%.3f", help="污染率: 预期异常值比例, 越小越严格")

    # === 加载数据 ===
    raw_df, clean_df, speed_grid, cmd_grid, accel_grid, metrics = load_and_process(config, str(data_dir))

    # === 中列：可视化 ===
    with viz_col:
        if raw_df is not None and speed_grid is not None and len(speed_grid) > 0:
            # 3D Surface Plot
            fig = go.Figure()
            fig.add_trace(go.Surface(z=accel_grid.T, x=cmd_grid, y=speed_grid, colorscale="RdBu", opacity=0.85))
            scatter_df = clean_df.sample(min(2000, len(clean_df)), random_state=0)
            fig.add_trace(go.Scatter3d(
                x=scatter_df["command"], y=scatter_df["final_speed"], z=scatter_df["accel_aligned"],
                mode="markers", marker={"size": 2, "color": "black"}, name="samples",
            ))
            fig.update_layout(
                scene={
                    "xaxis_title": "Cmd %",
                    "yaxis_title": "Speed m/s",
                    "zaxis_title": "Accel m/s²",
                    "camera": {
                        "eye": {"x": -1.5, "y": 1.5, "z": 1.5},
                        "center": {"x": 0, "y": 0, "z": 0},
                    }
                },
                height=500, margin={"l": 0, "r": 0, "t": 0, "b": 0}
            )
            st.plotly_chart(fig, use_container_width=True)

            # 底部导出按钮
            exp1, exp2 = st.columns(2)
            with exp1:
                if st.button("📥 Export", use_container_width=True):
                    export_dir.mkdir(parents=True, exist_ok=True)
                    Exporter.save_unified_csv(speed_grid, cmd_grid, accel_grid, export_dir)
                    Exporter.save_protobuf(speed_grid, cmd_grid, accel_grid, export_dir)
                    Exporter.save_metrics(metrics, export_dir)
                    st.success(f"Exported to {export_dir}")
            with exp2:
                if st.button("📈 Steps", use_container_width=True):
                    export_dir.mkdir(parents=True, exist_ok=True)
                    core = DataCore(config)
                    core.load_data(str(data_dir))
                    core.process_signals()
                    Exporter.save_step_responses(core.raw_dfs, export_dir)
                    st.success(f"Exported to {export_dir / 'step_responses'}")

            # 表格和响应曲线（折叠显示）
            with st.expander("📋 标定表 & 响应曲线"):
                tab1, tab2, tab3 = st.tabs(["油门表", "刹车表", "响应曲线"])
                throttle_table, brake_table = build_lookup_table_frames(speed_grid, cmd_grid, accel_grid)
                with tab1:
                    st.dataframe(throttle_table.style.format("{:.3f}"), use_container_width=True, height=300)
                with tab2:
                    st.dataframe(brake_table.style.format("{:.3f}"), use_container_width=True, height=300)
                with tab3:
                    st.plotly_chart(build_speed_slice_figure(speed_grid, cmd_grid, accel_grid), use_container_width=True)

                # 单个文件响应
                if "source_file" in clean_df.columns and len(clean_df) > 0:
                    st.markdown("---")
                    selected_file = st.selectbox("Step Response File", sorted(clean_df["source_file"].unique().tolist()))
                    step_df = clean_df[clean_df["source_file"] == selected_file].sort_values("time")
                    if not step_df.empty:
                        fig2 = go.Figure()
                        fig2.add_trace(go.Scatter(x=step_df["time"], y=step_df["command"], name="cmd"))
                        fig2.add_trace(go.Scatter(x=step_df["time"], y=step_df["raw_accel"] * 10.0, name="raw*10"))
                        fig2.add_trace(go.Scatter(x=step_df["time"], y=step_df["accel_aligned"] * 10.0, name="align*10"))
                        fig2.update_layout(height=300, xaxis_title="time", yaxis_title="scaled", margin={"l": 0, "r": 0, "t": 0, "b": 0})
                        st.plotly_chart(fig2, use_container_width=True)
        elif raw_df is None:
            st.warning(f"No data in: {data_dir}")

    # === 右列：质量指标 ===
    with metrics_col:
        if raw_df is not None:
            st.markdown("##### 📊 指标 Metrics")
            st.markdown("---")

            # 准备指标数据（2列显示）
            all_metrics = []

            t_dz = metrics.get('throttle_deadzone_pct', 0)
            b_dz = metrics.get('brake_deadzone_pct', 0)
            all_metrics.append(("油门死区", f"{t_dz:.0f}%", get_metric_rating(t_dz, 'deadzone')))
            all_metrics.append(("刹车死区", f"{b_dz:.0f}%", get_metric_rating(b_dz, 'deadzone')))

            t_r2 = metrics.get('throttle_linearity_R2', 0)
            b_r2 = metrics.get('brake_linearity_R2', 0)
            all_metrics.append(("油门线性度", f"{t_r2:.2f}", get_metric_rating(t_r2, 'r2'),
                               "参考指标"))
            all_metrics.append(("刹车线性度", f"{b_r2:.2f}", get_metric_rating(b_r2, 'r2'),
                               "参考指标"))

            smooth = metrics.get('smoothness_score_100', 0)
            all_metrics.append(("平滑度", f"{smooth:.0f}", get_metric_rating(smooth, 'smoothness')))

            max_accel = metrics.get('max_throttle_accel', 0)
            max_decel = metrics.get('max_brake_decel', 0)
            all_metrics.append(("响应范围", f"{max_accel:.1f}/{max_decel:.1f}", {'grade': 80, 'color': '#64DD17', 'label': 'm/s²'}))

            if metrics.get('residual_mae') is not None:
                mae = metrics.get('residual_mae', 0)
                rmse = metrics.get('residual_rmse', 0)
                all_metrics.append(("平均误差", f"{mae:.3f}", get_metric_rating(mae, 'mae')))
                all_metrics.append(("均方根误差", f"{rmse:.3f}", get_metric_rating(rmse, 'rmse')))

                tol_pct = metrics.get('within_tolerance_pct', 0)
                all_metrics.append(("拟合精度", f"{tol_pct:.0f}", get_metric_rating(tol_pct, 'tolerance_pct')))

            # 渲染指标（2列）
            render_metrics_grid(all_metrics, columns=2)

            # 单调性
            mono_pass = metrics.get('monotonicity_pass', False)
            t_vio = metrics.get('throttle_monotonic_violations', 0)
            b_vio = metrics.get('brake_monotonic_violations', 0)

            mono_color = '#00C853' if mono_pass else '#FF3D00'
            mono_status = '✓' if mono_pass else '✗'

            st.markdown(f"""
            <div style="margin-top:4px;padding:6px 8px;background: {'#1A2E1A' if mono_pass else '#2E1A1A'};border-radius:4px;border-left:2px solid {mono_color};">
                <div style="display:flex;justify-content:space-between;align-items:center;">
                    <span style="font-size:11px;color:#CCC;">单调性</span>
                    <span style="font-size:12px;font-weight:bold;color:{mono_color};">{mono_status}</span>
                </div>
                <div style="margin-top:2px;font-size:10px;color:#999;">油门:{t_vio} 刹车:{b_vio}</div>
            </div>
            """, unsafe_allow_html=True)

# === 侧边栏实用功能 ===
st.sidebar.markdown("### ⚡ 快捷操作")

# 缓存管理
if st.sidebar.button("🔄 清除缓存"):
    st.cache_data.clear()
    st.success("缓存已清除")
    st.rerun()

# 快速目录切换
st.sidebar.markdown("---")
st.sidebar.markdown("### 📁 快速目录")
quick_dir = st.sidebar.text_input("数据目录", value=output_default())
if st.sidebar.button("跳转到此目录"):
    st.session_state['analysis_data_dir'] = quick_dir
    st.rerun()

# 配置预设
st.sidebar.markdown("---")
st.sidebar.markdown("### 🔧 配置预设")
preset = st.sidebar.selectbox("选择预设", ["默认", "精细模式", "快速模式"])

# 根据预设自动设置参数
if preset == "默认":
    st.sidebar.caption("平衡精度和速度")
elif preset == "精细模式":
    st.sidebar.caption("高精度，慢处理")
elif preset == "快速模式":
    st.sidebar.caption("快速预览")

st.sidebar.markdown("---")
st.sidebar.markdown("### Runtime")
st.sidebar.text(str(Path(__file__).resolve()))
st.sidebar.text(time.strftime("%Y-%m-%d %H:%M:%S"))
