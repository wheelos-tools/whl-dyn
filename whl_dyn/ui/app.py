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

    # 确保输出目录字段有默认值
    if "collect_output_dir" not in st.session_state or not st.session_state["collect_output_dir"]:
        st.session_state["collect_output_dir"] = output_default()

    # 从输出目录恢复case状态
    output_dir = Path(st.session_state.get("output_dir", output_default()))
    if output_dir.exists():
        import glob
        csv_files = glob.glob(str(output_dir / "*.csv"))
        for csv_file in csv_files:
            csv_path = Path(csv_file)
            # 从文件名提取case名：格式为 {case_name}_{index}.csv
            case_name = "_".join(csv_path.stem.split("_")[:-1])  # 去掉最后的序号
            if case_name and case_name not in st.session_state.case_state:
                # 文件存在说明case已完成
                st.session_state.case_state.setdefault(case_name, {
                    "status": "completed",
                    "retry": 0,
                    "manual_confirmed": False,
                    "last_check": "ok",
                    "last_file": csv_path.name,
                    "rows": 0,  # 行数稍后可以读取CSV获取
                    "returncode": 0,
                })

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


def restore_case_state_from_files(output_dir: Path, case_name: str):
    """从CSV文件恢复case状态"""
    if not output_dir.exists():
        return

    # 查找所有匹配的CSV文件
    import glob
    case_files = glob.glob(str(output_dir / f"{case_name}_*.csv"))

    if not case_files:
        return

    # 获取最新的文件
    latest_file = max(case_files, key=lambda f: Path(f).stat().st_mtime)

    try:
        df = pd.read_csv(latest_file)
        row_count = len(df)

        # 更新状态
        state = get_case_state(case_name)
        if state["status"] == "pending" or state["rows"] == 0:
            state["status"] = "completed"
            state["rows"] = row_count
            state["last_file"] = Path(latest_file).name
            state["last_check"] = "ok"
    except Exception:
        pass


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

    # 发送SIGINT信号，让collector调用emergency_stop安全停止车辆
    if proc and proc.poll() is None:
        import signal
        try:
            proc.send_signal(signal.SIGINT)
            # 等待进程安全退出
            try:
                proc.wait(timeout=10)  # 给足够时间让车辆安全停止
            except subprocess.TimeoutExpired:
                # 超时则强制终止
                proc.kill()
        except Exception:
            # 如果发送信号失败，尝试终止
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    runtime["last_returncode"] = proc.returncode if proc else None
    runtime["proc"] = None
    runtime["mode"] = "idle"
    active_case = runtime.get("active_case")
    if active_case:
        set_case_status(active_case, status="stopped", last_check="stopped_by_user", returncode=runtime.get("last_returncode"))



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
    returncode = runtime.get("last_returncode")

    # 先检查是否有新生成的CSV文件
    logs = find_case_logs(output_dir, active_case) if active_case else []
    latest_log = logs[-1] if logs else None

    # 如果进程返回码非0，检查是否有文件生成
    if returncode is not None and returncode != 0:
        if latest_log:
            # 有文件生成，检查质量
            quality = check_csv_sanity(latest_log)
            runtime["quality"] = quality
            if active_case:
                set_case_status(
                    active_case,
                    status="quality_pass" if quality["ok"] else "quality_fail",
                    last_check=quality["reason"],
                    last_file=str(latest_log) if latest_log else "",
                    rows=int(quality["rows"]),
                    returncode=returncode,
                )
            # 采集失败（exit code != 0），不显示通过/打回按钮
            runtime["awaiting_confirmation"] = False
        else:
            # 没有文件生成，标记为错误
            if active_case:
                set_case_status(
                    active_case,
                    status="error",
                    last_check=f"process_failed: exit_code_{returncode}",
                    last_file="",
                    rows=0,
                    returncode=returncode,
                )
            runtime["awaiting_confirmation"] = False
        return

    # 进程正常退出，检查CSV文件质量
    quality = check_csv_sanity(latest_log)
    runtime["quality"] = quality
    if active_case:
        set_case_status(
            active_case,
            status="quality_pass" if quality["ok"] else "quality_fail",
            last_check=quality["reason"],
            last_file=str(latest_log) if latest_log else "",
            rows=int(quality["rows"]),
            returncode=returncode,
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



def approve_and_continue(plan_lookup: dict, plan_df=None):
    runtime = st.session_state.collector_runtime
    case_name = runtime.get("active_case")
    if case_name:
        set_case_status(case_name, status="approved", manual_confirmed=True)
    runtime["awaiting_confirmation"] = False
    # 清除active_case，强制使用selected_case
    runtime["active_case"] = None

    # 单用例模式：自动切换到下一个用例
    if runtime.get("mode") != "batch" and plan_df is not None and not plan_df.empty:
        case_list = plan_df["case_name"].tolist()
        current_idx = case_list.index(case_name) if case_name in case_list else -1
        if current_idx >= 0 and current_idx < len(case_list) - 1:
            # 切换到下一个用例
            st.session_state["selected_case_idx"] = current_idx + 1
        return

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



def retry_current_case(plan_lookup: dict, group_idx: int = 0, selected_case=None):
    """重试当前case：删除刚刚采集完成的那一组数据，重新执行"""
    runtime = st.session_state.collector_runtime

    # 获取输出目录 - 优先使用runtime中的，如果为空则从session state获取
    runtime_output_dir = runtime.get("output_dir", "")
    if not runtime_output_dir or runtime_output_dir == "":
        runtime_output_dir = st.session_state.get("output_dir", output_default())

    output_dir = Path(runtime_output_dir)

    # 如果在等待确认状态，使用active_case（刚采集完成的）
    # 否则使用selected_case（用户在下拉框中选择的）
    if runtime.get("awaiting_confirmation"):
        case_name = runtime.get("active_case")
    else:
        case_name = selected_case
    if not case_name:
        return

    if not output_dir.exists():
        return

    case_files = sorted(output_dir.glob(f"{case_name}_*.csv"))
    if case_files:
        # 删除最后一组（刚刚采集完成的）
        file_to_delete = case_files[-1]
        try:
            file_to_delete.unlink()
        except Exception:
            pass
        # 更新索引
        new_count = len(case_files) - 1
        if new_count > 0:
            st.session_state[f"{case_name}_selected_group"] = new_count - 1
        else:
            st.session_state[f"{case_name}_selected_group"] = 0

    # 记录是否在等待确认状态（在清除之前）
    was_awaiting = runtime.get("awaiting_confirmation", False)

    # 清除等待确认状态
    runtime["awaiting_confirmation"] = False

    # 如果不是在等待确认状态下重试（即用户手动选择用例重试），清除active_case避免混乱
    if not was_awaiting and runtime.get("active_case") != case_name:
        runtime["active_case"] = None

    # 重置状态
    state = get_case_state(case_name)
    state["retry"] += 1
    state["manual_confirmed"] = False
    state["last_file"] = ""
    state["rows"] = 0
    state["status"] = "pending"

    # 重新开始采集
    case = plan_lookup[case_name]
    start_collection(case, output_dir, runtime.get("mode", "single"),
                    batch_cases=runtime.get("batch_cases", []),
                    batch_index=runtime.get("batch_index", 0))


def delete_current_group(output_dir: Path, case_name: str, group_idx: int):
    """删除当前选中的组（单个CSV文件）"""
    if not case_name or group_idx < 0:
        return

    if output_dir.exists():
        import glob
        case_files = sorted(output_dir.glob(f"{case_name}_*.csv"))
        if 0 <= group_idx < len(case_files):
            file_to_delete = case_files[group_idx]
            try:
                file_to_delete.unlink()
            except Exception:
                pass

    # 更新状态：如果没有文件了，重置case状态
    case_files_after = list(output_dir.glob(f"{case_name}_*.csv")) if output_dir.exists() else []
    if not case_files_after:
        state = get_case_state(case_name)
        state["status"] = "pending"
        state["rows"] = 0
        state["last_file"] = ""

    # 重置组选择索引为合理的值
    new_count = len(case_files_after)
    if new_count == 0:
        new_idx = 0
    else:
        # 如果删除的是最后一组，选择新的最后一组；否则保持当前索引
        new_idx = min(group_idx, new_count - 1)
    st.session_state[f"{case_name}_selected_group"] = new_idx



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
        padding: 3px 6px;
        background: #1A1A1A;
        border-radius: 4px;
        cursor: help;
    }
    .metric-name {
        font-size: 10px;
        color: #999;
        margin-bottom: 2px;
        white-space: nowrap;
        overflow: hidden;
        text-overflow: ellipsis;
    }
    .metric-value-row {
        display: flex;
        justify-content: space-between;
        align-items: center;
    }
    .metric-value {
        font-size: 12px;
        font-weight: 600;
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

                    html_content = f'''<div class="metric-container"><div class="metric-name">{label}</div><div class="metric-value-row"><span class="metric-value" style="color: {rating['color']};">{value}</span><span style="font-size: 7px; padding: 1px 2px; border-radius: 2px; background: {rating['color']}; color: #000;">{rating['label']}</span></div><div class="metric-tooltip">{tooltip_html}</div></div>'''
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

/* 压缩所有元素的间距 */
div[data-testid="stVerticalBlock"] {
    gap: 0.1rem !important;
}

div[data-testid="column"] {
    gap: 0.1rem !important;
}

/* 压缩所有控件的内边距 - 使用更具体的选择器 */
div[data-testid="stNumberInput"] {
    padding: 0 !important;
}

div[data-testid="stNumberInput"] > div {
    padding: 0 !important;
}

div[data-testid="stSelectbox"] {
    padding: 0 !important;
}

div[data-testid="stSelectbox"] > div {
    padding: 0 !important;
}

div[data-testid="stSlider"] {
    padding: 0.1rem 0 !important;
}

div[data-testid="stSlider"] > div {
    padding: 0.1rem 0 !important;
}

div[data-testid="stCheckbox"] {
    padding: 0.1rem 0 !important;
}

div[data-testid="stRadio"] {
    padding: 0.1rem 0 !important;
}

[data-testid="stMarkdownContainer"] {
    margin-top: 0.1rem !important;
    margin-bottom: 0.1rem !important;
}

/* 压缩标题 */
h3 {
    margin-top: 0.2rem !important;
    margin-bottom: 0.2rem !important;
    font-size: 0.9rem !important;
}

h4 {
    margin-top: 0.1rem !important;
    margin-bottom: 0.1rem !important;
    font-size: 0.85rem !important;
}

/* 减小标签字体 */
label {
    font-size: 0.85rem !important;
}

/* 压缩元素间距 */
element-container {
    gap: 0.1rem !important;
}
</style>
""", unsafe_allow_html=True)

st.title("Chassis Dynamics Calibration Dashboard")

plan_default = str(PROJECT_ROOT / "calibration_plan.yaml")

# 主标签页自适应宽度
st.markdown("""
<style>
.stTabs [data-baseweb="tab-list"] {
    gap: 0px;
    width: 100%;
    display: flex;
}
.stTabs [data-baseweb="tab"] {
    flex-grow: 1;
    justify-content: center;
}
</style>
""", unsafe_allow_html=True)

plan_tab, collect_tab, analysis_tab = st.tabs([
    "📋 ① 生成计划",
    "🚗 ② 数据采集",
    "📊 ③ 分析",
])

with plan_tab:
    # 修复text_input label的margin-bottom对齐问题
    st.markdown("""
    <style>
    [data-testid="stTextInput"] label {
        margin-bottom: 0 !important;
    }
    </style>
    """, unsafe_allow_html=True)

    # 左侧参数网格，右侧操作区
    left_col, right_col = st.columns([2, 1])

    with left_col:
        # 第一行：油门参数（滑块 + 步数）
        t_col1, t_col2 = st.columns([4, 1])
        with t_col1:
            t_range = st.slider("油门范围 (%)", 0, 100, (0, 80))
            t_min, t_max = t_range
        with t_col2:
            t_steps = st.number_input("油门步数", 1, 30, 5)

        # 第二行：刹车参数（滑块 + 步数）
        b_col1, b_col2 = st.columns([4, 1])
        with b_col1:
            b_range = st.slider("刹车范围 (%)", 0, 100, (0, 80))
            b_min, b_max = b_range
        with b_col2:
            b_steps = st.number_input("刹车步数", 1, 30, 5)

        # 第三行：其他参数
        g1, g2, g3, g4 = st.columns([3, 1, 1, 1])
        with g1:
            speed_targets = st.text_input("速度目标 (m/s, 逗号分隔)", "3.0")
        with g2:
            default_throttle = st.number_input("默认油门 (%)", 0.0, 100.0, 80.0)
        with g3:
            default_brake = st.number_input("默认刹车 (%)", 0.0, 100.0, 30.0)
        with g4:
            hold_duration = st.number_input("保持时间 (ms)", 0, 5000, 500,
                                           help="触发条件满足后保持时间(毫秒), 0表示立即切换")

    with right_col:
        st.markdown("**操作**")
        plan_path_text = st.text_input("计划文件路径", value=plan_default, label_visibility="collapsed")
        st.write("")  # 添加间距
        if st.button("生成计划", type="primary"):
            plan_path = resolve_path(plan_path_text)
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
                default_throttle=float(default_throttle),
                default_brake=float(default_brake),
                hold_duration_ms=int(hold_duration),
                accel_timeout=30.0,
                decel_timeout=30.0,
            )
            generate_calibration_plan(args)
            st.success(f"计划已生成: {plan_path}")

    plan_path = resolve_path(plan_path_text)
    plan = load_plan(plan_path)
    plan_df = build_plan_df(plan)

    view_mode = st.radio("视图模式", ["表格", "源码"], horizontal=True, label_visibility="collapsed")
    if view_mode == "表格":
        st.dataframe(plan_df, use_container_width=True)
    else:
        if plan_path.exists():
            st.code(plan_path.read_text(encoding="utf-8"), language="yaml")
        else:
            st.warning("计划文件未找到。")

with collect_tab:
    running = poll_runtime()

    # 顶部：文件路径
    top1, top2 = st.columns([1, 1])
    with top1:
        plan_path_text = st.text_input("计划文件", value=plan_default, key="collect_plan")
    with top2:
        # 设置默认值，如果session state中有值就用，否则用默认值
        st.text_input("输出目录", key="collect_output_dir")

    # 执行全部逻辑（暂时注释掉按钮）
    # if st.button("执行全部", disabled=running):
    #     plan_path = resolve_path(plan_path_text)
    #     output_dir = resolve_path(out_dir_text)
    #     output_dir.mkdir(parents=True, exist_ok=True)
    #     plan = load_plan(plan_path)
    #     plan_df = build_plan_df(plan)
    #     if not plan_df.empty:
    #         batch_cases = plan_df["case_name"].tolist()
    #         plan_lookup = {case["case_name"]: case for case in plan}
    #         start_collection(plan_lookup[batch_cases[0]], output_dir, "batch",
    #                        batch_cases=batch_cases, batch_index=0)
    #         st.rerun()

    plan_path = resolve_path(plan_path_text)
    # 确保输出目录不为空
    out_dir_text = st.session_state.get("collect_output_dir", output_default())
    out_dir_resolved = resolve_path(out_dir_text) if out_dir_text.strip() else resolve_path(output_default())
    output_dir = out_dir_resolved
    # 同步到session state，让其他地方也能使用
    st.session_state["output_dir"] = str(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    plan = load_plan(plan_path)
    plan_df = build_plan_df(plan)
    plan_lookup = {case["case_name"]: case for case in plan}

    # 从现有CSV文件恢复case状态
    if not plan_df.empty:
        for case_name in plan_df["case_name"].tolist():
            restore_case_state_from_files(output_dir, case_name)

    runtime = st.session_state.collector_runtime

    # 初始化selected_case - 默认选中第一个没有数据的用例
    selected_case = None
    if not plan_df.empty:
        case_list = plan_df["case_name"].tolist()
        # 尝试找到第一个pending状态的用例
        if "selected_case_idx" not in st.session_state:
            for idx, case_name in enumerate(case_list):
                state = get_case_state(case_name)
                if state["status"] == "pending" or state["rows"] == 0:
                    st.session_state["selected_case_idx"] = idx
                    break
        current_idx = st.session_state.get("selected_case_idx", 0)
        current_idx = min(current_idx, len(case_list) - 1)
        selected_case = case_list[current_idx]

    # 左右分栏布局（左侧更窄）
    left_col, right_col = st.columns([1, 2])

    with left_col:
        st.markdown("**用例选择**")
        # 选择用例 + 上一个/下一个按钮
        if not plan_df.empty:
            case_list = plan_df["case_name"].tolist()
            current_idx = st.session_state.get("selected_case_idx", 0)
            current_idx = min(current_idx, len(case_list) - 1)

            # 上一个、选择器、下一个 在同一行
            nav1_col, select_col, nav2_col = st.columns([1, 4, 1])
            with nav1_col:
                if st.button("◀", disabled=current_idx <= 0):
                    st.session_state["selected_case_idx"] = current_idx - 1
                    st.rerun()
            with select_col:
                selected_case = st.selectbox("选择用例", case_list, index=current_idx, label_visibility="collapsed")
                # 更新索引
                current_idx = case_list.index(selected_case)
                st.session_state["selected_case_idx"] = current_idx
            with nav2_col:
                if st.button("▶", disabled=current_idx >= len(case_list) - 1):
                    st.session_state["selected_case_idx"] = current_idx + 1
                    st.rerun()

        st.markdown("**执行进度**")
        # 进度列表（紧凑）
        if not plan_df.empty:
            status_html = """<style>
.progress-list { display: flex; flex-direction: column; gap: 4px; }
.progress-row { display: grid; grid-template-columns: 1fr 50px 40px 70px; gap: 8px; padding: 6px 8px; border-radius: 4px; font-size: 0.85rem; align-items: center; }
.progress-row > * { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.row-pending { background: #f5f5f5; color: #666; }
.row-running { background: #fff3cd; color: #856404; }
.row-completed { background: #d4edda; color: #155724; }
.row-stopped { background: #f8d7da; color: #721c24; }
.row-error { background: #f8d7da; color: #721c24; }
.row-warning { background: #fff3cd; color: #856404; }
.row-quality_pass { background: #d4edda; color: #155724; }
.row-quality_fail { background: #f8d7da; color: #721c24; }
.row-approved { background: #d4edda; color: #155724; }
.row-selected { border: 2px solid #007bff !important; }
</style>
<div class="progress-list">"""

            for idx, case_name in enumerate(plan_df["case_name"].tolist()):
                state = get_case_state(case_name)
                status = state["status"]

                # 获取该case的组数和总行数
                group_count = 0
                total_rows = 0
                if output_dir.exists():
                    import glob
                    import pandas as pd
                    case_files = glob.glob(str(output_dir / f"{case_name}_*.csv"))
                    group_count = len(case_files)
                    # 计算所有组的总行数
                    for file_path in case_files:
                        try:
                            df = pd.read_csv(file_path)
                            total_rows += len(df)
                        except:
                            pass

                row_class = f"row-{status}"
                status_icon = "⏳"
                if status == "completed" or status == "quality_pass" or status == "approved":
                    status_icon = "✓"
                elif status == "running":
                    status_icon = "▶"
                elif status == "error" or status == "stopped" or status == "quality_fail":
                    status_icon = "✗"
                elif status == "warning":
                    status_icon = "⚠"

                case_name_short = case_name[:25] + "..." if len(case_name) > 25 else case_name

                # 检查是否是当前选中的用例，添加高亮class
                selected_class = " row-selected" if case_name == selected_case else ""

                status_html += f'<div class="progress-row {row_class}{selected_class}"><span>{case_name_short}</span><span style="text-align:center;">{group_count}组</span><span style="text-align:center;">{status_icon}</span><span style="text-align:right;">{total_rows}</span></div>'

            status_html += "</div>"
            st.markdown(status_html, unsafe_allow_html=True)

    with right_col:
        # 当前状态标题 + 组选择器
        col_title, col_select = st.columns([4, 2])
        with col_title:
            st.markdown("**当前状态**")
        # 确定当前active_case（优先使用运行中的case，否则用选中的case）
        active_case = runtime.get("active_case") or selected_case

        # 初始化变量
        group_idx = 0
        case_files = []

        # 组选择器列
        with col_select:
            # 获取当前case的所有文件
            if output_dir.exists() and active_case:
                case_files = sorted(output_dir.glob(f"{active_case}_*.csv"))
                # 如果有新文件，自动选择最新的
                if case_files:
                    current_idx = st.session_state.get(f"{active_case}_selected_group", len(case_files) - 1)
                    # 如果当前索引超出范围或者刚刚完成采集，选择最新的
                    if current_idx >= len(case_files) or (not running and runtime.get("last_returncode") is not None):
                        st.session_state[f"{active_case}_selected_group"] = len(case_files) - 1
            else:
                case_files = []

            if case_files:
                file_options = [f"第 {i+1} 组" for i in range(len(case_files))]
                selected_group_idx = st.session_state.get(f"{active_case}_selected_group", len(case_files) - 1)
                selected_group_idx = min(selected_group_idx, len(case_files) - 1)
                # selectbox返回的是值，不是索引，所以这里我们保存索引
                selected_label = st.selectbox("组", file_options, index=selected_group_idx, label_visibility="collapsed")
                group_idx = file_options.index(selected_label)  # 从值获取索引
                st.session_state[f"{active_case}_selected_group"] = group_idx
            else:
                st.write("-")

        # 显示选中组的信息
        if case_files:
            selected_file = case_files[group_idx]
            try:
                import pandas as pd
                df = pd.read_csv(selected_file)
                rows_count = len(df)
                file_name = selected_file.name
            except:
                rows_count = 0
                file_name = selected_file.name
        else:
            rows_count = 0
            file_name = 'N/A'
            if active_case:
                active_state = get_case_state(active_case)
                rows_count = active_state['rows']
                file_name = active_state['last_file'] or 'N/A'

        # 状态信息第一行（3列）
        col1, col2, col3 = st.columns(3)
        with col1:
            st.caption("数据行数")
            st.text(str(rows_count))
        with col2:
            st.caption("质量检查")
            if running:
                st.text("采集中...")
            elif case_files:
                st.text("已完成")
            else:
                if active_case:
                    active_state = get_case_state(active_case)
                    st.text(active_state['last_check'] or '-')
                else:
                    st.text("-")
        with col3:
            st.caption("用例状态")
            if running:
                st.text("running")
            elif active_case:
                active_state = get_case_state(active_case)
                st.text(active_state['status'])
            else:
                st.text("-")

        # 文件信息（单独一行）
        st.caption("文件")
        st.text(file_name)

        # 操作按钮（固定6列布局，避免按钮宽度变化）
        awaiting = runtime.get("awaiting_confirmation")
        btn_cols = st.columns(6)

        # 判断是否是最后一组
        is_last_group = not case_files or group_idx == len(case_files) - 1

        # 开始按钮 - 只有在查看最后一组时才能点击
        start_icon = "▶" if not running else "🔄"
        with btn_cols[0]:
            if st.button(f"{start_icon} 开始", disabled=running or not is_last_group):
                start_collection(plan_lookup[selected_case], output_dir, "single")
                st.rerun()
        with btn_cols[1]:
            if st.button("⏹ 停止", disabled=not running):
                stop_collection()
                st.rerun()
        with btn_cols[2]:
            # 重试按钮 - 总是可用（对于当前选中的组）
            if st.button("↺ 重试", disabled=running):
                retry_current_case(plan_lookup, group_idx, selected_case)
                st.rerun()
        with btn_cols[3]:
            # 清除按钮 - 删除当前选中的组（等待确认时禁用）
            if st.button("🗑 清除", disabled=running or not case_files or awaiting):
                if selected_case:
                    delete_current_group(output_dir, selected_case, group_idx)
                st.rerun()

        # 通过和打回按钮（固定在最后两列）
        with btn_cols[4]:
            if awaiting:
                if st.button("✓ 通过", key="btn_approve"):
                    approve_and_continue(plan_lookup, plan_df)
                    st.rerun()
        with btn_cols[5]:
            if awaiting:
                if st.button("✗ 打回", key="btn_reject"):
                    # 打回：删除当前选中的那一组数据，不自动重新执行
                    runtime["awaiting_confirmation"] = False
                    active_case_name = runtime.get("active_case")
                    if active_case_name:
                        output_dir_path = Path(runtime.get("output_dir", ""))
                        if output_dir_path.exists():
                            case_files = sorted(output_dir_path.glob(f"{active_case_name}_*.csv"))
                            if case_files and 0 <= group_idx < len(case_files):
                                # 删除当前选中的组
                                file_to_delete = case_files[group_idx]
                                try:
                                    file_to_delete.unlink()
                                except Exception:
                                    pass
                                # 更新索引
                                new_count = len(case_files) - 1
                                if new_count > 0:
                                    st.session_state[f"{active_case_name}_selected_group"] = min(group_idx, new_count - 1)
                                else:
                                    st.session_state[f"{active_case_name}_selected_group"] = 0
                        # 重置状态
                        state = get_case_state(active_case_name)
                        state["status"] = "pending"
                        state["last_check"] = "rejected"
                        state["last_file"] = ""
                        state["rows"] = 0
                    st.rerun()

        st.markdown("**实时日志**")
        log_text = "\n".join(runtime.get("logs", [])[-200:])
        st.text_area("日志", value=log_text, height=300, label_visibility="collapsed", key="log_area")

        if running:
            time.sleep(1)
            st.rerun()

with analysis_tab:
    # 注入超紧凑样式到整个标签页
    st.markdown("""
    <style>
    /* 强制压缩所有控件高度和间距 */
    .main [data-testid="stVerticalBlock"] > div {
        gap: 0 !important;
    }
    .main [data-testid="column"] > div {
        gap: 0 !important;
    }
    .main [data-testid="column"] > div > div {
        gap: 0 !important;
        padding: 0 !important;
    }
    /* 强制减小控件内部高度 */
    [data-testid="stNumberInput"] > div {
        padding: 0 !important;
        min-height: 25px !important;
    }
    [data-testid="stSelectbox"] > div {
        padding: 0 !important;
        min-height: 25px !important;
    }
    [data-testid="stSlider"] > div {
        padding: 2px 0 !important;
        min-height: 35px !important;
    }
    [data-testid="stCheckbox"] > div {
        padding: 0 !important;
        min-height: 25px !important;
    }
    /* 强制减小字体 */
    [data-testid="stNumberInput"] label,
    [data-testid="stSelectbox"] label,
    [data-testid="stSlider"] label,
    [data-testid="stCheckbox"] label {
        font-size: 0.75rem !important;
        line-height: 1rem !important;
        margin-bottom: 0 !important;
    }
    /* 减小markdown间距 */
    [data-testid="stMarkdownContainer"] {
        margin: 0 !important;
    }
    /* 减小标题间距 */
    h3, h4, h5 {
        margin: 0.1rem 0 !important;
        padding: 0 !important;
    }
    /* 减小hr分隔线高度和间距 */
    hr {
        margin: 0.2rem 0 !important;
        border: none !important;
        border-top: 1px solid #444 !important;
    }
    /* 减小行间距 */
    .element-container {
        margin-bottom: 0 !important;
    }
    /* 减小粗体文字间距 */
    strong {
        margin: 0.1rem 0 !important;
    }
    /* 调整指标标题间距 */
    .main [data-testid="column"] strong {
        margin-top: 0.5rem !important;
        margin-bottom: 0.1rem !important;
    }
    </style>
    """, unsafe_allow_html=True)

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

    # === 左列：参数调节 ===
    with param_col:
        st.markdown("##### ⚙️ 参数")

        p1, p2 = st.columns(2)
        with p1:
            config.speed_source = st.selectbox("Speed", ["chassis", "localization"])
        with p2:
            config.accel_source = st.selectbox("Accel", ["imu", "derivative"])

        p3, p4 = st.columns(2)
        with p3:
            config.throttle_latency_ms = st.number_input("Throttle ms", 0, 500, 60)
        with p4:
            config.brake_latency_ms = st.number_input("Brake ms", 0, 500, 60)

        p_stab1, p_stab2 = st.columns(2)
        with p_stab1:
            config.throttle_stability_window_ms = st.number_input("油门稳定窗口", 0, 1000, 200)
        with p_stab2:
            config.brake_stability_window_ms = st.number_input("刹车稳定窗口", 0, 1000, 300)

        # 合并速度范围
        speed_min, speed_max = st.slider("速度范围(m/s)", 0.0, 10.0, (0.0, 5.0))
        config.min_throttle_speed_mps = speed_min
        config.max_throttle_speed_mps = speed_max
        config.min_brake_speed_mps = speed_min
        config.max_brake_speed_mps = speed_max

        config.lowpass_cutoff = st.slider("Filter Hz", 0.1, 5.0, 1.0, 0.1)

        p5, p6 = st.columns(2)
        with p5:
            config.command_resolution = st.slider("Cmd %", 1.0, 10.0, 5.0, 0.5)
        with p6:
            config.speed_resolution = st.slider("Speed m/s", 0.1, 1.0, 0.2, 0.05)

        config.enable_lof = st.checkbox("LOF", True)
        if config.enable_lof:
            p7, p8 = st.columns(2)
            with p7:
                config.lof_neighbors = st.slider("Neighbors", 5, 100, 30)
            with p8:
                config.lof_contamination = st.slider("Contam", 0.0, 0.1, 0.02, 0.005, format="%.3f")

    # === 加载数据 ===
    raw_df, clean_df, speed_grid, cmd_grid, accel_grid, metrics = load_and_process(config, str(data_dir))

    # === 中列：可视化 ===
    with viz_col:
        if raw_df is not None and speed_grid is not None and len(speed_grid) > 0:
            # 可视化数据切换
            viz_mode = st.radio(
                "显示数据",
                ["油门+刹车", "仅油门", "仅刹车"],
                horizontal=True,
                help="选择3D曲面图显示的数据范围"
            )

            # 根据选择过滤数据
            if viz_mode == "仅油门":
                mask_filter = (cmd_grid >= 0)
                display_name = "油门标定曲面"
            elif viz_mode == "仅刹车":
                mask_filter = (cmd_grid <= 0)
                display_name = "刹车标定曲面"
            else:
                mask_filter = slice(None)  # 全部
                display_name = "完整标定曲面"

            # 过滤命令网格和加速度网格
            if isinstance(mask_filter, slice):
                cmd_grid_display = cmd_grid[mask_filter]
                accel_grid_display = accel_grid[mask_filter, :]
            else:
                # Boolean array indexing
                cmd_grid_display = cmd_grid[mask_filter]
                accel_grid_display = accel_grid[mask_filter, :]

            # 过滤散点数据
            if viz_mode == "仅油门":
                scatter_df = clean_df[clean_df["command"] > 0].copy()
            elif viz_mode == "仅刹车":
                scatter_df = clean_df[clean_df["command"] < 0].copy()
            else:
                scatter_df = clean_df.copy()

            scatter_df = scatter_df.sample(min(2000, len(scatter_df)), random_state=0)

            # 3D Surface Plot
            fig = go.Figure()
            fig.add_trace(go.Surface(
                z=accel_grid_display.T,
                x=cmd_grid_display,
                y=speed_grid,
                colorscale="RdBu",
                opacity=0.85,
                colorbar={"title": "Accel (m/s²)"}
            ))
            fig.add_trace(go.Scatter3d(
                x=scatter_df["command"],
                y=scatter_df["final_speed"],
                z=scatter_df["accel_aligned"],
                mode="markers",
                marker={"size": 2, "color": "black"},
                name="samples",
            ))
            fig.update_layout(
                title=display_name,
                scene={
                    "xaxis_title": "Cmd %",
                    "yaxis_title": "Speed m/s",
                    "zaxis_title": "Accel m/s²",
                    "camera": {
                        "eye": {"x": -1.5, "y": 1.5, "z": 1.5},
                        "center": {"x": 0, "y": 0, "z": 0},
                    }
                },
                height=500, margin={"l": 0, "r": 0, "t": 30, "b": 0}
            )
            st.plotly_chart(fig, use_container_width=True)

            # 底部导出按钮
            exp1, exp2 = st.columns(2)
            with exp1:
                if st.button("📥 Export"):
                    export_dir.mkdir(parents=True, exist_ok=True)
                    Exporter.save_unified_csv(speed_grid, cmd_grid, accel_grid, export_dir)
                    Exporter.save_protobuf(speed_grid, cmd_grid, accel_grid, export_dir)
                    Exporter.save_metrics(metrics, export_dir)
                    st.success(f"Exported to {export_dir}")
            with exp2:
                if st.button("📈 Steps"):
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

            # 调整指标标题间距
            st.markdown("""
            <style>
            /* 调整指标标题间距 - 在列内注入 */
            [data-testid="stMarkdownContainer"] strong {
                display: block !important;
                margin-top: 0.6rem !important;
                margin-bottom: -0.3rem !important;
                font-size: 0.9rem !important;
            }
            /* 压缩指标容器顶部间距 */
            .metric-container {
                margin-top: -0.2rem !important;
            }
            </style>
            """, unsafe_allow_html=True)

            # === 油门指标 ===
            st.markdown("**油门指标**")
            throttle_metrics = []
            t_dz = metrics.get('throttle_deadzone_pct', 0)
            rating_t_dz = get_metric_rating(t_dz, 'deadzone')
            throttle_metrics.append((f"油门死区", f"{t_dz:.0f}%", rating_t_dz, "最小指令产生响应的阈值"))

            t_r2 = metrics.get('throttle_linearity_R2', 0)
            rating_t_r2 = get_metric_rating(t_r2, 'r2')
            throttle_metrics.append((f"线性度(R²)", f"{t_r2:.3f}", rating_t_r2, "参考指标，非主要评价标准"))

            t_smooth = metrics.get('throttle_smoothness_score_100', 0)
            rating_t_smooth = get_metric_rating(t_smooth, 'smoothness')
            throttle_metrics.append((f"平滑度", f"{t_smooth:.0f}分", rating_t_smooth, "曲线平滑度评分"))

            t_vio = metrics.get('throttle_monotonic_violations', 0)
            rating_t_vio = get_metric_rating(t_vio, 'monotonicity')
            throttle_metrics.append((f"单调性", f"{t_vio}违反", rating_t_vio, "硬性要求：必须为0违反"))

            t_mae = metrics.get('throttle_residual_mae')
            if t_mae is not None:
                rating_t_mae = get_metric_rating(t_mae, 'mae')
                throttle_metrics.append((f"平均误差", f"{t_mae:.3f}", rating_t_mae, "主要评价指标：越小越好"))
                t_rmse = metrics.get('throttle_residual_rmse', 0)
                rating_t_rmse = get_metric_rating(t_rmse, 'rmse')
                throttle_metrics.append((f"均方根误差", f"{t_rmse:.3f}", rating_t_rmse, "主要评价指标：越小越好"))

            render_metrics_grid(throttle_metrics, columns=2)

            # === 刹车指标 ===
            st.markdown("**刹车指标**")
            brake_metrics = []
            b_dz = metrics.get('brake_deadzone_pct', 0)
            rating_b_dz = get_metric_rating(b_dz, 'deadzone')
            brake_metrics.append((f"刹车死区", f"{b_dz:.0f}%", rating_b_dz, "最小指令产生响应的阈值"))

            b_r2 = metrics.get('brake_linearity_R2', 0)
            rating_b_r2 = get_metric_rating(b_r2, 'r2')
            brake_metrics.append((f"线性度(R²)", f"{b_r2:.3f}", rating_b_r2, "参考指标，非主要评价标准"))

            b_smooth = metrics.get('brake_smoothness_score_100', 0)
            rating_b_smooth = get_metric_rating(b_smooth, 'smoothness')
            brake_metrics.append((f"平滑度", f"{b_smooth:.0f}分", rating_b_smooth, "曲线平滑度评分"))

            b_vio = metrics.get('brake_monotonic_violations', 0)
            rating_b_vio = get_metric_rating(b_vio, 'monotonicity')
            brake_metrics.append((f"单调性", f"{b_vio}违反", rating_b_vio, "硬性要求：必须为0违反"))

            b_mae = metrics.get('brake_residual_mae')
            if b_mae is not None:
                rating_b_mae = get_metric_rating(b_mae, 'mae')
                brake_metrics.append((f"平均误差", f"{b_mae:.3f}", rating_b_mae, "主要评价指标：越小越好"))
                b_rmse = metrics.get('brake_residual_rmse', 0)
                rating_b_rmse = get_metric_rating(b_rmse, 'rmse')
                brake_metrics.append((f"均方根误差", f"{b_rmse:.3f}", rating_b_rmse, "主要评价指标：越小越好"))

            render_metrics_grid(brake_metrics, columns=2)

            # === 下方：公共指标 ===
            st.markdown("**公共指标**")

            # 响应范围
            max_accel = metrics.get('max_throttle_accel', 0)
            max_decel = metrics.get('max_brake_decel', 0)
            st.markdown(f"""
            <div style="display:flex;gap:20px;margin-bottom:8px;">
                <div style="font-size:11px;color:#999;">响应范围:</div>
                <div style="font-size:13px;color:#64DD17;">油门 {max_accel:.2f} m/s²</div>
                <div style="font-size:13px;color:#FF3D00;">刹车 {max_decel:.2f} m/s²</div>
            </div>
            """, unsafe_allow_html=True)

            # 单调性总体状态
            mono_pass = metrics.get('monotonicity_pass', False)
            t_vio = metrics.get('throttle_monotonic_violations', 0)
            b_vio = metrics.get('brake_monotonic_violations', 0)

            mono_color = '#00C853' if mono_pass else '#FF3D00'
            mono_status = '✓ 通过' if mono_pass else '✗ 不通过'

            st.markdown(f"""
            <div style="padding:6px 8px;background:#1A2E1A;border-radius:4px;border-left:2px solid {mono_color};">
                <div style="display:flex;justify-content:space-between;align-items:center;">
                    <span style="font-size:11px;color:#CCC;">单调性总评</span>
                    <span style="font-size:12px;font-weight:bold;color:{mono_color};">{mono_status}</span>
                </div>
                <div style="margin-top:2px;font-size:10px;color:#999;">油门:{t_vio} 违反  |  刹车:{b_vio} 违反</div>
            </div>
            """, unsafe_allow_html=True)

# === 侧边栏实用功能 ===
st.sidebar.markdown("### ⚡ 快捷操作")

# 缓存管理
if st.sidebar.button("🔄 清除缓存"):
    st.cache_data.clear()
    st.success("缓存已清除")
    st.rerun()

st.sidebar.markdown("---")
st.sidebar.markdown("### Runtime")
st.sidebar.text(str(Path(__file__).resolve()))
st.sidebar.text(time.strftime("%Y-%m-%d %H:%M:%S"))
