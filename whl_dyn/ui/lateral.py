# -*- coding: utf-8 -*-
"""Streamlit workbench controls for lateral vehicle dynamics tests across Phases 1–3."""

import json
import select
import shutil
import signal
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import streamlit as st
import yaml

from whl_dyn.planning.handling import (
    generate_open_loop_identification_plan,
    generate_steady_state_circle_plan,
    generate_closed_loop_curve_plan,
)
from whl_dyn.planning.vehicle_dynamics import generate_lateral_frequency_plan
from whl_dyn.processing.lateral_dynamics import write_lateral_frequency_report


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def _load_plan_cases(plan_path: Path) -> List[Dict]:
    """Load case dictionary list from YAML plan."""
    if not plan_path.exists():
        return []
    try:
        with open(plan_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
            if isinstance(data, list):
                return data
            elif isinstance(data, dict):
                return [data]
    except Exception:
        pass
    return []


def _parse_lateral_case_summary(case: Dict) -> Dict:
    name = case.get("case_name", "unknown")
    test_type = case.get("test_type", "")
    phase = case.get("phase", "")
    mode = case.get("mode", "")
    duration = float(case.get("duration_sec", 0.0))
    speed_gate = case.get("speed_gate", {})
    target_speed = float(speed_gate.get("target_mps", 0.0)) if isinstance(speed_gate, dict) else 0.0
    safety_limits = case.get("safety_limits", {})
    max_steer = float(safety_limits.get("max_abs_steering", 0.0)) if isinstance(safety_limits, dict) else 0.0
    return {
        "case_name": name,
        "test_type": test_type or mode,
        "phase": phase,
        "duration_sec": duration,
        "target_speed_mps": target_speed,
        "max_steer": max_steer,
    }


def _build_lateral_plan_df(cases: List[Dict]) -> pd.DataFrame:
    columns = ["case_name", "test_type", "phase", "duration_sec", "target_speed_mps", "max_steer"]
    if not cases:
        return pd.DataFrame(columns=columns)
    return pd.DataFrame([_parse_lateral_case_summary(c) for c in cases], columns=columns)



def _find_case_runs(output_dir: Path, case_name: str) -> List[Path]:
    """Find directories in output_dir matching case_name that contain samples.csv."""
    if not output_dir.exists():
        return []
    matches = []
    for d in output_dir.iterdir():
        if d.is_dir() and (d.name == case_name or d.name.startswith(f"{case_name}_")):
            if (d / "samples.csv").exists():
                matches.append(d)
    return sorted(matches, key=lambda p: p.stat().st_mtime)


def _get_case_status_info(case_name: str, output_dir: Path, running_case: Optional[str], last_rc: Optional[int]) -> Dict:
    """Calculate execution status and metrics for a specific case."""
    if running_case == case_name:
        return {
            "status_code": "running",
            "status_label": "运行中",
            "status_icon": "⏳",
            "rows": 0,
            "latest_run": None,
        }

    runs = _find_case_runs(output_dir, case_name)
    if runs:
        latest = runs[-1]
        samples_file = latest / "samples.csv"
        status = {}
        status_file = latest / "status.json"
        if status_file.exists():
            try:
                with open(status_file, "r", encoding="utf-8") as f:
                    status = json.load(f)
            except (OSError, ValueError):
                status = {}
        try:
            row_count = sum(1 for _ in open(samples_file, "rb")) - 1
        except Exception:
            row_count = 0

        abort_reason = status.get("abort_reason")
        if status.get("completed") is False:
            return {
                "status_code": "error",
                "status_label": "中断",
                "status_icon": "✗",
                "rows": max(row_count, 0),
                "latest_run": latest,
                "abort_reason": abort_reason or "unknown interruption",
            }
        if row_count > 10:
            return {
                "status_code": "completed",
                "status_label": "采集完成",
                "status_icon": "✓",
                "rows": row_count,
                "latest_run": latest,
                "abort_reason": None,
            }
        else:
            return {
                "status_code": "warning",
                "status_label": "数据过少",
                "status_icon": "⚠",
                "rows": row_count,
                "latest_run": latest,
                "abort_reason": abort_reason,
            }

    if st.session_state.get("lateral_last_case") == case_name and last_rc not in (None, 0):
        return {
            "status_code": "error",
            "status_label": f"异常({last_rc})",
            "status_icon": "✗",
            "rows": 0,
            "latest_run": None,
            "abort_reason": None,
        }

    return {
        "status_code": "pending",
        "status_label": "未完成",
        "status_icon": "⚪",
        "rows": 0,
        "latest_run": None,
        "abort_reason": None,
    }


def _save_case_temp_plan(case: Dict, case_name: str, runtime_dir: Path) -> Path:
    """Generate a single-case plan YAML for execution."""
    runtime_dir.mkdir(parents=True, exist_ok=True)
    temp_plan = runtime_dir / f"lat_plan_{case_name}.yaml"
    with open(temp_plan, "w", encoding="utf-8") as f:
        yaml.safe_dump([case], f, allow_unicode=True)
    return temp_plan


def _start_collection(plan_path: Path, signal_config: str, output_dir: str, case_name: Optional[str] = None):
    """Start lateral dynamics collection process in active excitation mode."""
    command = [
        sys.executable,
        "-m",
        "whl_dyn.cli",
        "collect-lateral",
        "--plan",
        str(plan_path),
        "--signal-config",
        signal_config,
        "--output-dir",
        output_dir,
        "--execute",
        "--arm",
    ]
    proc = subprocess.Popen(
        command,
        cwd=str(PROJECT_ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    st.session_state["lateral_process"] = proc
    st.session_state["lateral_logs"] = ["$ " + " ".join(command)]
    st.session_state["lateral_last_returncode"] = None
    st.session_state["lateral_running_case"] = case_name
    st.session_state["lateral_last_case"] = case_name
    st.session_state["lateral_active_plan"] = str(plan_path)
    st.session_state["lateral_output_dir"] = output_dir


def _stop_collection():
    """Safely terminate lateral collection subprocess."""
    proc = st.session_state.get("lateral_process")
    if proc and proc.poll() is None:
        try:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        except Exception:
            proc.kill()
    if proc:
        st.session_state["lateral_last_returncode"] = proc.returncode
    st.session_state["lateral_process"] = None
    st.session_state["lateral_running_case"] = None


def _drain_lateral_logs():
    """Read available stdout lines from background collection process."""
    proc = st.session_state.get("lateral_process")
    if proc is None:
        return
    if proc.stdout is not None:
        while True:
            ready, _, _ = select.select([proc.stdout], [], [], 0)
            if not ready:
                break
            line = proc.stdout.readline()
            if not line:
                break
            st.session_state.setdefault("lateral_logs", []).append(line.rstrip())

    if proc.poll() is not None:
        st.session_state["lateral_last_returncode"] = proc.returncode
        st.session_state["lateral_running_case"] = None
        try:
            rem = proc.stdout.read()
            if rem:
                for l in rem.splitlines():
                    if l.strip():
                        st.session_state.setdefault("lateral_logs", []).append(l.rstrip())
        except Exception:
            pass


def _float_list(text_str: str) -> tuple:
    return tuple(float(x.strip()) for x in text_str.split(",") if x.strip())


def render_lateral_plan(runtime_dir: Path):
    """Render Plan Generation tab for lateral vehicle dynamics (Phases 1–3)."""
    runtime_path = Path(runtime_dir)
    runtime_path.mkdir(parents=True, exist_ok=True)

    st.markdown("##### 📋 横向动力学测试计划生成 (Phase 1 ~ Phase 3)")

    test_type = st.selectbox(
        "测试阶段与工况 (Test Phase & Case)",
        [
            "phase1_step_ramp",
            "phase1_pulse",
            "phase1_sine",
            "phase1_chirp",
            "phase1_prbs",
            "phase2_circles",
            "phase3_closed_loop",
        ],
        format_func=lambda k: {
            "phase1_step_ramp": "Phase 1.1: 开环阶跃与慢斜坡 (Step & Slow Ramp)",
            "phase1_pulse": "Phase 1.1: 开环脉冲 (Pulse)",
            "phase1_sine": "Phase 1.1: 开环单正弦 (Single Sine)",
            "phase1_chirp": "Phase 1.2: 开环正弦扫频 (Chirp Frequency Sweep)",
            "phase1_prbs": "Phase 1.3: 开环伪随机序列 (PRBS Excitation)",
            "phase2_circles": "Phase 2.1: 定转角圆周稳态 (Fixed-Steering Steady Circles)",
            "phase3_closed_loop": "Phase 3.1: 闭环缓和曲线跟踪 (Clothoid Curve Tracking)",
        }[k],
        key="lat_plan_test_type",
    )

    # Suggested default path based on test type
    default_filename_map = {
        "phase1_step_ramp": "open_loop_identification.yaml",
        "phase1_pulse": "lateral_pulse.yaml",
        "phase1_sine": "lateral_sine.yaml",
        "phase1_chirp": "lateral_chirp.yaml",
        "phase1_prbs": "lateral_prbs.yaml",
        "phase2_circles": "steady_state_circles.yaml",
        "phase3_closed_loop": "closed_loop_curve.yaml",
    }
    suggested_path = str(PROJECT_ROOT / "plans" / default_filename_map[test_type])

    left_col, right_col = st.columns(2)

    # 1. Phase 1.1: Step & Slow Ramp
    if test_type == "phase1_step_ramp":
        with left_col:
            p1_amp = st.number_input("转向转角幅值 (命令单位)", min_value=0.1, value=2.0, key="p1_ol_amp")
            p1_ramp = st.number_input("慢斜坡速率 (命令单位/s)", min_value=0.1, value=1.0, key="p1_ol_ramp")
            p1_hold = st.number_input("阶跃保持时间 (s)", min_value=1.0, value=8.0, key="p1_ol_hold")
        with right_col:
            p1_speed = st.number_input("目标车速 (m/s)", min_value=0.5, value=2.0, key="p1_ol_speed")
            p1_tol = st.number_input("车速容差 (m/s)", min_value=0.01, value=0.15, key="p1_ol_tol")
            p1_max_steer = st.number_input("最大转角限制 (命令单位)", min_value=1.0, value=20.0, key="p1_ol_max_steer")
            p1_max_rate = st.number_input("最大转角速率 (命令单位/s)", min_value=1.0, value=30.0, key="p1_ol_max_rate")

    # 2. Phase 1.2: Chirp
    elif test_type in ("phase1_pulse", "phase1_sine", "phase1_chirp"):
        with left_col:
            p2_amp = st.number_input("扫频转角幅值 (命令单位)", min_value=0.01, value=2.0, key="p2_ch_amp")
            if test_type == "phase1_pulse":
                p2_pulse_dur = st.number_input("脉冲宽度 (s)", min_value=0.01, value=1.0, key="p2_pulse_dur")
            elif test_type == "phase1_sine":
                p2_sine_freq = st.number_input("正弦频率 (Hz)", min_value=0.01, value=0.5, key="p2_sine_freq")
            f1, f2 = st.columns(2)
            with f1:
                p2_fstart = st.number_input("起始频率 (Hz)", min_value=0.01, value=0.05, key="p2_ch_fstart")
            with f2:
                p2_fend = st.number_input("终止频率 (Hz)", min_value=0.02, value=2.0, key="p2_ch_fend")
            d1, d2 = st.columns(2)
            with d1:
                p2_dur = st.number_input("测试时长 (s)", min_value=1.0, value=120.0, key="p2_ch_dur")
            with d2:
                p2_fs = st.number_input("采样频率 (Hz)", min_value=10.0, value=100.0, key="p2_ch_fs")
        with right_col:
            s1, s2 = st.columns(2)
            with s1:
                p2_smin = st.number_input("速度下限 (m/s)", min_value=0.0, value=0.0, key="p2_ch_smin")
                p2_starget = st.number_input("目标速度 (m/s)", min_value=0.0, value=2.0, key="p2_ch_starget")
                p2_sstable = st.number_input("速度稳定时间 (s)", min_value=0.0, value=3.0, key="p2_ch_sstable")
                p2_max_steer = st.number_input("最大轮角 (命令单位)", min_value=0.01, value=20.0, key="p2_ch_max_steer")
            with s2:
                p2_smax = st.number_input("速度上限 (m/s)", min_value=0.01, value=3.0, key="p2_ch_smax")
                p2_stolerance = st.number_input("速度容差 (m/s)", min_value=0.0, value=0.15, key="p2_ch_stolerance")
                p2_swait = st.number_input("速度超时 (s)", min_value=1.0, value=30.0, key="p2_ch_swait")
                p2_max_rate = st.number_input("最大转角速率 (命令/s)", min_value=0.01, value=30.0, key="p2_ch_max_rate")

    # 3. Phase 1.3: PRBS
    elif test_type == "phase1_prbs":
        with left_col:
            p2_prbs_amp = st.number_input("PRBS 转角幅值 (命令单位)", min_value=0.01, value=2.0, key="p2_prbs_amp")
            p2_prbs_bit = st.number_input("Bit Duration (s)", min_value=0.01, value=0.25, key="p2_prbs_bit")
            p2_prbs_seed = st.number_input("PRBS Seed", min_value=0, value=7, step=1, key="p2_prbs_seed")
            d1, d2 = st.columns(2)
            with d1:
                p2_prbs_dur = st.number_input("测试时长 (s)", min_value=1.0, value=120.0, key="p2_prbs_dur")
            with d2:
                p2_prbs_fs = st.number_input("采样频率 (Hz)", min_value=10.0, value=100.0, key="p2_prbs_fs")
        with right_col:
            s1, s2 = st.columns(2)
            with s1:
                p2_prbs_smin = st.number_input("速度下限 (m/s)", min_value=0.0, value=0.0, key="p2_prbs_smin")
                p2_prbs_starget = st.number_input("目标速度 (m/s)", min_value=0.0, value=2.0, key="p2_prbs_starget")
                p2_prbs_sstable = st.number_input("速度稳定时间 (s)", min_value=0.0, value=3.0, key="p2_prbs_sstable")
                p2_prbs_max_steer = st.number_input("最大轮角 (命令单位)", min_value=0.01, value=20.0, key="p2_prbs_max_steer")
            with s2:
                p2_prbs_smax = st.number_input("速度上限 (m/s)", min_value=0.01, value=3.0, key="p2_prbs_smax")
                p2_prbs_stolerance = st.number_input("速度容差 (m/s)", min_value=0.0, value=0.15, key="p2_prbs_stolerance")
                p2_prbs_swait = st.number_input("速度超时 (s)", min_value=1.0, value=30.0, key="p2_prbs_swait")
                p2_max_rate = st.number_input("最大转角速率 (命令/s)", min_value=0.01, value=30.0, key="p2_prbs_max_rate")

    # 4. Phase 2.1: Steady-State Circles
    elif test_type == "phase2_circles":
        with left_col:
            p1_c_steer = st.text_input("转向角矩阵 (逗号分隔)", value="1.0, 2.0, 3.0", key="p1_c_steer")
            p1_c_speeds = st.text_input("车速矩阵 (m/s, 逗号分隔)", value="1.0, 2.0, 3.0", key="p1_c_speeds")
            p1_c_dur = st.number_input("稳态持续时间 (s)", min_value=1.0, value=20.0, key="p1_c_dur")
        with right_col:
            p1_c_ramp = st.number_input("转向进入速率 (命令单位/s)", min_value=0.1, value=0.5, key="p1_c_ramp")
            p1_c_accel = st.number_input("最大侧向加速度 (m/s²)", min_value=0.5, value=1.5, key="p1_c_accel")
            p1_c_rep = st.number_input("每工况重复次数", min_value=1, value=3, key="p1_c_rep")

    # 5. Phase 3.1: Closed Loop Curve
    else:
        with left_col:
            p3_radius = st.number_input("圆弧半径 R (m)", min_value=5.0, value=50.0, key="p3_radius")
            p3_speed = st.number_input("跟踪车速 (m/s)", min_value=0.5, value=2.0, key="p3_speed")
            p3_dir = st.selectbox("转弯方向", ["left", "right"], format_func=lambda d: "左转 (Left)" if d == "left" else "右转 (Right)", key="p3_dir")
        with right_col:
            p3_straight = st.number_input("直道引入段长度 (m)", min_value=0.0, value=20.0, key="p3_straight")
            p3_entry = st.number_input("缓和曲线段长度 (m)", min_value=1.0, value=15.0, key="p3_entry")
            p3_arc = st.number_input("圆弧转角 (rad)", min_value=0.1, value=1.57, key="p3_arc")

    st.markdown("**计划保存路径**")
    plan_path_text = st.text_input(
        "计划文件路径",
        value=suggested_path,
        key="lat_plan_path_input",
        help="支持自定义 YAML 计划保存路径，避免相互覆盖",
    )
    plan_path = Path(plan_path_text) if Path(plan_path_text).is_absolute() else PROJECT_ROOT / plan_path_text

    if st.button("生成测试计划", type="primary", key="btn_gen_lat_plan"):
        try:
            plan_path.parent.mkdir(parents=True, exist_ok=True)
            if test_type == "phase1_step_ramp":
                generate_open_loop_identification_plan(
                    output=str(plan_path),
                    target_speed_mps=float(p1_speed),
                    speed_tolerance_mps=float(p1_tol),
                    amplitude=float(p1_amp),
                    ramp_rate=float(p1_ramp),
                    step_hold_sec=float(p1_hold),
                    max_steering=float(p1_max_steer),
                    max_steering_rate=float(p1_max_rate),
                )
            elif test_type in ("phase1_pulse", "phase1_sine", "phase1_chirp"):
                generate_lateral_frequency_plan(
                    output=str(plan_path),
                    mode={"phase1_pulse": "pulse", "phase1_sine": "single_sine"}.get(test_type, "chirp"),
                    duration_sec=float(p2_dur),
                    sampling_rate_hz=float(p2_fs),
                    steering_amplitude=float(p2_amp),
                    frequency_start_hz=float(p2_fstart),
                    frequency_end_hz=float(p2_fend),
                    pulse_duration_sec=float(p2_pulse_dur) if test_type == "phase1_pulse" else 1.0,
                    sine_frequency_hz=float(p2_sine_freq) if test_type == "phase1_sine" else 0.5,
                    speed_min_mps=float(p2_smin),
                    speed_max_mps=float(p2_smax),
                    target_speed_mps=float(p2_starget),
                    speed_tolerance_mps=float(p2_stolerance),
                    stable_speed_sec=float(p2_sstable),
                    max_steering=float(p2_max_steer),
                    max_steering_rate=float(p2_max_rate),
                    max_speed_wait_sec=float(p2_swait),
                )
            elif test_type == "phase1_prbs":
                generate_lateral_frequency_plan(
                    output=str(plan_path),
                    mode="prbs",
                    duration_sec=float(p2_prbs_dur),
                    sampling_rate_hz=float(p2_prbs_fs),
                    steering_amplitude=float(p2_prbs_amp),
                    bit_duration_sec=float(p2_prbs_bit),
                    prbs_seed=int(p2_prbs_seed),
                    speed_min_mps=float(p2_prbs_smin),
                    speed_max_mps=float(p2_prbs_smax),
                    target_speed_mps=float(p2_prbs_starget),
                    speed_tolerance_mps=float(p2_prbs_stolerance),
                    stable_speed_sec=float(p2_prbs_sstable),
                    max_steering=float(p2_prbs_max_steer),
                    max_steering_rate=float(p2_prbs_max_rate),
                    max_speed_wait_sec=float(p2_prbs_swait),
                )
            elif test_type == "phase2_circles":
                generate_steady_state_circle_plan(
                    output=str(plan_path),
                    steering_commands=_float_list(p1_c_steer),
                    speed_targets_mps=_float_list(p1_c_speeds),
                    steady_duration_sec=float(p1_c_dur),
                    steering_ramp_rate=float(p1_c_ramp),
                    max_lateral_accel_mps2=float(p1_c_accel),
                    repeats=int(p1_c_rep),
                )
            else:
                generate_closed_loop_curve_plan(
                    output=str(plan_path),
                    radius_m=float(p3_radius),
                    speed_mps=float(p3_speed),
                    straight_entry_length_m=float(p3_straight),
                    entry_length_m=float(p3_entry),
                    arc_angle_rad=float(p3_arc),
                    direction=p3_dir,
                )
            st.success(f"测试计划生成成功：{plan_path}")
        except Exception as error:
            st.error(f"生成失败: {error}")

    if plan_path.exists():
        st.markdown("---")
        st.markdown(f"**计划内容预览 ({plan_path.name})**")
        cases = _load_plan_cases(plan_path)
        lat_plan_df = _build_lateral_plan_df(cases)

        view_mode = st.radio(
            "视图模式", ["表格", "源码"], horizontal=True, label_visibility="collapsed", key="lat_plan_view_mode"
        )
        if view_mode == "表格":
            st.dataframe(lat_plan_df, use_container_width=True)
        else:
            with open(plan_path, "r", encoding="utf-8") as f:
                plan_yaml = f.read()
            edited_yaml = st.text_area(
                "编辑 YAML 计划", value=plan_yaml, height=350, key=f"lat_plan_yaml_edit_{plan_path.name}"
            )
            if st.button("💾 保存计划修改", key="btn_save_lat_yaml_plan"):
                try:
                    yaml.safe_load(edited_yaml)
                    with open(plan_path, "w", encoding="utf-8") as f:
                        f.write(edited_yaml)
                    st.success(f"计划修改已保存: {plan_path}")
                    st.rerun()
                except Exception as e:
                    st.error(f"YAML 格式错误: {e}")


def render_lateral_collect(runtime_dir: Path):
    """Render Data Collection tab for lateral vehicle dynamics."""
    runtime_path = Path(runtime_dir)
    runtime_path.mkdir(parents=True, exist_ok=True)

    _drain_lateral_logs()
    process = st.session_state.get("lateral_process")
    running = process is not None and process.poll() is None
    running_case = st.session_state.get("lateral_running_case") if running else None
    last_rc = st.session_state.get("lateral_last_returncode")

    # Available plans in plans/
    plans_dir = PROJECT_ROOT / "plans"
    preset_plans = []
    if plans_dir.exists():
        preset_plans = sorted([str(p.relative_to(PROJECT_ROOT)) for p in plans_dir.glob("*.yaml")])
    default_plan = "plans/open_loop_identification.yaml" if "plans/open_loop_identification.yaml" in preset_plans else (preset_plans[0] if preset_plans else "plans/lateral_chirp.yaml")

    top1, top2 = st.columns(2)
    with top1:
        plan_file_str = st.selectbox(
            "测试计划文件",
            preset_plans if preset_plans else [default_plan],
            index=preset_plans.index(default_plan) if default_plan in preset_plans else 0,
            key="lat_collect_plan_select",
        )
        plan_path = Path(plan_file_str) if Path(plan_file_str).is_absolute() else PROJECT_ROOT / plan_file_str
    with top2:
        out_dir_text = st.text_input(
            "采集输出目录",
            value="vehicle_dynamics_runs",
            key="lat_collect_out_dir",
        )
        output_dir = Path(out_dir_text) if Path(out_dir_text).is_absolute() else PROJECT_ROOT / out_dir_text

    signal_config = "vehicle_signals.yaml"

    # Load plan cases
    cases = _load_plan_cases(plan_path)

    # 左右分栏布局
    left_col, right_col = st.columns([1, 2])

    with left_col:
        st.markdown("**用例选择**")
        selected_case_name = None
        selected_case_obj = None

        if cases:
            case_names = [c.get("case_name", f"case_{idx}") for idx, c in enumerate(cases)]
            case_lookup = {c.get("case_name", f"case_{idx}"): c for idx, c in enumerate(cases)}

            case_idx = st.session_state.get("lat_selected_case_idx", 0)
            case_idx = min(max(0, case_idx), len(case_names) - 1)

            nav1, sel, nav2 = st.columns([1, 4, 1])
            with nav1:
                if st.button("◀", disabled=case_idx <= 0, key="btn_lat_case_prev"):
                    st.session_state["lat_selected_case_idx"] = case_idx - 1
                    st.rerun()
            with sel:
                selected_case_name = st.selectbox(
                    "选择用例",
                    case_names,
                    index=case_idx,
                    label_visibility="collapsed",
                    key="lat_case_selector",
                )
                case_idx = case_names.index(selected_case_name)
                st.session_state["lat_selected_case_idx"] = case_idx
                selected_case_obj = case_lookup[selected_case_name]
            with nav2:
                if st.button("▶", disabled=case_idx >= len(case_names) - 1, key="btn_lat_case_next"):
                    st.session_state["lat_selected_case_idx"] = case_idx + 1
                    st.rerun()

            st.markdown("**执行进度**")
            # Build list rows with zero markdown indentation
            style_block = """<style>
.progress-list { display: flex; flex-direction: column; gap: 4px; }
.progress-row { display: grid; grid-template-columns: 1fr 75px 55px; gap: 6px; padding: 6px 8px; border-radius: 4px; font-size: 0.85rem; align-items: center; }
.progress-row > * { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.row-pending { background: #f5f5f5; color: #666; }
.row-running { background: #fff3cd; color: #856404; }
.row-completed { background: #d4edda; color: #155724; }
.row-stopped { background: #f8d7da; color: #721c24; }
.row-error { background: #f8d7da; color: #721c24; }
.row-warning { background: #fff3cd; color: #856404; }
.row-selected { border: 2px solid #007bff !important; font-weight: bold; }
</style>
<div class="progress-list">"""

            rows_html = []
            for idx, cname in enumerate(case_names):
                st_info = _get_case_status_info(cname, output_dir, running_case, last_rc)
                is_sel = " row-selected" if cname == selected_case_name else ""
                rows_text = f"{st_info['rows']}行" if st_info['rows'] > 0 else "-"
                case_name_short = cname[:22] + "..." if len(cname) > 22 else cname
                row_code = st_info["status_code"]
                row_html = (
                    f'<div class="progress-row row-{row_code}{is_sel}">'
                    f'<span title="{cname}">{case_name_short}</span>'
                    f'<span style="text-align:center;">{st_info["status_icon"]} {st_info["status_label"]}</span>'
                    f'<span style="text-align:right;">{rows_text}</span>'
                    f'</div>'
                )
                rows_html.append(row_html)

            full_html = style_block + "".join(rows_html) + "</div>"
            st.markdown(full_html, unsafe_allow_html=True)
        else:
            st.info("当前计划文件中无可用用例，请先在‘生成计划’中创建计划。")

    with right_col:
        st.markdown("**当前状态**")
        curr_info = _get_case_status_info(selected_case_name, output_dir, running_case, last_rc) if selected_case_name else None
        latest_run = curr_info["latest_run"] if curr_info else None
        latest_dir_name = latest_run.name if latest_run else "N/A"
        row_count = curr_info["rows"] if curr_info else 0

        c1, c2, c3 = st.columns(3)
        with c1:
            st.caption("数据行数")
            st.text(str(row_count))
        with c2:
            st.caption("采集状态")
            if curr_info:
                st.text(f"{curr_info['status_icon']} {curr_info['status_label']}")
            else:
                st.text("就绪")
        with c3:
            st.caption("模式")
            st.text("主动激励 (Active)")

        st.caption("最新数据目录")
        st.text(latest_dir_name)
        if curr_info and curr_info.get("abort_reason"):
            st.error(f"中断原因：{curr_info['abort_reason']}")

        # Toolbar Actions (▶ 开始 | ⏹ 停止 | ↺ 重试 | 🗑 清除)
        btn_cols = st.columns(4)
        with btn_cols[0]:
            start_disabled = running or not selected_case_obj
            if st.button("▶ 开始", disabled=start_disabled, type="primary", key="btn_lat_act_start"):
                single_plan = _save_case_temp_plan(selected_case_obj, selected_case_name, runtime_path)
                _start_collection(single_plan, signal_config, str(output_dir), case_name=selected_case_name)
                st.rerun()

        with btn_cols[1]:
            if st.button("⏹ 停止", disabled=not running, key="btn_lat_act_stop"):
                _stop_collection()
                st.rerun()

        with btn_cols[2]:
            retry_disabled = running or not selected_case_obj
            if st.button("↺ 重试", disabled=retry_disabled, key="btn_lat_act_retry"):
                # Delete existing latest run directory for this case
                if latest_run and latest_run.exists():
                    try:
                        shutil.rmtree(latest_run)
                    except Exception:
                        pass
                single_plan = _save_case_temp_plan(selected_case_obj, selected_case_name, runtime_path)
                _start_collection(single_plan, signal_config, str(output_dir), case_name=selected_case_name)
                st.rerun()

        with btn_cols[3]:
            delete_disabled = running or not latest_run
            if st.button("🗑 清除", disabled=delete_disabled, key="btn_lat_act_delete"):
                # Delete all matching runs for this case
                all_runs = _find_case_runs(output_dir, selected_case_name)
                for r in all_runs:
                    if r.exists():
                        try:
                            shutil.rmtree(r)
                        except Exception:
                            pass
                st.rerun()

        if last_rc not in (None, 0):
            reason = ""
            if latest_run:
                status_file = latest_run / "status.json"
                if status_file.exists():
                    try:
                        with open(status_file, "r", encoding="utf-8") as f:
                            reason = json.load(f).get("abort_reason") or ""
                    except (OSError, ValueError):
                        pass
            suffix = f"：{reason}" if reason else ""
            st.error(f"⚠️ 采集进程已退出 (返回码 {last_rc}){suffix}")

        st.markdown("**实时日志**")
        logs = st.session_state.get("lateral_logs", [])
        log_text = "\n".join(logs[-150:]) if logs else "暂无运行日志。"
        st.text_area("实时日志", value=log_text, height=220, label_visibility="collapsed", key="lat_log_area")


def _render_bode_plotly(bode_df: pd.DataFrame, title: str):
    """Render interactive 2-subplot Bode figure."""
    fig = make_subplots(
        rows=2,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.08,
        subplot_titles=(f"{title} - 增益响应 (Gain)", f"{title} - 相位响应 (Phase)"),
    )
    fig.add_trace(
        go.Scatter(
            x=bode_df["frequency_hz"],
            y=bode_df["magnitude_db"],
            mode="lines+markers",
            name="Magnitude (dB)",
            line=dict(color="#1f77b4", width=2),
        ),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=bode_df["frequency_hz"],
            y=bode_df["phase_deg"],
            mode="lines+markers",
            name="Phase (deg)",
            line=dict(color="#ff7f0e", width=2),
        ),
        row=2,
        col=1,
    )
    fig.update_xaxes(title_text="Frequency (Hz)", type="log", row=2, col=1)
    fig.update_xaxes(type="log", row=1, col=1)
    fig.update_yaxes(title_text="Gain (dB)", row=1, col=1)
    fig.update_yaxes(title_text="Phase (deg)", row=2, col=1)
    fig.update_layout(height=420, margin=dict(l=40, r=20, t=40, b=30))
    st.plotly_chart(fig, use_container_width=True)


def render_lateral_analysis(runtime_dir: Path):
    """Render Analysis tab for lateral vehicle dynamics."""
    st.markdown("##### 📊 横向动力学频域分析 (Bode 响应)")
    st.caption("计算转向反馈 -> 横摆角速度 (Yaw Rate) 与 横向加速度 (Lateral Accel) 的频率响应函数。")

    output_base = PROJECT_ROOT / "vehicle_dynamics_runs"
    available_runs = []
    if output_base.exists():
        available_runs = sorted(
            [str(d) for d in output_base.iterdir() if d.is_dir() and (d / "samples.csv").exists()],
            key=lambda p: Path(p).stat().st_mtime,
            reverse=True,
        )

    col1, col2 = st.columns([3, 1])
    with col1:
        if available_runs:
            run_directory = st.selectbox(
                "选择已完成的运行目录",
                available_runs,
                format_func=lambda p: Path(p).name,
                key="lat_analysis_run_select",
            )
        else:
            run_directory = st.text_input(
                "已完成的运行目录",
                value="",
                placeholder="例如: /path/to/vehicle_dynamics_runs/run_20260820_...",
                key="lat_analysis_run_text",
            )
    with col2:
        st.write("")  # spacing
        analyze_btn = st.button("生成 Bode 指标与图", type="primary", disabled=not bool(run_directory), key="btn_lat_analyze")

    if run_directory:
        run_path = Path(run_directory)
        analysis_path = run_path / "analysis"

        if analyze_btn:
            try:
                out_dir, summary = write_lateral_frequency_report(run_directory)
                st.success(f"分析完成！报告已保存至: {out_dir}")
            except (OSError, ValueError, FileNotFoundError) as error:
                st.error(f"分析失败: {error}")

        if analysis_path.exists():
            metrics_json = analysis_path / "metrics.json"
            if metrics_json.exists():
                with open(metrics_json, "r", encoding="utf-8") as f:
                    summary = json.load(f)

                st.markdown("---")
                st.markdown("#### 核心频域响应指标")

                responses = summary.get("responses", {})
                r_col1, r_col2 = st.columns(2)

                if "yaw_rate" in responses:
                    yr = responses["yaw_rate"]
                    with r_col1:
                        st.markdown("**🧭 转向 -> 横摆角速度 (Yaw Rate)**")
                        m1, m2, m3 = st.columns(3)
                        m1.metric("带宽 (Bandwidth)", f"{yr.get('bandwidth_hz', 0.0):.2f} Hz")
                        m2.metric("共振峰值", f"{yr.get('resonance_peak_db', 0.0):.1f} dB")
                        m3.metric("估计延迟", f"{yr.get('estimated_delay_sec', 0.0)*1000:.1f} ms")

                if "lateral_acceleration" in responses:
                    la = responses["lateral_acceleration"]
                    with r_col2:
                        st.markdown("**📈 转向 -> 横向加速度 (Lateral Accel)**")
                        m1, m2, m3 = st.columns(3)
                        m1.metric("带宽 (Bandwidth)", f"{la.get('bandwidth_hz', 0.0):.2f} Hz")
                        m2.metric("共振峰值", f"{la.get('resonance_peak_db', 0.0):.1f} dB")
                        m3.metric("估计延迟", f"{la.get('estimated_delay_sec', 0.0)*1000:.1f} ms")

                st.markdown("---")
                # Tabs for Bode plots
                bode_tab1, bode_tab2 = st.tabs(["横摆角速度响应 (Yaw Rate)", "横向加速度响应 (Lateral Accel)"])
                with bode_tab1:
                    yr_csv = analysis_path / "bode_yaw_rate.csv"
                    if yr_csv.exists():
                        df_yr = pd.read_csv(yr_csv)
                        _render_bode_plotly(df_yr, "转向 -> 横摆角速度 (Yaw Rate)")
                        with st.expander("查看数值表格"):
                            st.dataframe(df_yr, use_container_width=True)

                with bode_tab2:
                    la_csv = analysis_path / "bode_lateral_acceleration.csv"
                    if la_csv.exists():
                        df_la = pd.read_csv(la_csv)
                        _render_bode_plotly(df_la, "转向 -> 横向加速度 (Lateral Accel)")
                        with st.expander("查看数值表格"):
                            st.dataframe(df_la, use_container_width=True)


def render_lateral_dynamics(runtime_dir: Path, step: Optional[str] = None):
    """Render lateral dynamics tab(s)."""
    if step == "plan":
        render_lateral_plan(runtime_dir)
    elif step == "collect":
        render_lateral_collect(runtime_dir)
    elif step == "analysis":
        render_lateral_analysis(runtime_dir)
    else:
        render_lateral_plan(runtime_dir)
        render_lateral_collect(runtime_dir)
        render_lateral_analysis(runtime_dir)
