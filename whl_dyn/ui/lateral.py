"""Streamlit controls for collection-first lateral dynamics experiments."""

import signal
import subprocess
import sys
from pathlib import Path

import streamlit as st

from whl_dyn.planning.vehicle_dynamics import generate_lateral_frequency_plan
from whl_dyn.processing.lateral_dynamics import write_lateral_frequency_report


def _start_collection(plan_path, signal_config, output_dir, execute):
    command = [
        sys.executable, "-m", "whl_dyn.cli", "collect-lateral",
        "--plan", str(plan_path),
        "--signal-config", signal_config,
        "--output-dir", output_dir,
    ]
    if execute:
        command.extend(("--execute", "--arm"))
    st.session_state["lateral_process"] = subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def render_lateral_dynamics(runtime_dir):
    """Render plan, collection and analysis controls in one focused tab."""

    runtime_path = Path(runtime_dir)
    runtime_path.mkdir(parents=True, exist_ok=True)
    st.subheader("横向动力学：采集 -> 分析")
    st.caption(
        "固定输出：方向反馈 -> 横摆角速度、横向加速度。先使用记录模式核对信号，再明确解锁主动激励。"
    )

    with st.form("lateral_plan_form"):
        left, right = st.columns(2)
        with left:
            mode = st.selectbox("测试项", ("chirp", "prbs"), format_func=lambda value: {
                "chirp": "2.2 扫频 / Chirp",
                "prbs": "2.3 PRBS",
            }[value])
            amplitude = st.number_input(
                "方向转角幅值（命令单位）", min_value=0.01, value=2.0,
                help="通过 vehicle signal mapping 的 control_steering_scale 转换为车辆实际命令单位。")
            bit_duration = st.number_input(
                "PRBS Bit Duration (s)", min_value=0.01, value=0.25,
                help="双极性 PRBS 的最坏命令速率为 2 × 幅值 / Bit Duration。")
            prbs_seed = st.number_input("PRBS Seed", min_value=0, value=7, step=1)
            frequency_start = st.number_input("起始频率 (Hz)", min_value=0.01, value=0.05)
            frequency_end = st.number_input("终止频率 (Hz)", min_value=0.02, value=2.0)
        with right:
            duration = st.number_input("测试时长 (s)", min_value=1.0, value=120.0)
            sample_rate = st.number_input("采样频率 (Hz)", min_value=10.0, value=100.0)
            speed_min = st.number_input("速度门下限 (m/s)", min_value=0.0, value=0.0)
            speed_max = st.number_input("速度门上限 (m/s)", min_value=0.01, value=3.0)
            target_speed = st.number_input("目标速度 (m/s)", min_value=0.0, value=2.0)
            speed_tolerance = st.number_input(
                "目标速度容差 (m/s)", min_value=0.0, value=0.15)
            stable_duration = st.number_input("速度稳定时间 (s)", min_value=0.0, value=3.0)
            max_speed_wait = st.number_input(
                "达到目标速度超时 (s)", min_value=1.0, value=30.0)
            max_steering = st.number_input("最大轮角（命令单位）", min_value=0.01, value=20.0)
            max_rate = st.number_input(
                "最大轮角速率（命令单位/s）", min_value=0.01, value=30.0)
        create_plan = st.form_submit_button("生成横向测试计划")

    plan_path = runtime_path / "lateral_frequency_plan.yaml"
    if create_plan:
        try:
            generate_lateral_frequency_plan(
                output=str(plan_path), mode=mode, duration_sec=duration,
                sampling_rate_hz=sample_rate, steering_amplitude=amplitude,
                frequency_start_hz=frequency_start, frequency_end_hz=frequency_end,
                bit_duration_sec=bit_duration, prbs_seed=int(prbs_seed),
                speed_min_mps=speed_min, speed_max_mps=speed_max,
                target_speed_mps=target_speed, speed_tolerance_mps=speed_tolerance,
                stable_speed_sec=stable_duration, max_steering=max_steering,
                max_steering_rate=max_rate, max_speed_wait_sec=max_speed_wait,
            )
            st.success("计划已生成：{0}".format(plan_path))
        except ValueError as error:
            st.error(str(error))

    st.markdown("#### 采集")
    signal_config = st.text_input(
        "车辆信号映射 YAML", value="vehicle_signals.yaml",
        help="配置 chassis_detail 的 protobuf 类型和前后桥反馈字段；该映射不写入通用代码。")
    output_dir = st.text_input("采集输出目录", value="vehicle_dynamics_runs")
    process = st.session_state.get("lateral_process")
    running = process is not None and process.poll() is None
    if running:
        st.info("采集正在运行，PID: {0}".format(process.pid))
        if st.button("停止横向测试"):
            process.send_signal(signal.SIGINT)
            st.session_state["lateral_process"] = None
            st.rerun()
    else:
        if st.button("开始记录模式", disabled=not plan_path.exists()):
            _start_collection(plan_path, signal_config, output_dir, execute=False)
            st.rerun()
        armed = st.checkbox(
            "我已确认车辆场地、转向单位、信号映射和速度范围，允许主动转向激励。",
            value=False,
        )
        if st.button("开始主动激励", disabled=not (plan_path.exists() and armed)):
            _start_collection(plan_path, signal_config, output_dir, execute=True)
            st.rerun()

    st.markdown("#### 分析")
    run_directory = st.text_input("已完成的运行目录", value="")
    if st.button("生成 Bode 指标与图", disabled=not run_directory):
        try:
            output, _ = write_lateral_frequency_report(run_directory)
            st.success("分析结果：{0}".format(output))
        except (OSError, ValueError) as error:
            st.error(str(error))
