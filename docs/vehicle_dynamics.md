# Vehicle Actuator Identification Platform

本工具沿用 Streamlit 的“生成计划 → 执行采集 → 分析画图”流程。除原有油门/刹车
标定外，还支持执行器动态和频率响应测试。

## 网页操作流程

1. 在 **生成计划** 标签中保留原来的油门/刹车参数，点击“生成计划”即可生成旧
   格式的 speed-trigger case。动态测试在“动态测试计划”折叠区中生成；Step 和
   Sweep 是优先入口，也可选择其它 profile。
2. 在 **数据采集** 标签选择同一个 YAML。动态 case 会显示名称、类型、执行器和
   持续时间，并可复用“开始/停止/重试/清除”。动态采集不依赖 speed trigger，而
   是按单一 profile 随时间发布命令。
3. 在 **分析** 标签选择采集目录。动态 CSV 自动显示时域响应（Step）或 Bode
   幅值/相位/coherence（Sine、Chirp、Sweep、Multi-Sine）。Step 还显示 dead
   time、rise/settling time、gain、time constant 和基础 FOPDT 辨识结果；原有
   标定曲面、标定表和导出功能不变。

## YAML 动态 case 示例

```yaml
- case_name: dynamic_throttle_sweep
  description: Open-loop throttle sweep
  dynamic: true
  domain: frequency_response       # actuator_characterization 或 frequency_response
  mode: sweep                       # step/ramp/pulse/triangle/hysteresis/
                                   # single_sine/chirp/sweep/multi_sine
  actuator: throttle                # throttle 或 brake
  sampling_rate_hz: 50
  duration_sec: 20
  input_signals: [command]
  output_signals: [speed_mps, imu_accel_y]
  command_profile:
    type: sweep
    baseline: 10
    amplitude: 15
    frequency_start_hz: 0.1
    frequency_end_hz: 3.0
    duration_sec: 20
    method: linear
```

动态类型包括 Actuator Characterization 的 `step`、`ramp`、`pulse`、`triangle`、
`hysteresis`，以及 Frequency Response 的 `single_sine`、`chirp`、`sweep`、
`multi_sine`。通用 profile 字段有 `type`、`baseline`、`amplitude`、
`start_time_sec`、`duration_sec`、`period_sec`、`frequency_hz`、
`frequency_start_hz`、`frequency_end_hz`、`frequencies_hz`、`amplitudes` 和
`phases_rad`。Step 使用基线/幅值，Ramp 使用 `ramp_start_sec`/`ramp_end_sec`，
Pulse 使用 `pulse_duration_sec`，Triangle/Hysteresis 使用 `period_sec`，
Multi-Sine 使用频率和幅值数组。`profile` 是 `command_profile` 的兼容别名。
YAML 顶层仍是 case 列表，因此旧的 `steps` + `trigger` 标定 YAML 无需修改。

## 输出格式

每个 case 生成 `{case_name}_{index}.csv`。每个 chassis 回调写入时间、速度、INS
速度、IMU 横向加速度、实测油门/刹车和控制油门/刹车；动态日志另外写入 signed
`command`、`domain`、`mode`、`actuator`、`profile_type`，供分析标签自动识别。
旧标定 CSV 仍保持原有列格式。结束或中止时油门归零并发送安全刹车命令。

动态分析 API 位于 `whl_dyn.processing.dynamics`，返回只含 Python 标量和列表的
JSON-friendly 结果；`data` 字段可直接用于 Plotly 或 matplotlib。输出包括 Step
指标、Welch/CSD 频响、coherence，以及一阶 FOPDT 的 gain、dead time 和 time
constant。
