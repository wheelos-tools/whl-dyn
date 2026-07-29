# 当前可执行测试用例清单（可直接用于路测）

本文只列出**当前仓库已经实现并可执行**的测试项，统一遵循主链路：

`生成计划 → 执行采集 → 分析画图 → 导出`

参考原则：`docs/context/chassis_test_framework.md`。

## 1. 覆盖范围（当前可测）

- 执行器性能（Actuator）：**Drive(Throttle)**、**Brake**
- 车辆动力学（Vehicle Dynamics）：**Longitudinal（纵向）**
- 测试类型：静态标定（速度触发阶梯）、动态激励（Step/Ramp/Pulse/Triangle/Hysteresis/Single-Sine/Chirp/Sweep/Multi-Sine/PRBS）

> 当前未覆盖：Steering、Lateral/Vertical、Closed-loop Path/Speed/抗扰（见文末建议）。

## 2. 测试用例总表（建议先跑这些）

| 用例ID | 类别 | 执行器 | 模式 | 主要输入 | 主要输出 | 当前可得指标 |
|---|---|---|---|---|---|---|
| LON-CAL-THR-01 | 静态标定 | Throttle | speed-trigger step | 油门阶梯 + 速度目标 | `speed_mps`, `imu_accel_y` | 死区、线性度、平滑度、单调性、残差 |
| LON-CAL-BRK-01 | 静态标定 | Brake | speed-trigger step | 刹车阶梯 + 初速度 | `speed_mps`, `imu_accel_y` | 死区、线性度、平滑度、单调性、残差 |
| ACT-DYN-THR-STEP-01 | 动态响应 | Throttle | step | `baseline`,`amplitude`,`start_time_sec` | `command`,`imu_accel_y` | dead time, rise, peak time, overshoot, settling, SSE, gain, tau |
| ACT-DYN-BRK-STEP-01 | 动态响应 | Brake | step | 同上 | 同上 | 同上 |
| ACT-DYN-THR-SWEEP-01 | 频响 | Throttle | sweep/chirp | `f_start`,`f_end`,`duration` | `command`,`imu_accel_y` | gain, phase, coherence, -3dB带宽, 共振峰, 延迟估计 |
| ACT-DYN-BRK-SWEEP-01 | 频响 | Brake | sweep/chirp | 同上 | 同上 | 同上 |
| ACT-DYN-THR-PRBS-01 | 辨识激励 | Throttle | prbs | `bit_duration_sec`,`prbs_seed`,`prbs_low/high` | `command`,`imu_accel_y` | 同频响指标（Bode+coherence+带宽/共振/延迟） |
| ACT-DYN-BRK-PRBS-01 | 辨识激励 | Brake | prbs | 同上 | 同上 | 同上 |

## 3. 推荐执行顺序（一次完整回归）

1. `LON-CAL-THR-01` + `LON-CAL-BRK-01`（先拿到稳定标定面）
2. `ACT-DYN-*-STEP-01`（确认时域动态）
3. `ACT-DYN-*-SWEEP-01`（确认频响范围）
4. `ACT-DYN-*-PRBS-01`（补全辨识激励）
5. 导出并归档（CSV + 指标 + 图）

## 4. 计划模板（YAML）

### 4.1 Sweep/Chirp（频响）

```yaml
- case_name: dynamic_throttle_sweep
  description: Open-loop throttle sweep
  dynamic: true
  domain: frequency_response
  mode: sweep
  actuator: throttle
  sampling_rate_hz: 50
  duration_sec: 30
  input_signals: [command]
  output_signals: [speed_mps, imu_accel_y]
  command_profile:
    type: sweep
    baseline: 10
    amplitude: 15
    frequency_start_hz: 0.1
    frequency_end_hz: 3.0
    duration_sec: 30
    method: linear
```

### 4.2 PRBS（辨识激励）

```yaml
- case_name: dynamic_brake_prbs
  description: Open-loop brake prbs
  dynamic: true
  domain: frequency_response
  mode: prbs
  actuator: brake
  sampling_rate_hz: 50
  duration_sec: 30
  input_signals: [command]
  output_signals: [speed_mps, imu_accel_y]
  command_profile:
    type: prbs
    baseline: 0
    bit_duration_sec: 0.1
    prbs_seed: 7
    prbs_low: 5
    prbs_high: 25
    start_time_sec: 1.0
    end_time_sec: 29.0
```

## 5. 分析与导出检查项（执行后必看）

- Step：`dead_time_sec / rise_time_sec / peak_time_sec / overshoot_pct / settling_time_sec / steady_state_error / gain / time_constant_sec`
- 频域：`magnitude_db / phase_deg / coherence / bandwidth_hz / resonance_peak_hz / resonance_peak_db / estimated_delay_sec`
- 标定面：油门与刹车死区、单调性违反数、残差（MAE/RMSE）
- 导出：`unified_calibration_table.csv`、`calibration_table.pb.txt`、`evaluation_metrics.json`、step图

## 6. 不完整项与补充建议（建议你本轮测试时关注）

1. 频域分析中 `bandwidth_hz` 目前基于首频点参考的 -3dB 近似，建议实测时结合 coherence 门限和工作点分段复核。  
2. `estimated_delay_sec` 由相位反推，建议仅在高相干频段解释，不应单独作为放行依据。  
3. PRBS 目前输出到统一频域分析链路，建议你测试时增加不同 `bit_duration_sec`（如 0.05/0.1/0.2s）做稳定性对比。  
4. 当前输出信号以 `imu_accel_y` 与 `speed_mps` 为主，若要提高刹车建压解释度，后续可补充 brake pressure（不在本阶段）。  
5. 建议每个关键 case 至少重复 3 次，并保留环境信息（路面坡度、载荷、温度）用于结果筛选。  
