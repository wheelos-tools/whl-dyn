# 底盘性能测试框架与实施原则

本仓库统一采用以下层级进行测试设计：

```text
底盘性能
│
├── 执行器性能（Actuator）
│      Steering
│      Brake
│      Drive
│
├── 车辆动力学（Vehicle Dynamics）
│      Lateral
│      Longitudinal
│      Vertical
│
├── 闭环控制（Closed-loop Control）
│      Path Tracking
│      Speed Tracking
│      Disturbance Rejection
```

所有能力按同一主链路实现：

```text
生成计划 → 执行采集 → 分析画图 → 导出
```

## 分阶段实施策略

### Phase 1（当前优先）
目标：补齐纵向执行器测试能力，不改通信拓扑。

1. 执行器动态指标补齐（Step）
   - Dead Time
   - Rise Time
   - Peak Time
   - Overshoot
   - Settling Time
   - Steady-State Error

2. 频域测试增强（Chirp/Sweep/PRBS）
   - Gain / Phase / Coherence
   - -3 dB Bandwidth
   - Resonance Peak
   - Delay

3. PRBS 全链路接入
   - 计划生成：支持 `prbs` profile 及参数
   - 采集执行：按时间生成 PRBS 控制命令
   - 分析画图：纳入频域分析与 Bode 结果
   - 导出：复用现有 CSV 与分析输出

### Phase 2（已实现：横摆/横向加速度频响）

Steering 执行器 + 横向动力学的以下闭环前数据链路已实现：

1. Chirp/Sweep 和 PRBS 计划生成；
2. `ControlCommand.speed` 保持目标速度，稳定后才开始转向激励；
3. chassis、chassis detail、localization 的通用信号采集；
4. steering-feedback -> yaw-rate、lateral-acceleration 的 Bode 分析；
5. Gain / Phase / Coherence / -3 dB Bandwidth / Resonance Peak / Delay；
6. 每次采集按 `UTC 时间 + case + UUID` 写入独立目录，避免覆盖。

横摆、横向加速度频响可上车测试；Sideslip、Roll 和 Understeer 仍依赖额外的
状态估计或传感器验证，暂不作为本阶段验收输出。

主动测试前，程序必须先以零转向持续下发 `ControlCommand.speed`。实际速度在
`target_mps +/- tolerance_mps` 内稳定 `stable_duration_sec` 后，才开始 Chirp
或 PRBS；运行期间继续下发相同速度目标和转向 profile。`min_mps..max_mps` 是
独立硬保护范围。消息新鲜度、驾驶模式、故障状态的硬中止策略已在计划中预留，
但需完成车种集成验证后启用。

### Phase 3（后续）
闭环控制（Path Tracking / Speed Tracking / Disturbance Rejection）。

## 横向操稳前三阶段实现状态

### 1. 开环执行器与底盘响应辨识：已实现

`whl-dyn plan-open-loop` 生成正负方向的转向阶跃和慢斜坡 case；已有
`plan-lateral` 生成 Chirp/Sweep/PRBS。它们共享恒速门、轮角和轮角速率限制、
前后桥反馈采集与时间戳目录存储。

输出用于确定：

```text
command -> wheel feedback: dead time / rise time / deadzone / hysteresis / rate
wheel feedback -> yaw rate, lateral acceleration: gain / phase / bandwidth
```

### 2. 开环稳态转向与物理极限：已实现计划与指标

`whl-dyn plan-circles` 接收固定转向角、目标速度、斜坡速率和重复次数，生成纯开环
左右转矩阵：只发布 `ControlCommand.speed` 与 `steering_target`，不发布 planning
轨迹。实际半径/曲率与横向加速度是辨识输出。`steady_state_handling_metrics` 对经
人工选择的稳态窗口拟合 `delta - L*kappa = Ku*ay + offset`。

当前不输出侧偏角或极限失稳结论；需要先验证 GNSS/INS 车体系速度和轮角单位。

### 3. 闭环定曲率稳态跟随：已实现连续轨迹发布与 tracking 指标

`whl-dyn plan-closed-loop` 生成 Clothoid -> circle -> Clothoid case；
`whl-dyn run-closed-loop` 直接发布 `/apollo/planning` 的
`ADCTrajectory`。轨迹仅在起始定位时锚定一次，后续帧按试验全局时间截取同一路径，
确保前后帧重叠点的 `x/y/theta/kappa` 连续。

该 runner 必须独占 `/apollo/planning`，不能与普通 planning 模块同时发布。
阶段指标为 `ey/epsi` 的 MAE、RMSE、P95、Peak；稳定 `<=5 cm` 是后续验收目标，
不是当前实现的默认通过判据。

## 设计约束

1. 不修改通信拓扑，不新增底层消息通道。
2. 保持模块化：planning / collection / processing / ui / export 职责清晰。
3. 保持兼容性：旧版 calibration YAML 与 CSV 可继续使用。
4. 代码简洁高效：优先纯函数、最小状态、可测试实现。
