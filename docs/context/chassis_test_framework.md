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

### Phase 2（后续）
Steering 执行器 + 横向动力学（Yaw/LatAcc/Sideslip/Roll/Understeer）。

### Phase 3（后续）
闭环控制（Path Tracking / Speed Tracking / Disturbance Rejection）。

## 设计约束

1. 不修改通信拓扑，不新增底层消息通道。
2. 保持模块化：planning / collection / processing / ui / export 职责清晰。
3. 保持兼容性：旧版 calibration YAML 与 CSV 可继续使用。
4. 代码简洁高效：优先纯函数、最小状态、可测试实现。
