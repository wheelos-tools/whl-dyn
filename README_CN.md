# 车辆动力学标定工具

## 1. 概述

本工具集提供了一套完整的车辆动力学标定解决方案，包括标定计划生成、数据自动收集、数据处理和结果可视化等功能。通过这套工具，可以高效地完成车辆纵向动力学特性的标定工作。

### 1.1 工具组成

- **whl-dyn/tools/generate_plan.py**: 生成车辆动力学标定计划
- **whl-dyn/tools/collect_data.py**: 根据标定计划自动收集数据
- **whl-dyn/tools/process.py**: 处理收集的数据并生成校准表和可视化图表，包括可选的protobuf格式输出
- **whl-dyn/tools/plot.py**: 可视化标定结果

### 1.2 工作流程

```
生成标定计划 → 自动数据收集 → 数据处理 → 结果可视化
(whl-dyn/tools/generate_plan.py) (whl-dyn/tools/collect_data.py) (whl-dyn/tools/process.py) (whl-dyn/tools/plot.py)
```

## 2. 安装和设置

### 2.1 依赖项

确保系统已安装以下依赖：

```bash
# Python依赖
pip install numpy scipy matplotlib pandas pyyaml

# Apollo相关依赖（如需要）
# 需要配置Apollo CyberRT环境
```

### 2.2 环境配置

1. 克隆项目仓库
2. 安装Python依赖
3. 配置Apollo环境（如需要使用自动数据收集功能）

## 3. 使用流程

### 3.1 生成标定计划

使用`generate_plan.py`生成YAML格式的标定计划：

```bash
# 生成默认标定计划
python whl-dyn/tools/generate_plan.py

# 生成自定义标定计划
python whl-dyn/tools/generate_plan.py \
  --throttle-min 10 --throttle-max 80 --throttle-num-steps 8 \
  --brake-min 10 --brake-max 50 --brake-num-steps 5 \
  --speed-targets 1.0 3.0 5.0 7.0 \
  -o my_calibration_plan.yaml
```

**参数说明：**
- `--throttle-min`, `--throttle-max`: 油门测试范围（%）
- `--throttle-num-steps`: 油门测试步数
- `--brake-min`, `--brake-max`: 刹车测试范围（%）
- `--brake-num-steps`: 刹车测试步数
- `--speed-targets`: 加速测试的目标速度（m/s）
- `-o`: 输出文件名

### 3.2 数据收集

使用`collect_data.py`根据标定计划自动收集数据：

```bash
# 使用默认计划文件和输出目录
python whl-dyn/tools/collect_data.py

# 使用自定义计划文件
python whl-dyn/tools/collect_data.py -p my_calibration_plan.yaml

# 使用自定义输出目录
python whl-dyn/tools/collect_data.py -o /path/to/output/directory

# 同时指定计划文件和输出目录
python whl-dyn/tools/collect_data.py -p my_calibration_plan.yaml -o /path/to/output/directory
```

收集的数据将保存在指定的输出目录中，每个测试用例生成一个CSV文件。

### 3.3 数据处理

使用`whl-dyn/tools/process.py`处理收集的数据并生成可视化图表：

```bash
# 处理目录下所有CSV文件并生成可视化图表
python whl-dyn/tools/process.py -i data_directory/ -o results/

# 处理目录下所有CSV文件并同时生成校准表
python whl-dyn/tools/process.py -i data_directory/ -o results/ --output-calibration-table
```

**参数说明：**
- `-i`, `--input_dir`: 包含原始CSV数据日志的目录（必需）
- `-o`, `--output_dir`: 保存最终图表和表格的目录（可选，默认为`./calibration_results`）
- `--output-calibration-table`: 同时输出校准表（可选）

处理结果将生成`unified_calibration_table.csv`文件，包含速度、命令和加速度的映射关系，以及3D表面图和2D等高线图。如果指定了`--output-calibration-table`标志，还将生成名为`calibration_table.pb.txt`的校准表，格式为protobuf（如果protobuf模块可用）或原生格式。

### 3.4 结果可视化

使用`whl-dyn/tools/plot.py`可视化标定结果：

```bash
# 使用默认输入文件
python whl-dyn/tools/plot.py

# 使用自定义输入文件
python whl-dyn/tools/plot.py -i calibration_data.txt
```

可视化工具将生成油门和刹车动力学的3D表面图和2D等高线图。

## 4. 安全操作指南

### 4.1 现场操作规范

1. **测试前检查**
   - 确保测试区域安全，无人员和障碍物
   - 检查车辆状态，确保电池电量充足
   - 确认车辆处于良好的工作状态
   - 检查传感器和控制系统工作正常

2. **操作人员要求**
   - 操作人员必须经过专业培训
   - 现场必须有安全监督员
   - 操作人员应熟悉紧急停止程序

3. **测试环境要求**
   - 选择平坦、宽敞的测试场地
   - 确保场地表面摩擦系数稳定
   - 避免在恶劣天气条件下进行测试

### 4.2 安全注意事项

1. **紧急情况处理**
   - 测试过程中如发现异常，立即按下紧急停止按钮
   - 保持与车辆的安全距离
   - 准备好手动控制设备作为备用

2. **参数设置安全**
   - 初次测试时使用较低的油门和刹车值
   - 逐步增加测试强度
   - 避免设置过高的目标速度

3. **设备安全**
   - 定期检查传感器和执行器
   - 确保通信链路稳定
   - 备份重要数据

## 5. 故障排除

### 5.1 常见问题

1. **无法连接到车辆**
   - 检查网络连接
   - 确认CyberRT服务已启动
   - 检查防火墙设置

2. **数据收集失败**
   - 检查传感器数据是否正常
   - 确认车辆处于正确的驾驶模式
   - 检查输出目录权限

3. **处理结果异常**
   - 检查输入数据质量
   - 确认参数设置合理
   - 查看日志文件获取详细信息

## 6. 输入输出文件格式说明

### 6.1 标定计划文件 (YAML格式)

标定计划文件定义了测试用例和步骤，格式如下：

```yaml
- case_name: "throttle_30_to_5mps"
  description: "Accelerate with 30% throttle, target >5m/s, then brake to stop."
  steps:
    - command:
        throttle: 30.0
        brake: 0.0
      trigger:
        type: "speed_greater_than"
        value: 5.0
      timeout_sec: 30.0
    - command:
        throttle: 0.0
        brake: 30.0
      trigger:
        type: "speed_less_than"
        value: 0.1
      timeout_sec: 30.0

- case_name: "brake_20_from_5mps"
  description: "Accelerate to >5m/s, then apply 20% brake."
  steps:
    - command:
        throttle: 80.0
        brake: 0.0
      trigger:
        type: "speed_greater_than"
        value: 5.0
      timeout_sec: 30.0
    - command:
        throttle: 0.0
        brake: 20.0
      trigger:
        type: "speed_less_than"
        value: 0.1
      timeout_sec: 30.0
```

**字段说明：**
- `case_name`: 测试用例名称
- `description`: 测试用例描述
- `steps`: 测试步骤列表
  - `command`: 控制命令
    - `throttle`: 油门命令（%）
    - `brake`: 刹车命令（%）
  - `trigger`: 触发条件
    - `type`: 触发类型（`speed_greater_than` 或 `speed_less_than`）
    - `value`: 触发值（m/s）
  - `timeout_sec`: 步骤超时时间（秒）

### 6.2 数据日志文件 (CSV格式)

数据日志文件记录了测试过程中的车辆状态信息，格式如下：

```
time,speed_mps,imu_accel_y,driving_mode,actual_gear,throttle_pct,brake_pct,ctl_throttle,ctl_brake
123456.7890,2.5,0.3,1,1,25.0,0.0,30.0,0.0
123456.7990,2.6,0.4,1,1,25.0,0.0,30.0,0.0
...
```

**字段说明：**
- `time`: 时间戳（秒）
- `speed_mps`: 车辆速度（m/s）
- `imu_accel_y`: IMU Y轴加速度（m/s²）
- `driving_mode`: 驾驶模式
- `actual_gear`: 实际档位
- `throttle_pct`: 实际油门百分比
- `brake_pct`: 实际刹车百分比
- `ctl_throttle`: 控制油门命令（%）
- `ctl_brake`: 控制刹车命令（%）

### 6.3 处理结果文件 (CSV格式)

处理结果文件包含速度、命令和加速度的映射关系：

```
speed,command,acceleration
0.00,0.00,0.0000
0.00,2.00,0.0000
0.00,4.00,0.0000
...
```

**字段说明：**
- `speed`: 速度值（m/s）
- `command`: 命令值（正数表示油门，负数表示刹车）
- `acceleration`: 加速度值（m/s²）

注意：`whl-dyn/tools/process.py`脚本生成一个名为`unified_calibration_table.csv`的文件，该文件包含标题行和三列，分别表示速度、命令和加速度值。

### 6.4 校准表文件 (Protocol Buffer文本格式)

校准表文件包含最终的校准数据：

```
calibration_table {
  calibration {
    speed: 0.10000000149011612
    acceleration: -3.085177183151245
    command: -50.599998474121094
  }
  calibration {
    speed: 0.10000000149011612
    acceleration: -3.0587549209594727
    command: -48.042103817588405
  }
  ...
}
```

**字段说明：**
- `speed`: 速度值（m/s）
- `acceleration`: 加速度值（m/s²）
- `command`: 命令值（正数表示油门，负数表示刹车）

## 7. 附录

### 7.1 命令行参数参考

#### generate_plan.py
```
usage: generate_plan.py [-h] [-o OUTPUT] [--throttle-min THROTTLE_MIN]
                        [--throttle-max THROTTLE_MAX]
                        [--throttle-num-steps THROTTLE_NUM_STEPS]
                        [--brake-min BRAKE_MIN] [--brake-max BRAKE_MAX]
                        [--brake-num-steps BRAKE_NUM_STEPS]
                        [--speed-targets SPEED_TARGETS [SPEED_TARGETS ...]]
                        [--default-brake DEFAULT_BRAKE]
                        [--accel-timeout ACCEL_TIMEOUT]
                        [--decel-timeout DECEL_TIMEOUT]

Generate a YAML plan for vehicle longitudinal calibration.

optional arguments:
  -h, --help            show this help message and exit
  -o OUTPUT, --output OUTPUT
                        Output YAML file name.
  --throttle-min THROTTLE_MIN
                        Minimum throttle command (%) to test.
  --throttle-max THROTTLE_MAX
                        Maximum throttle command (%) to test.
  --throttle-num-steps THROTTLE_NUM_STEPS
                        Number of throttle steps to generate.
  --brake-min BRAKE_MIN
                        Minimum brake command (%) to test.
  --brake-max BRAKE_MAX
                        Maximum brake command (%) to test.
  --brake-num-steps BRAKE_NUM_STEPS
                        Number of brake steps to generate.
  --speed-targets SPEED_TARGETS [SPEED_TARGETS ...]
                        List of target speeds (m/s) for acceleration tests.
  --default-brake DEFAULT_BRAKE
                        Default brake command (%) used to stop the vehicle
                        after a test step.
  --accel-timeout ACCEL_TIMEOUT
                        Timeout in seconds for acceleration steps.
  --decel-timeout DECEL_TIMEOUT
                        Timeout in seconds for deceleration steps.
```

#### collect_data.py
```
usage: collect_data.py [-h] [-p PLAN] [-o OUTPUT_DIR]

Production-Ready, Plan-Driven Data Collector for Apollo.

optional arguments:
  -h, --help            show this help message and exit
  -p PLAN, --plan PLAN  Path to the YAML calibration plan file (default:
                        calibration_plan.yaml)
  -o OUTPUT_DIR, --output-dir OUTPUT_DIR
                        Output directory for collected data files (default:
                        ./calibration_data_logs)
```

#### tools/process.py
```
usage: process.py [-h] [-i INPUT_DIR] [-o OUTPUT_DIR] [--output-calibration-table]

Process and visualize raw vehicle data to generate a unified calibration table.

optional arguments:
  -h, --help            show this help message and exit
  -i INPUT_DIR, --input-dir INPUT_DIR
                        Directory containing the raw CSV data logs.
  -o OUTPUT_DIR, --output-dir OUTPUT_DIR
                        Directory to save the final plots and table (default:
                        ./calibration_results)
  --output-calibration-table
                        Also output calibration table in protobuf or native
                        format.
```

#### tools/plot.py
```
usage: plot.py [-h] [-i INPUT]

Plot vehicle dynamics calibration data.

optional arguments:
  -h, --help            show this help message and exit
  -i INPUT, --input INPUT
                        Input calibration data file (default:
                        calibration_data.txt)
```