# 圈速仿真结果绘图使用指南

本文说明如何使用
`simulation/time_domain_closed_loop/plotting/plotLapSimulationResults.m`
读取圈速仿真结果，
选择横轴和纵轴参数、设置显示范围与单位，并导出图片。

适用环境：MATLAB / Simulink R2026a。

## 1. 快速开始

在 MATLAB 中将当前文件夹切换到项目根目录，然后运行：

```matlab
projectRoot = pwd;
addpath(genpath(projectRoot));
run(fullfile(projectRoot, "simulation", "time_domain_closed_loop", ...
    "plotting", "plotLapSimulationResults.m"));
```

绘图脚本不会重新运行 Simulink，只读取已经保存的
`lap_simulation_result.mat`。默认加载最新的 Skidpad 结果。

主要配置位于
`simulation/time_domain_closed_loop/plotting/plotLapSimulationResults.m` 顶部：

```matlab
% 空字符串：读取指定赛事下的最新结果
plotCfg.Result.File = "";
plotCfg.Result.Track = "skidpad";

plotCfg.X.Parameter = "Distance";
plotCfg.X.Range = [0, 1000];

plotCfg.Y.Parameters = [
    "Vehicle.Speed"
    "Vehicle.Ax"
    "Vehicle.Ay"
    "Wheel.NormalLoad"
];

plotCfg.Y.Ranges = [
      0, 30
    -10, 10
    -15, 15
      0, NaN
];
```

`tabs` 模式为每个纵轴参数在同一图窗中创建独立标签页。纵轴参数不要求固定为四个；
可以选择任意正整数个有效字段。

## 2. 选择结果文件

### 2.1 加载指定赛事的最新结果

```matlab
plotCfg.Result.File = "";
plotCfg.Result.Track = "skidpad";
```

`Result.Track` 可选：

- `"acceleration"`
- `"skidpad"`
- `"autocross"`
- `"endurance"`

### 2.2 加载指定结果文件

```matlab
plotCfg.Result.File = ...
    "E:\FSAE_model\results\time_domain_closed_loop\skidpad\" + ...
    "20260730_165849_boundary_fix_validation\lap_simulation_result.mat";
```

设置了 `Result.File` 后，`Result.Track` 不参与文件选择。

## 3. 横轴参数

### 3.1 推荐横轴

以下字段是正式推荐的横轴：

| 参数 | 单位 | 说明 |
|---|---:|---|
| `Time` | s | 仿真时间 |
| `Distance` | m | 根据车辆实际 `X/Y` 累积的行驶距离 |
| `Track.EventDistance` | m | 解回绕后的全赛项中心线距离 |
| `Track.PathS` | m | 当前参考点在单圈中心线上的弧长 |
| `Track.Progress` | 1 | 全赛项进度，范围通常为 0–1 |
| `Vehicle.X` | m | 车辆全局 X 坐标，适合 XY 轨迹图 |
| `Vehicle.Y` | m | 车辆全局 Y 坐标 |

示例：

```matlab
plotCfg.X.Parameter = "Time";
plotCfg.X.Range = [0, 20];
```

```matlab
plotCfg.X.Parameter = "Track.EventDistance";
plotCfg.X.Range = [NaN, NaN];  % 自动范围
```

### 3.2 其他可用横轴

结果中任何第一维等于 `numel(result.Time)`、尺寸为 `N×1` 的数值或逻辑时序
字段都可作为横轴。本文第 5 节表格中标为 `N×1` 的字段均满足这一条件。

不建议使用 `N×4` 四轮字段作为横轴。当前绘图器只对纵轴执行四轮通道选择。

横轴不要求严格单调。`Vehicle.X`、`Vehicle.Y`、横向误差等非单调横轴仍可使用，
范围筛选按布尔掩码处理。

## 4. 纵轴配置

### 4.1 任意数量的纵轴参数

```matlab
plotCfg.Y.Parameters = [
    "Vehicle.Speed"
    "Vehicle.Ax"
    "Track.LateralError"
];
```

`Y.Ranges` 按行依次对应 `Y.Parameters`：

- 范围行多于参数数量：自动忽略多余行；
- 范围行少于参数数量：缺少的行自动使用 `[NaN NaN]`；
- `NaN` 表示该侧自动缩放；
- `Y.Ranges = []` 表示全部纵轴自动缩放。

```matlab
plotCfg.Y.Ranges = [
     0, 15
   -10, 10
];  % 第三个纵轴自动缩放
```

### 4.2 四轮通道

所有 `N×4` 字段的列顺序固定为：

```text
[FL, FR, RL, RR]
```

可选择并重排通道：

```matlab
plotCfg.Wheel.Channels = ["FL", "FR", "RL", "RR"];
```

```matlab
plotCfg.Wheel.Channels = ["RL", "RR"];  % 只绘制后轮
```

通道不得重复。

## 5. 全部纵轴参数

所有下列时序字段均可作为纵轴。`N×1` 表示单曲线，`N×4` 表示四轮曲线。
字段是否实际存在取决于仿真模型和日志配置；空字段会被警告并跳过，不会用零值
代替。

### 5.1 根字段与赛道

| 参数 | 尺寸 | 单位 | 说明 |
|---|---:|---:|---|
| `Time` | N×1 | s | 仿真时间 |
| `Distance` | N×1 | m | 车辆实际累计距离 |
| `Track.PathS` | N×1 | m | 单圈中心线弧长 |
| `Track.EventDistance` | N×1 | m | 全赛项中心线累计距离 |
| `Track.LapIndex` | N×1 | 1 | 当前圈序号 |
| `Track.Progress` | N×1 | 1 | 赛项进度，通常为 0–1 |
| `Track.ReferenceSpeed` | N×1 | m/s | 参考车速；自适应驾驶员模式为兼容 Bus 的全零占位，不是停车目标 |
| `Track.Curvature` | N×1 | 1/m | 参考路径曲率 |
| `Track.LateralError` | N×1 | m | 车辆相对最近赛道中心线的横向误差 |
| `Track.BoundaryViolation` | N×1 | m | 超出左右边界的距离，未越界时为 0 |
| `Track.VehicleEnvelopeClearance` | N×1 | m | 前后轴四个轮胎外缘点的最小边界净空；正值在界内 |
| `Track.VehicleEnvelopeViolation` | N×1 | m | 车辆包络越界量；界内为 0 |

### 5.2 车辆状态

| 参数 | 尺寸 | 单位 | 说明 |
|---|---:|---:|---|
| `Vehicle.X` | N×1 | m | 全局 X 坐标 |
| `Vehicle.Y` | N×1 | m | 全局 Y 坐标 |
| `Vehicle.Psi` | N×1 | rad | 车辆航向角 |
| `Vehicle.Ux` | N×1 | m/s | 车身纵向速度 |
| `Vehicle.Uy` | N×1 | m/s | 车身横向速度 |
| `Vehicle.Speed` | N×1 | m/s | 合速度 |
| `Vehicle.YawRate` | N×1 | rad/s | 横摆角速度 |
| `Vehicle.RollAngle` | N×1 | rad | 侧倾角 |
| `Vehicle.PitchAngle` | N×1 | rad | 俯仰角 |
| `Vehicle.Ax` | N×1 | m/s² | 纵向加速度 |
| `Vehicle.Ay` | N×1 | m/s² | 横向加速度 |
| `Vehicle.Az` | N×1 | m/s² | 垂向加速度 |

### 5.3 车轮与轮胎

| 参数 | 尺寸 | 单位 | 说明 |
|---|---:|---:|---|
| `Wheel.Speed` | N×4 | rad/s | 四轮角速度 |
| `Wheel.RPM` | N×4 | rpm | 四轮转速 |
| `Wheel.SteerAngle` | N×4 | rad | 四轮转角 |
| `Wheel.NormalLoad` | N×4 | N | 四轮法向载荷 |
| `Wheel.SlipRatio` | N×4 | 1 | 四轮滑移率 |
| `Wheel.SlipAngle` | N×4 | rad | 四轮侧偏角 |
| `Wheel.CamberAngle` | N×4 | rad | 四轮外倾角 |
| `Tire.FxWheel` | N×4 | N | 轮胎坐标系纵向力 |
| `Tire.FyWheel` | N×4 | N | 轮胎坐标系横向力 |
| `Tire.MuUtilization` | N×4 | 1 | 由 `VehicleStateBus.MuUtilization` 记录的四轮摩擦利用率 |

### 5.4 动力系统与电池

| 参数 | 尺寸 | 单位 | 说明 |
|---|---:|---:|---|
| `Powertrain.MotorTorqueActual` | N×4 | N·m | 四电机实际转矩 |
| `Powertrain.MotorSpeed` | N×4 | rad/s | 四电机角速度 |
| `Powertrain.MotorRPM` | N×4 | rpm | 四电机转速 |
| `Powertrain.MotorMechanicalPower` | N×4 | W | 四电机机械功率 |
| `Powertrain.MotorLimitActive` | N×4 | 1 | 四电机限制状态 |
| `Powertrain.TotalPowerLimitActive` | N×1 | 1 | 总功率限制状态 |
| `Powertrain.RegenLimitActive` | N×1 | 1 | 再生限制状态 |
| `Battery.Voltage` | N×1 | V | 电池端电压 |
| `Battery.Current` | N×1 | A | 电池电流 |
| `Battery.Power` | N×1 | W | 电池功率 |
| `Battery.SOC` | N×1 | 1 | 电池荷电状态 |

### 5.5 驾驶员与执行器

| 参数 | 尺寸 | 单位 | 说明 |
|---|---:|---:|---|
| `Driver.SteeringWheelAngleRequest` | N×1 | rad | 方向盘角请求 |
| `Driver.SteeringRackAngleRequest` | N×1 | rad | 转向齿条角请求 |
| `Driver.LongitudinalAccelerationRequest` | N×1 | m/s² | 纵向加速度请求 |
| `Driver.DriveTorqueRequest` | N×1 | N·m | 总驱动转矩请求 |
| `Driver.BrakePressureRequest` | N×1 | Pa | 制动压力请求 |
| `Actuator.SteeringRackAngleRequest` | N×1 | rad | 执行器齿条角请求 |
| `Actuator.MotorTorqueRequest` | N×4 | N·m | 四电机转矩请求 |
| `Actuator.FrictionBrakeTorqueRequest` | N×4 | N·m | 四轮摩擦制动转矩请求 |
| `Actuator.RegenTorqueRequest` | N×4 | N·m | 四轮再生转矩请求 |
| `Actuator.TotalPowerRequest` | N×1 | W | 总功率请求 |

### 5.6 控制器

| 参数 | 尺寸 | 单位 | 说明 |
|---|---:|---:|---|
| `Controller.ReferenceYawRate` | N×1 | rad/s | 参考横摆角速度 |
| `Controller.YawRateError` | N×1 | rad/s | 横摆角速度误差 |
| `Controller.DesiredLongitudinalForce` | N×1 | N | 目标纵向力 |
| `Controller.DesiredYawMoment` | N×1 | N·m | 目标横摆力矩 |
| `Controller.AllocatedYawMoment` | N×1 | N·m | 已分配横摆力矩 |
| `Controller.TVActive` | N×1 | 1 | 转矩矢量控制状态 |
| `Controller.TCActive` | N×1 | 1 | 牵引力控制状态 |
| `Controller.RegenActive` | N×1 | 1 | 再生制动状态 |
| `Controller.EnergyManagementActive` | N×1 | 1 | 能量管理状态 |
| `Controller.ControllerSaturated` | N×1 | 1 | 控制器饱和状态 |

### 5.7 传感器

| 参数 | 尺寸 | 单位 | 说明 |
|---|---:|---:|---|
| `Sensor.PositionX` | N×1 | m | 测量全局 X 坐标 |
| `Sensor.PositionY` | N×1 | m | 测量全局 Y 坐标 |
| `Sensor.Heading` | N×1 | rad | 测量航向角 |
| `Sensor.LongitudinalSpeed` | N×1 | m/s | 测量纵向速度 |
| `Sensor.LateralSpeed` | N×1 | m/s | 测量横向速度 |
| `Sensor.YawRate` | N×1 | rad/s | 测量横摆角速度 |
| `Sensor.AccelX` | N×1 | m/s² | 测量纵向加速度 |
| `Sensor.AccelY` | N×1 | m/s² | 测量横向加速度 |
| `Sensor.WheelSpeed` | N×4 | rad/s | 测量四轮角速度 |
| `Sensor.SteeringRackAngle` | N×1 | rad | 测量齿条角 |
| `Sensor.MotorSpeed` | N×4 | rad/s | 测量四电机角速度 |
| `Sensor.MotorTorqueEstimate` | N×4 | N·m | 四电机转矩估计 |
| `Sensor.BatteryVoltage` | N×1 | V | 测量电池电压 |
| `Sensor.BatteryCurrent` | N×1 | A | 测量电池电流 |
| `Sensor.PoseValid` | N×1 | 1 | 位姿信号有效位 |
| `Sensor.IMUValid` | N×1 | 1 | IMU 信号有效位 |
| `Sensor.WheelSpeedValid` | N×4 | 1 | 四轮轮速有效位 |
| `Sensor.PowertrainValid` | N×1 | 1 | 动力系统信号有效位 |

`result.Metrics`、`result.Meta`、`result.Config` 是摘要或配置，不是长度为 N 的
时序，因此不能直接作为横轴或纵轴。

自适应模式用 `result.Meta.ReferenceSpeedUsed=false` 标识无参考速度控制。专用
路径/整车/圈速报告在该模式下只绘制实际车速，`SpeedRMSE` 为 `NaN`（不适用），
并报告 `MinimumVehicleEnvelopeClearance`、`MaximumVehicleEnvelopeViolation`、
越界计数和比例。质心 `BoundaryViolation` 不能替代车辆包络验收。

## 6. 显示单位

单位转换只影响图形显示，不修改 MAT 文件中的原始数据。

```matlab
plotCfg.Units.Speed = "m/s";            % 或 "km/h"
plotCfg.Units.Angle = "deg";            % 或 "rad"
plotCfg.Units.RotationalSpeed = "rpm";  % 或 "rad/s"
```

说明：

- 速度字段内部单位为 m/s；
- 角度字段内部单位为 rad；
- `Wheel.Speed`、`Powertrain.MotorSpeed` 和 `Sensor.MotorSpeed` 内部单位为 rad/s；
- `Wheel.RPM` 和 `Powertrain.MotorRPM` 已经是 rpm；
- 状态、有效位和限制标志按 0/1 显示。

## 7. 布局

### 7.1 标签页图窗

```matlab
plotCfg.Layout = "tabs";
```

每个有效纵轴字段创建一个独立标签页。不同物理单位可同时使用。

### 7.2 叠加

```matlab
plotCfg.Layout = "overlay";
```

所有纵轴字段绘制在同一坐标轴。只有单位相同的字段可以叠加，否则脚本会报
`FSAE:Lap:OverlayUnitMismatch`。

## 8. 常用示例

### 8.1 车速、加速度与横向误差

```matlab
plotCfg.X.Parameter = "Time";
plotCfg.X.Range = [0, NaN];

plotCfg.Y.Parameters = [
    "Vehicle.Speed"
    "Vehicle.Ax"
    "Vehicle.Ay"
    "Track.LateralError"
];
plotCfg.Y.Ranges = [];
plotCfg.Layout = "tabs";
```

### 8.2 四轮载荷和轮胎力

```matlab
plotCfg.X.Parameter = "Distance";
plotCfg.X.Range = [NaN, NaN];

plotCfg.Y.Parameters = [
    "Wheel.NormalLoad"
    "Tire.FxWheel"
    "Tire.FyWheel"
];
plotCfg.Wheel.Channels = ["FL", "FR", "RL", "RR"];
plotCfg.Y.Ranges = [];
```

### 8.3 车辆 XY 轨迹

```matlab
plotCfg.X.Parameter = "Vehicle.X";
plotCfg.X.Range = [NaN, NaN];
plotCfg.Y.Parameters = "Vehicle.Y";
plotCfg.Y.Ranges = [NaN, NaN];
```

使用 `Vehicle.X` 作为横轴时，绘图器自动采用等比例坐标。

### 8.4 只绘制后轮电机转矩

```matlab
plotCfg.Y.Parameters = "Powertrain.MotorTorqueActual";
plotCfg.Wheel.Channels = ["RL", "RR"];
plotCfg.Y.Ranges = [NaN, NaN];
```

## 9. 导出

```matlab
plotCfg.Export.Enabled = true;
plotCfg.Export.Formats = "fig";
plotCfg.Export.Directory = "";
```

当前仅支持 `"fig"`：MATLAB FIG，可在 MATLAB 中直接打开和编辑。

`Export.Directory=""` 时优先导出到当前结果的输出目录。

## 10. 查看当前结果实际可用字段

绘图入口运行时会自动打印字段清单。也可以在结果已加载到变量 `result` 后执行：

```matlab
listLapResultFields(result, Display = true);
```

只返回字段字符串：

```matlab
fields = listLapResultFields(result);
```

当前模型新增字段后，只要它是非空的数值或逻辑时序，且第一维长度等于
`numel(result.Time)`，就会自动出现在清单中。

## 11. 常见错误

| 错误 | 原因与处理 |
|---|---|
| `NoSavedResults` | 指定赛事目录没有结果；修改 `Result.Track` 或设置 `Result.File` |
| `UnknownResultField` | 字段名拼写错误或当前结果没有记录该字段；查看自动打印的字段清单 |
| `YFieldEmpty` | 字段存在但没有数据；检查 `result.Meta.MissingSignals` |
| `EmptyRange` | `X.Range` 与实际横轴范围没有交集 |
| `InvalidYAxisRanges` | `Y.Ranges` 不是两列；每行必须为 `[下限 上限]` |
| `OverlayUnitMismatch` | overlay 中选择了不同单位的字段；改用 tabs |
| `InvalidWheelChannels` | 四轮通道名称错误或重复 |

## 12. 相关文件

- 绘图入口：`simulation/time_domain_closed_loop/plotting/plotLapSimulationResults.m`
- 结果加载：`scripts/reporting/loadLapSimulationResult.m`
- 字段列举：`scripts/reporting/listLapResultFields.m`
- 字段提取与单位转换：`scripts/reporting/getLapResultSignal.m`
- 绘图与导出：`scripts/reporting/plotLapResultSelection.m`
- 功能规格：[圈速仿真与结果分析规格](../Lap_Simulation_Spec.md)
