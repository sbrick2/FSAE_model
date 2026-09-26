# 标准结果信号与绘图参考

> 当前标准：圈速结果 Schema 1.2（100 个时序字段）；兼容 Schema 1.1（96）和 1.0（82）
> 更新日期：2026-09-23
> 依赖：[信号接口](../models/SignalInterfaces.md)、[坐标系与符号](../models/CoordinateSystem.md)、[圈速仿真规范](time_domain_closed_loop/Lap_Simulation_Spec.md)

## 1. 目的

本文件统一说明圈速仿真结果的字段命名、单位、维度、四轮顺序、生产来源和绘图分组。Schema 1.2 在 Schema 1.1 的 96 个时序字段基础上，追加 4 个预见制动诊断字段，共 100 个时序字段。旧 adaptive 结果仍按 Schema 1.1 的 14 字段 `DriverDebug` 读取；不含该顶层输出的既有结果继续按 Schema 1.0 解释。

ResultVisualization 开环结果字段仍保留在文末作为兼容参考。开环和闭环字段不得仅因名称相近而自动互换；缺失字段必须明确报告，不得静默填零。

## 2. 结果容器

闭环圈速结果由下列函数整理：

```matlab
result = collectLapSimulationResults(simulationOutput, scenario, cfg);
```

标准结果顶层结构为：

```text
result
├─ SchemaVersion
├─ Time
├─ Distance
├─ Config
├─ Track
├─ Vehicle
├─ Wheel
├─ Tire
├─ Powertrain
├─ Battery
├─ Driver
├─ DriverDebug                 # adaptive Schema 1.1/1.2
├─ Actuator
├─ Controller
├─ Sensor
├─ Metrics
└─ Meta
```

- `SchemaVersion`：带 18 字段 `DriverDebug` 的当前 adaptive 结果为 `"1.2"`；旧 14 字段 adaptive 结果为 `"1.1"`；无该输出的既有结果保持 `"1.0"`；
- `Time`：严格递增的 `Nx1` 秒制时间向量；
- `Distance`：由车辆全局位置派生的 `Nx1` 实际累计行驶距离，单位 m；
- `Config`：本次仿真配置快照，不属于时序字段；
- `Metrics`：整次仿真的标量指标，不计入下列 100/96/82 个 Schema 时序字段；
- `Meta`：信号来源、完整性、有效性和运行环境等元数据，不计入时序字段。

原始数据从 `yout` 和 `logsout` 中读取。既有闭环顶层输出为 `VehicleState`、`PowertrainState`、`Sensor`、`TrackReference`、`ControllerDebug`、`DriverCommand` 和 `ActuatorCommand`。两个 adaptive 顶层仅在末尾追加第 8 个 `DriverDebug` 输出；前 7 个输出的名称和顺序不变。

## 3. 通用规则

- 所有时序字段的第一维均为 `N = numel(result.Time)`；
- 四轮量为 `Nx4`，列顺序固定为 `[FL, FR, RL, RR]`；
- 内部角度统一使用 rad；
- 逻辑状态使用 `boolean`/`logical`，数值和派生量使用 `double`；
- 无量纲量的单位记为 `1`；
- 后处理允许从已有有效信号派生字段，但不得用零值伪装缺失信号；
- 缺失、非有限值和维度问题分别记录在 `Meta.MissingSignals`、`Meta.NonFiniteSignals` 和 `Meta.DimensionIssues`。

## 4. Schema 1.2 时序信号表（100 个字段）

Schema 1.2 的字段总数按分组统计为：`Track 11 + Vehicle 12 + Wheel 7 + Tire 3 + Powertrain 7 + Battery 4 + Driver 5 + DriverDebug 18 + Actuator 5 + Controller 10 + Sensor 18 = 100`。Schema 1.1 保持旧 14 字段 `DriverDebug` 的 96 个字段；Schema 1.0 保持不含 `DriverDebug` 的原 82 个字段。

### 4.1 `Track`（11）

| # | 结果字段 | 单位 | 维度 | 类型 | 生产来源/说明 |
|---:|---|---|---:|---|---|
| 1 | `Track.PathS` | m | `Nx1` | double | 当前采样点在单圈中心线上的弧长；优先读取 `TrackReference.PathS`，无效点由几何投影补充 |
| 2 | `Track.EventDistance` | m | `Nx1` | double | 对 `PathS` 解绕后的赛项累计中心线距离，闭合赛道跨圈连续 |
| 3 | `Track.LapIndex` | 1 | `Nx1` | double | 由赛项累计距离派生的圈序号，从 1 开始 |
| 4 | `Track.Progress` | 1 | `Nx1` | double | 赛项完成比例，范围 `[0,1]` |
| 5 | `Track.ReferenceSpeed` | m/s | `Nx1` | double | `TrackReference.ReferenceSpeed`；缺失时按投影弧长查询赛道速度参考 |
| 6 | `Track.Curvature` | 1/m | `Nx1` | double | 参考线曲率，左弯为正 |
| 7 | `Track.LateralError` | m | `Nx1` | double | 车辆质心相对最近中心线的横向误差，左侧为正 |
| 8 | `Track.BoundaryViolation` | m | `Nx1` | double | 车辆质心越过赛道边界的距离；界内为 0 |
| 9 | `Track.ReportedPathS` | m | `Nx1` | double | 保留的顶层参考/驾驶员报告弧长，用于与后处理投影的 `PathS` 对比 |
| 10 | `Track.VehicleEnvelopeClearance` | m | `Nx1` | double | 前后轴四个轮胎外缘点到赛道边界的最小净空；界内为正 |
| 11 | `Track.VehicleEnvelopeViolation` | m | `Nx1` | double | 车辆包络越界量，等于 `max(0, -VehicleEnvelopeClearance)` |

其中 `ReportedPathS`、`VehicleEnvelopeClearance` 和 `VehicleEnvelopeViolation` 是资格检查/后处理扩展字段；基础收集器初始化其余 8 个字段。

### 4.2 `Vehicle`（12）

| # | 结果字段 | 单位 | 维度 | 类型 | 生产来源/说明 |
|---:|---|---|---:|---|---|
| 1 | `Vehicle.X` | m | `Nx1` | double | `VehicleState.X`，车辆质心全局 X 坐标 |
| 2 | `Vehicle.Y` | m | `Nx1` | double | `VehicleState.Y`，车辆质心全局 Y 坐标 |
| 3 | `Vehicle.Psi` | rad | `Nx1` | double | `VehicleState.Psi`，全局航向，左转为正 |
| 4 | `Vehicle.Ux` | m/s | `Nx1` | double | `VehicleState.Ux`，车身纵向速度 |
| 5 | `Vehicle.Uy` | m/s | `Nx1` | double | `VehicleState.Uy`，车身横向速度 |
| 6 | `Vehicle.Speed` | m/s | `Nx1` | double | 由 `hypot(Ux, Uy)` 派生的车速大小 |
| 7 | `Vehicle.YawRate` | rad/s | `Nx1` | double | `VehicleState.YawRate`，左转为正 |
| 8 | `Vehicle.RollAngle` | rad | `Nx1` | double | `VehicleState.RollAngle`，左侧上抬为正 |
| 9 | `Vehicle.PitchAngle` | rad | `Nx1` | double | `VehicleState.PitchAngle`，车头下俯为正 |
| 10 | `Vehicle.Ax` | m/s² | `Nx1` | double | `VehicleState.Ax`，车身纵向加速度 |
| 11 | `Vehicle.Ay` | m/s² | `Nx1` | double | `VehicleState.Ay`，车身横向加速度 |
| 12 | `Vehicle.Az` | m/s² | `Nx1` | double | `VehicleState.Az`，车身垂向加速度 |

### 4.3 `Wheel`（7）

| # | 结果字段 | 单位 | 维度 | 类型 | 生产来源/说明 |
|---:|---|---|---:|---|---|
| 1 | `Wheel.Speed` | rad/s | `Nx4` | double | `VehicleState.WheelSpeed`，四轮机械角速度 |
| 2 | `Wheel.RPM` | rpm | `Nx4` | double | 由 `Wheel.Speed * 60 / (2*pi)` 派生 |
| 3 | `Wheel.SteerAngle` | rad | `Nx4` | double | `VehicleState.WheelSteerAngle`，左转为正 |
| 4 | `Wheel.NormalLoad` | N | `Nx4` | double | `VehicleState.NormalLoad`，向上支持力幅值 |
| 5 | `Wheel.SlipRatio` | 1 | `Nx4` | double | `VehicleState.SlipRatio`，正驱动、负制动 |
| 6 | `Wheel.SlipAngle` | rad | `Nx4` | double | `VehicleState.SlipAngle`，符号见坐标系文档 |
| 7 | `Wheel.CamberAngle` | rad | `Nx4` | double | `VehicleState.CamberAngle`，轮顶向车辆外侧为正 |

### 4.4 `Tire`（3）

| # | 结果字段 | 单位 | 维度 | 类型 | 生产来源/说明 |
|---:|---|---|---:|---|---|
| 1 | `Tire.FxWheel` | N | `Nx4` | double | `VehicleState.TireFx`，车轮局部坐标系纵向力 |
| 2 | `Tire.FyWheel` | N | `Nx4` | double | `VehicleState.TireFy`，车轮局部坐标系横向力 |
| 3 | `Tire.MuUtilization` | 1 | `Nx4` | double | `VehicleState.MuUtilization`，摩擦能力利用率，范围 `[0,1]` |

### 4.5 `Powertrain`（7）

| # | 结果字段 | 单位 | 维度 | 类型 | 生产来源/说明 |
|---:|---|---|---:|---|---|
| 1 | `Powertrain.MotorTorqueActual` | N·m | `Nx4` | double | `PowertrainState.MotorTorqueActual`，电机轴侧实际转矩，正驱动、负再生 |
| 2 | `Powertrain.MotorSpeed` | rad/s | `Nx4` | double | `PowertrainState.MotorSpeed`，电机机械角速度 |
| 3 | `Powertrain.MotorRPM` | rpm | `Nx4` | double | 由 `MotorSpeed * 60 / (2*pi)` 派生 |
| 4 | `Powertrain.MotorMechanicalPower` | W | `Nx4` | double | 电机机械功率；缺失时由 `MotorTorqueActual .* MotorSpeed` 派生 |
| 5 | `Powertrain.MotorLimitActive` | 1 | `Nx4` | boolean | `PowertrainState.MotorLimitActive`，各电机约束是否激活 |
| 6 | `Powertrain.TotalPowerLimitActive` | 1 | `Nx1` | boolean | `PowertrainState.TotalPowerLimitActive`，总功率约束是否激活 |
| 7 | `Powertrain.RegenLimitActive` | 1 | `Nx1` | boolean | `PowertrainState.RegenLimitActive`，再生约束是否激活 |

### 4.6 `Battery`（4）

| # | 结果字段 | 单位 | 维度 | 类型 | 生产来源/说明 |
|---:|---|---|---:|---|---|
| 1 | `Battery.Voltage` | V | `Nx1` | double | `PowertrainState.BatteryVoltage`，电池端电压 |
| 2 | `Battery.Current` | A | `Nx1` | double | `PowertrainState.BatteryCurrent`，放电为正 |
| 3 | `Battery.Power` | W | `Nx1` | double | 放电为正、充电为负；缺失时由 `Voltage .* Current` 派生 |
| 4 | `Battery.SOC` | 1 | `Nx1` | double | `PowertrainState.BatterySOC`，范围 `[0,1]` |

### 4.7 `Driver`（5）

| # | 结果字段 | 单位 | 维度 | 类型 | 生产来源/说明 |
|---:|---|---|---:|---|---|
| 1 | `Driver.SteeringWheelAngleRequest` | rad | `Nx1` | double | `DriverCommand.SteeringWheelAngleRequest`，方向盘请求 |
| 2 | `Driver.SteeringRackAngleRequest` | rad | `Nx1` | double | `DriverCommand.SteeringRackAngleRequest`，等效转向请求，左转为正 |
| 3 | `Driver.LongitudinalAccelerationRequest` | m/s² | `Nx1` | double | `DriverCommand.LongitudinalAccelerationRequest` |
| 4 | `Driver.DriveTorqueRequest` | N·m | `Nx1` | double | `DriverCommand.DriveTorqueRequest`，总驱动转矩请求 |
| 5 | `Driver.BrakePressureRequest` | Pa | `Nx1` | double | `DriverCommand.BrakePressureRequest`，非负制动压力请求 |

### 4.7.1 `DriverDebug`（18，Schema 1.2 adaptive）

| # | 结果字段 | 单位 | 维度 | 类型 | 生产来源/说明 |
|---:|---|---|---:|---|---|
| 1 | `DriverDebug.SafeSpeed` | m/s | `Nx1` | double | `DriverDebug.SafeSpeed`，当前安全速度上限 |
| 2 | `DriverDebug.LateralError` | m | `Nx1` | double | `DriverDebug.LateralError`，相对控制参考线横向误差 |
| 3 | `DriverDebug.HeadingError` | rad | `Nx1` | double | `DriverDebug.HeadingError`，相对控制参考航向误差 |
| 4 | `DriverDebug.BoundaryMargin` | m | `Nx1` | double | `DriverDebug.BoundaryMargin`，车辆包络最小净空 |
| 5 | `DriverDebug.TargetCurvature` | 1/m | `Nx1` | double | `DriverDebug.TargetCurvature` |
| 6 | `DriverDebug.LimitingCurvature` | 1/m | `Nx1` | double | `DriverDebug.LimitingCurvature` |
| 7 | `DriverDebug.LateralUtilization` | 1 | `Nx1` | double | `DriverDebug.LateralUtilization` |
| 8 | `DriverDebug.ProjectedX` | m | `Nx1` | double | `DriverDebug.ProjectedX`，控制参考投影全局 X |
| 9 | `DriverDebug.ProjectedY` | m | `Nx1` | double | `DriverDebug.ProjectedY`，控制参考投影全局 Y |
| 10 | `DriverDebug.ResetActive` | 1 | `Nx1` | boolean | `DriverDebug.ResetActive`，启动复位周期 |
| 11 | `DriverDebug.StatePreviousIndex` | 1 | `Nx1` | double | `DriverDebug.StatePreviousIndex` |
| 12 | `DriverDebug.StatePreviousReferenceIndex` | 1 | `Nx1` | double | `DriverDebug.StatePreviousReferenceIndex` |
| 13 | `DriverDebug.StatePreviousSteering` | rad | `Nx1` | double | `DriverDebug.StatePreviousSteering` |
| 14 | `DriverDebug.StateSpeedIntegrator` | m/s² | `Nx1` | double | `DriverDebug.StateSpeedIntegrator` |
| 15 | `DriverDebug.PreviewBrakingDeceleration` | m/s² | `Nx1` | double | 前方参考速度要求的最大减速度 |
| 16 | `DriverDebug.PreviewBrakingDistance` | m | `Nx1` | double | 限制预见制动的前方距离 |
| 17 | `DriverDebug.PreviewBrakingTargetSpeed` | m/s | `Nx1` | double | 限制点参考速度 |
| 18 | `DriverDebug.PreviewBrakingActive` | 1 | `Nx1` | boolean | 预见制动前馈是否生效 |

Schema 1.1 的旧结果只包含前 14 项；读取器不会为其伪造 4 个预见制动字段。

### 4.8 `Actuator`（5）

| # | 结果字段 | 单位 | 维度 | 类型 | 生产来源/说明 |
|---:|---|---|---:|---|---|
| 1 | `Actuator.SteeringRackAngleRequest` | rad | `Nx1` | double | `ActuatorCommand.SteeringRackAngleRequest`，最终转向请求 |
| 2 | `Actuator.MotorTorqueRequest` | N·m | `Nx4` | double | `ActuatorCommand.MotorTorqueRequest`，电机轴侧请求，正驱动、负再生 |
| 3 | `Actuator.FrictionBrakeTorqueRequest` | N·m | `Nx4` | double | `ActuatorCommand.FrictionBrakeTorqueRequest`，非负轮端摩擦制动幅值 |
| 4 | `Actuator.RegenTorqueRequest` | N·m | `Nx4` | double | `ActuatorCommand.RegenTorqueRequest`，非负再生需求幅值 |
| 5 | `Actuator.TotalPowerRequest` | W | `Nx1` | double | `ActuatorCommand.TotalPowerRequest`，正放电、负回收 |

### 4.9 `Controller`（10）

| # | 结果字段 | 单位 | 维度 | 类型 | 生产来源/说明 |
|---:|---|---|---:|---|---|
| 1 | `Controller.ReferenceYawRate` | rad/s | `Nx1` | double | `ControllerDebug.ReferenceYawRate`，参考横摆角速度 |
| 2 | `Controller.YawRateError` | rad/s | `Nx1` | double | `ControllerDebug.YawRateError`，横摆角速度误差 |
| 3 | `Controller.DesiredLongitudinalForce` | N | `Nx1` | double | `ControllerDebug.DesiredLongitudinalForce`，期望纵向合力 |
| 4 | `Controller.DesiredYawMoment` | N·m | `Nx1` | double | `ControllerDebug.DesiredYawMoment`，期望横摆力矩 |
| 5 | `Controller.AllocatedYawMoment` | N·m | `Nx1` | double | `ControllerDebug.AllocatedYawMoment`，分配后的横摆力矩 |
| 6 | `Controller.TVActive` | 1 | `Nx1` | boolean | 横摆控制门控激活；实际 TV 交付需比较 `DesiredYawMoment` 与 `AllocatedYawMoment` |
| 7 | `Controller.TCActive` | 1 | `Nx1` | 至少一个轮端 TC 滞环激活 |
| 8 | `Controller.RegenActive` | 1 | `Nx1` | boolean | `ControllerDebug.RegenActive`，再生制动激活状态 |
| 9 | `Controller.EnergyManagementActive` | 1 | `Nx1` | boolean | `ControllerDebug.EnergyManagementActive`，能量管理激活状态 |
| 10 | `Controller.ControllerSaturated` | 1 | `Nx1` | boolean | `ControllerDebug.ControllerSaturated`，控制器限幅状态 |

### 4.10 `Sensor`（18）

| # | 结果字段 | 单位 | 维度 | 类型 | 生产来源/说明 |
|---:|---|---|---:|---|---|
| 1 | `Sensor.PositionX` | m | `Nx1` | double | `Sensor.PositionX`，定位测量/估计 |
| 2 | `Sensor.PositionY` | m | `Nx1` | double | `Sensor.PositionY`，定位测量/估计 |
| 3 | `Sensor.Heading` | rad | `Nx1` | double | `Sensor.Heading`，航向测量/估计 |
| 4 | `Sensor.LongitudinalSpeed` | m/s | `Nx1` | double | `Sensor.LongitudinalSpeed`，估计纵向车速 |
| 5 | `Sensor.LateralSpeed` | m/s | `Nx1` | double | `Sensor.LateralSpeed`，估计横向车速 |
| 6 | `Sensor.YawRate` | rad/s | `Nx1` | double | `Sensor.YawRate`，IMU 横摆角速度 |
| 7 | `Sensor.AccelX` | m/s² | `Nx1` | double | `Sensor.AccelX`，IMU 纵向加速度 |
| 8 | `Sensor.AccelY` | m/s² | `Nx1` | double | `Sensor.AccelY`，IMU 横向加速度 |
| 9 | `Sensor.WheelSpeed` | rad/s | `Nx4` | double | `Sensor.WheelSpeed`，四轮轮速测量 |
| 10 | `Sensor.SteeringRackAngle` | rad | `Nx1` | double | `Sensor.SteeringRackAngle`，转向测量 |
| 11 | `Sensor.MotorSpeed` | rad/s | `Nx4` | double | `Sensor.MotorSpeed`，四电机转速测量 |
| 12 | `Sensor.MotorTorqueEstimate` | N·m | `Nx4` | double | `Sensor.MotorTorqueEstimate`，四电机转矩估计 |
| 13 | `Sensor.BatteryVoltage` | V | `Nx1` | double | `Sensor.BatteryVoltage`，电池电压测量 |
| 14 | `Sensor.BatteryCurrent` | A | `Nx1` | double | `Sensor.BatteryCurrent`，放电为正 |
| 15 | `Sensor.PoseValid` | 1 | `Nx1` | boolean | `Sensor.PoseValid`，位置和航向是否有效 |
| 16 | `Sensor.IMUValid` | 1 | `Nx1` | boolean | `Sensor.IMUValid`，IMU 测量是否有效 |
| 17 | `Sensor.WheelSpeedValid` | 1 | `Nx4` | boolean | `Sensor.WheelSpeedValid`，各轮轮速测量是否有效 |
| 18 | `Sensor.PowertrainValid` | 1 | `Nx1` | boolean | `Sensor.PowertrainValid`，动力系统测量是否有效 |

## 5. 绘图分组

闭环圈速结果建议至少包含以下图组：

1. 赛道和全局轨迹：`Vehicle.X-Y`、赛道边界和车辆包络越界点；
2. 赛项进度：`Track.EventDistance`、`Track.Progress`、`Track.LapIndex`；
3. 路径跟踪：`Track.LateralError`、`Track.BoundaryViolation`、车辆包络净空；
4. 参考和实际速度：`Track.ReferenceSpeed`、`Vehicle.Speed`；
5. 车身状态：`Ux`、`Uy`、`YawRate`、`RollAngle`、`PitchAngle`、`Ax`、`Ay`、`Az`；
6. 四轮运动学：轮速、转角、轮荷、滑移率、侧偏角和外倾角；
7. 轮胎：`FxWheel`、`FyWheel` 和 `MuUtilization`；
8. 驾驶员和执行器：转向、加速、驱动、摩擦制动、再生和功率请求；Schema 1.2 adaptive 还可绘制安全速度、边界净空、投影、复位与预见制动状态；
9. 控制器：参考/实际横摆响应、力和力矩分配、控制激活与限幅状态；
10. 动力系统和电池：电机转矩、转速、机械功率、电池电压、电流、功率和 SOC；
11. 传感器：真值/测量对比及各有效位。

四轮颜色固定为 FL 蓝、FR 橙、RL 绿、RR 红；图例必须使用 `FL/FR/RL/RR`。角度内部保留 rad，若显示为 deg，标题或纵轴必须明确标注转换。

## 6. 派生指标和元数据

`Metrics` 可从上述时序字段计算圈速/赛项时间、最大纵横向加速度、最大摩擦利用率、电池能量、边界违规、车辆包络违规、横向误差 RMS 和控制器限幅比例等标量指标。派生指标必须保留计算公式或实现入口，并明确积分或统计的时间范围。

`Meta` 至少保存 Schema 版本、场景、赛项、数据集名称、顶层输出列表、缺失信号、非有限信号、维度问题、信号来源、MATLAB 版本和结果有效性。Schema 1.1/1.2 adaptive 记录八组顶层输出；Schema 1.0 和非 adaptive 模型仍为七组。资格检查可以继续增加完赛状态、车辆包络和输出路径等元数据。

## 7. ResultVisualization 开环兼容字段

旧版 `collectOpenLoopResults` 可能包含下列闭环 Schema 1.0 未冻结的组件级诊断字段：

| 分组 | 兼容字段 |
|---|---|
| `Wheel` | `VelocityXWheel`、`VelocityYWheel`、`LowSpeedBlend` |
| `Tire` | `SaturationScale` |
| `Aero` | `ForceXBody`、`ForceYBody`、`DownforceFront`、`DownforceRear`、`RelativeAirSpeed`、`DynamicPressure` |
| `Powertrain` | `MotorTorqueRequest`、`WheelAppliedTorque`、`ElectricalPowerUnconstrained`、`ElectricalPowerActual`、`SpeedLimitScale`、`TorqueSaturationResidual` |
| `Battery` | `AllowedElectricalPower`、`PowerScale`、`DrivePowerClipped`、`RegenPowerClipped` |

这些字段继续用于开环组件验证，不计入本文件第 4 节的 100/96/82 个闭环 Schema 时序字段。若未来将其纳入闭环标准，应同时更新结果初始化器、信号映射、完整性检查、绘图入口和 Schema 版本。

## 8. 完整性标准

- Schema 1.2 的 100 个字段、Schema 1.1 的 96 个字段或 Schema 1.0 的 82 个字段，其名称、单位和维度均与实际保存结果一致；
- `Time` 严格递增，所有非空时序字段的第一维等于 `numel(Time)`；
- 四轮字段统一为 `Nx4`，列顺序固定为 `[FL, FR, RL, RR]`；
- 出现信号缺失、时间不单调、非有限值或维度错误时，结果标记为无效并给出原因；
- 图和机器可读摘要共同导出，且不覆盖原始仿真输出。
