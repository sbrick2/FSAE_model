# FSAE 信号接口合同

> 状态：TireModel 扩展版 + 独立自适应驾驶员
> 更新日期：2026-08-13
> 依赖：[坐标系与符号](CoordinateSystem.md)、[系统架构](Architecture.md)

## 1. 通用规则

- Bus 元素名、维度、类型和单位以本文件为准；
- 四轮向量统一为 `4x1`，顺序 `[FL, FR, RL, RR]`；
- ProjectFoundation 连续 Plant 和离散控制接口使用 `double`，模式使用 `uint8`，开关/有效位使用 `boolean`；
- 内部角度均为 `rad`；
- 每个 Bus 都在 `VehicleData.sldd` 中定义对应 `Simulink.Bus` 和 `Default<BusName>`；
- 默认结构仅用于占位模型和初始化，不代表真实车辆状态；
- Bus 变更必须更新本文件、Data Dictionary、所有引用模型和回归测试。

## 2. 模型端口合同

### 2.1 `DriverModel.slx`

| 端口 | 方向 | 类型 | 速率 | 分类 | 说明 |
|---|---|---|---:|---|---|
| `TrackReference` | In | `Bus: TrackReferenceBus` | 10 ms 暂定 | 输入 | 当前参考点和速度目标 |
| `Sensor` | In | `Bus: SensorBus` | 5 ms 暂定 | `y` | 驾驶员可用车辆测量 |
| `DriverCommand` | Out | `Bus: DriverCommandBus` | 10 ms 暂定 | 输出 | 转向、加速和制动需求 |

`AdaptiveAutocrossDriver.slx` 使用同一个输出合同，但输入改为：

| 端口 | 方向 | 类型 | 速率 | 分类 | 说明 |
|---|---|---|---:|---|---|
| `TrackData` | In | `Bus: PathTrackingTrackDataBus` | 10 ms | 输入 | 完整中心线几何、曲率和左右半宽；`ReferenceSpeed` 元素明确不读取 |
| `Sensor` | In | `Bus: SensorBus` | 5 ms 暂定 | `y` | 位姿、速度和 IMU 测量 |
| `DriverCommand` | Out | `Bus: DriverCommandBus` | 10 ms | 输出 | 与现有 TorqueVectoring 控制器兼容的转向、加减速及功能开关 |

该模型只在 `FSAE_AdaptiveAutocross_7DOF.slx` 中使用，不改变原
`DriverModel/TorqueVectoringPathTrackingDriver` 的 `TrackReferenceBus` 合同。

### 2.2 `VehicleController.slx`

| 端口 | 方向 | 类型 | 速率 | 分类 | 说明 |
|---|---|---|---:|---|---|
| `DriverCommand` | In | `Bus: DriverCommandBus` | 10 ms 暂定 | 输入 | 驾驶员需求 |
| `Sensor` | In | `Bus: SensorBus` | 5 ms 暂定 | `y` | 控制器可用测量 |
| `PowertrainState` | In | `Bus: PowertrainStateBus` | 1–5 ms 暂定 | `y` | 执行器和电池约束状态 |
| `ActuatorCommand` | Out | `Bus: ActuatorCommandBus` | 1–5 ms 暂定 | `u` | 最终执行器命令 |
| `ControllerDebug` | Out | `Bus: ControllerDebugBus` | 5 ms 暂定 | `z` | 调试量，不进入 Plant |

`TorqueVectoringVehicleController` 仅在驾驶员请求再生、执行器使能且
`PowertrainState.BatterySOC < Battery.SOCUpperLimit` 时向分配器使能再生。
达到 SOC 上限时，分配器按“再生禁用”处理负纵向力，由
`FrictionBrakeTorqueRequest` 接管，避免 Plant 裁剪再生后出现制动力缺口。
该门控不改变任何 Bus 字段、类型或顺序。

### 2.3 `VehiclePlant.slx`

| 端口 | 方向 | 类型 | 速率 | 分类 | 说明 |
|---|---|---|---:|---|---|
| `ActuatorCommand` | In | `Bus: ActuatorCommandBus` | 1–5 ms 暂定 | `u` | 转向、电机和制动命令 |
| `Environment` | In | `Bus: EnvironmentBus` | 连续/场景速率 | `w` | 路面、风和环境 |
| `VehicleState` | Out | `Bus: VehicleStateBus` | 连续 | `z` | 车辆真值，仅验证和传感器模型使用 |
| `PowertrainState` | Out | `Bus: PowertrainStateBus` | 1 ms 暂定 | `y/z` | 执行器测量与约束状态 |

### 2.4 `SensorModel.slx`

| 端口 | 方向 | 类型 | 速率 | 分类 | 说明 |
|---|---|---|---:|---|---|
| `VehicleState` | In | `Bus: VehicleStateBus` | 连续 | `z` | Plant 真值输入 |
| `PowertrainState` | In | `Bus: PowertrainStateBus` | 1 ms 暂定 | `z` | 动力系统真值输入 |
| `Sensor` | Out | `Bus: SensorBus` | 5 ms 暂定 | `y` | 控制器和驾驶员可用测量 |

## 3. `TrackReferenceBus`

| 元素 | 维度 | 类型 | 单位 | 默认值 | 说明 |
|---|---:|---|---|---:|---|
| `ReferenceX` | 1 | double | m | 0 | 当前参考点全局 X |
| `ReferenceY` | 1 | double | m | 0 | 当前参考点全局 Y |
| `ReferenceHeading` | 1 | double | rad | 0 | 参考航向 |
| `ReferenceCurvature` | 1 | double | 1/m | 0 | 左弯为正 |
| `ReferenceSpeed` | 1 | double | m/s | 0 | 目标车速 |
| `PathS` | 1 | double | m | 0 | 沿赛道弧长 |
| `LeftHalfWidth` | 1 | double | m | 0 | 参考线左侧可用宽度 |
| `RightHalfWidth` | 1 | double | m | 0 | 参考线右侧可用宽度 |
| `Valid` | 1 | boolean | 1 | false | 当前参考是否有效 |

## 4. `DriverCommandBus`

| 元素 | 维度 | 类型 | 单位 | 默认值 | 说明 |
|---|---:|---|---|---:|---|
| `SteeringWheelAngleRequest` | 1 | double | rad | 0 | 方向盘请求 |
| `SteeringRackAngleRequest` | 1 | double | rad | 0 | 转向齿条/等效前轮请求，左转为正 |
| `LongitudinalAccelerationRequest` | 1 | double | m/s^2 | 0 | 纵向加速度请求 |
| `DriveTorqueRequest` | 1 | double | N*m | 0 | 总驱动转矩请求 |
| `BrakePressureRequest` | 1 | double | Pa | 0 | 非负制动压力请求 |
| `EnableTV` | 1 | boolean | 1 | false | 扭矩矢量使能 |
| `EnableTC` | 1 | boolean | 1 | false | 牵引力控制使能 |
| `EnableRegen` | 1 | boolean | 1 | false | 再生制动使能 |
| `EnableEnergyManagement` | 1 | boolean | 1 | false | 能量管理使能 |
| `DriverMode` | 1 | uint8 | 1 | 0 | 0 Disabled, 1 OpenLoop, 2 PathTracking, 3 Replay |

## 5. `ActuatorCommandBus`

| 元素 | 维度 | 类型 | 单位 | 默认值 | 说明 |
|---|---:|---|---|---:|---|
| `SteeringRackAngleRequest` | 1 | double | rad | 0 | 最终转向请求，左转为正 |
| `MotorTorqueRequest` | 4x1 | double | N*m | zeros | 正驱动、负再生，电机侧或轮侧必须由参数注明；ProjectFoundation 约定为电机轴侧 |
| `FrictionBrakeTorqueRequest` | 4x1 | double | N*m | zeros | 非负轮端制动幅值 |
| `RegenTorqueRequest` | 4x1 | double | N*m | zeros | 非负再生需求幅值，只作分配信息，不与最终电机转矩重复叠加 |
| `TotalPowerRequest` | 1 | double | W | 0 | 正放电、负回收 |
| `ControllerMode` | 1 | uint8 | 1 | 0 | 0 Disabled, 1 Standby, 2 Active, 3 Degraded, 4 Fault |
| `EnableActuation` | 1 | boolean | 1 | false | 总执行器使能 |

当前液压制动为前后固定比例系统。Plant 将四轮
`FrictionBrakeTorqueRequest` 的非负总量解释为总轮端制动需求，再按字典中的
四轮最大制动转矩向量分配；因此不支持独立改变单轮液压制动比例。实际作用
方向由轮速符号决定，正转和反转时均与轮速相反。

## 6. `VehicleStateBus`

`VehicleStateBus` 是 `z` 真值总线，不得作为最终控制器输入。

| 元素 | 维度 | 类型 | 单位 | 默认值 | 说明 |
|---|---:|---|---|---:|---|
| `X` | 1 | double | m | 0 | 全局位置 |
| `Y` | 1 | double | m | 0 | 全局位置 |
| `Psi` | 1 | double | rad | 0 | 全局航向 |
| `Ux` | 1 | double | m/s | 0 | 车身纵向速度 |
| `Uy` | 1 | double | m/s | 0 | 车身横向速度 |
| `YawRate` | 1 | double | rad/s | 0 | 左转为正 |
| `RollAngle` | 1 | double | rad | 0 | 左侧上抬为正 |
| `PitchAngle` | 1 | double | rad | 0 | 车头下俯为正 |
| `VerticalPosition` | 1 | double | m | 0 | 质心向上位移 |
| `Ax` | 1 | double | m/s^2 | 0 | 车身纵向加速度 |
| `Ay` | 1 | double | m/s^2 | 0 | 车身横向加速度 |
| `Az` | 1 | double | m/s^2 | 0 | 车身垂向加速度 |
| `WheelSpeed` | 4x1 | double | rad/s | zeros | 正向滚动 |
| `WheelSteerAngle` | 4x1 | double | rad | zeros | 左转为正 |
| `NormalLoad` | 4x1 | double | N | zeros | 向上支持力幅值 |
| `SlipRatio` | 4x1 | double | 1 | zeros | 正驱动、负制动 |
| `SlipAngle` | 4x1 | double | rad | zeros | 见坐标系文档 |
| `CamberAngle` | 4x1 | double | rad | zeros | 轮顶向车辆外侧为正 |
| `TireFx` | 4x1 | double | N | zeros | 车轮局部 +x |
| `TireFy` | 4x1 | double | N | zeros | 车轮局部 +y |
| `MuUtilization` | 4x1 | double | 1 | zeros | 轮胎摩擦能力利用率，裁剪到 `[0,1]` |

## 7. `WheelStateBus`

该 Bus 用于轮胎子模型和测试，不要求在 ProjectFoundation 顶层单独连接。

| 元素 | 维度 | 类型 | 单位 | 默认值 |
|---|---:|---|---|---:|
| `WheelSpeed` | 4x1 | double | rad/s | zeros |
| `WheelSteerAngle` | 4x1 | double | rad | zeros |
| `NormalLoad` | 4x1 | double | N | zeros |
| `SlipRatio` | 4x1 | double | 1 | zeros |
| `SlipAngle` | 4x1 | double | rad | zeros |
| `CamberAngle` | 4x1 | double | rad | zeros |
| `TireFx` | 4x1 | double | N | zeros |
| `TireFy` | 4x1 | double | N | zeros |
| `TireFz` | 4x1 | double | N | zeros |
| `MuUtilization` | 4x1 | double | 1 | zeros |

## 8. `PowertrainStateBus`

| 元素 | 维度 | 类型 | 单位 | 默认值 | 说明 |
|---|---:|---|---|---:|---|
| `MotorTorqueActual` | 4x1 | double | N*m | zeros | 电机轴侧实际转矩 |
| `MotorSpeed` | 4x1 | double | rad/s | zeros | 电机机械角速度 |
| `MotorMechanicalPower` | 4x1 | double | W | zeros | 正输出、负回收 |
| `BatteryVoltage` | 1 | double | V | 0 | 端电压 |
| `BatteryCurrent` | 1 | double | A | 0 | 放电为正 |
| `BatteryPower` | 1 | double | W | 0 | 放电为正、充电为负 |
| `BatterySOC` | 1 | double | 1 | 0 | 范围 `[0,1]` |
| `MotorLimitActive` | 4x1 | boolean | 1 | false | 单电机约束激活 |
| `TotalPowerLimitActive` | 1 | boolean | 1 | false | 总功率约束激活 |
| `RegenLimitActive` | 1 | boolean | 1 | false | 再生约束激活 |

## 9. `EnvironmentBus`

| 元素 | 维度 | 类型 | 单位 | 默认值 | 说明 |
|---|---:|---|---|---:|---|
| `RoadGripScale` | 4x1 | double | 1 | ones | 相对轮胎参考路面的抓地缩放；先同时缩放 `Fx`、`Fy` |
| `RoadMuLimit` | 4x1 | double | 1 | inf | 可选绝对摩擦系数上限；有限时约束 `hypot(Fx,Fy)<=RoadMuLimit*NormalLoad`，`Inf` 表示不增加该限幅 |
| `RoadGrade` | 1 | double | rad | 0 | 上坡为正 |
| `RoadBank` | 1 | double | rad | 依坐标右手定则 |
| `RoadHeight` | 4x1 | double | m | zeros | 四轮接地点高度 |
| `AirDensity` | 1 | double | kg/m^3 | 0 | 安全占位；`EnvironmentValid=false` 时不得用于性能结论 |
| `WindVelocityGlobal` | 3x1 | double | m/s | zeros | 空气相对地面的全局速度 |
| `AmbientTemperature` | 1 | double | degC | 0 | 安全占位；有效性由 `EnvironmentValid` 指示 |
| `Gravity` | 1 | double | m/s^2 | 9.80665 | 标准重力常数，用于占位和测试 |
| `EnableRoadDisturbance` | 1 | boolean | 1 | false | 路面扰动使能 |
| `EnvironmentValid` | 1 | boolean | 1 | false | 环境物理参数是否已由场景或有效数据提供 |

## 10. `SensorBus`

`SensorBus` 只包含可由计划传感器或估计器获得的量，不包含轮胎真值力。

| 元素 | 维度 | 类型 | 单位 | 默认值 | 说明 |
|---|---:|---|---|---:|---|
| `PositionX` | 1 | double | m | 0 | GPS/定位估计 |
| `PositionY` | 1 | double | m | 0 | GPS/定位估计 |
| `Heading` | 1 | double | rad | 0 | 航向测量/估计 |
| `LongitudinalSpeed` | 1 | double | m/s | 0 | 估计纵向车速 |
| `LateralSpeed` | 1 | double | m/s | 0 | 估计横向车速 |
| `YawRate` | 1 | double | rad/s | 0 | IMU 横摆角速度 |
| `AccelX` | 1 | double | m/s^2 | 0 | IMU 纵向加速度 |
| `AccelY` | 1 | double | m/s^2 | 0 | IMU 横向加速度 |
| `WheelSpeed` | 4x1 | double | rad/s | zeros | 四轮轮速 |
| `SteeringRackAngle` | 1 | double | rad | 0 | 转向测量 |
| `MotorSpeed` | 4x1 | double | rad/s | zeros | 电机转速测量 |
| `MotorTorqueEstimate` | 4x1 | double | N*m | zeros | 电机转矩估计 |
| `BatteryVoltage` | 1 | double | V | 0 | 电池电压 |
| `BatteryCurrent` | 1 | double | A | 0 | 放电为正 |
| `PoseValid` | 1 | boolean | 1 | false | 位置/航向有效 |
| `IMUValid` | 1 | boolean | 1 | false | IMU 有效 |
| `WheelSpeedValid` | 4x1 | boolean | 1 | false | 各轮轮速有效 |
| `PowertrainValid` | 1 | boolean | 1 | false | 动力系统测量有效 |

## 11. `ControllerDebugBus`

| 元素 | 维度 | 类型 | 单位 | 默认值 |
|---|---:|---|---|---:|
| `ReferenceYawRate` | 1 | double | rad/s | 0 |
| `YawRateError` | 1 | double | rad/s | 0 |
| `DesiredLongitudinalForce` | 1 | double | N | 0 |
| `DesiredYawMoment` | 1 | double | N*m | 0 |
| `AllocatedYawMoment` | 1 | double | N*m | 0 |
| `WheelTorqueUnconstrained` | 4x1 | double | N*m | zeros |
| `TVActive` | 1 | boolean | 1 | false |
| `TCActive` | 1 | boolean | 1 | false |
| `RegenActive` | 1 | boolean | 1 | false |
| `EnergyManagementActive` | 1 | boolean | 1 | false |
| `ControllerSaturated` | 1 | boolean | 1 | false |

## 12. `ScoringBus`

该 Bus 在 ProjectFoundation 中定义但不强制接入顶层。

| 元素 | 维度 | 类型 | 单位 | 默认值 |
|---|---:|---|---|---:|
| `LapTime` | 1 | double | s | 0 |
| `EventTime` | 1 | double | s | 0 |
| `LateralErrorRMS` | 1 | double | m | 0 |
| `BoundaryViolationCount` | 1 | uint32 | 1 | 0 |
| `MaxLongitudinalAcceleration` | 1 | double | m/s^2 | 0 |
| `MaxLateralAcceleration` | 1 | double | m/s^2 | 0 |
| `BatteryEnergyUsed` | 1 | double | J | 0 |
| `ConstraintActiveTime` | 1 | double | s | 0 |
| `SimulationValid` | 1 | boolean | 1 | false |

## 13. 默认结构

Data Dictionary 必须包含：

```text
DefaultTrackReferenceBus
DefaultDriverCommandBus
DefaultActuatorCommandBus
DefaultVehicleStateBus
DefaultWheelStateBus
DefaultPowertrainStateBus
DefaultEnvironmentBus
DefaultSensorBus
DefaultControllerDebugBus
DefaultScoringBus
```

默认结构通过 `Simulink.Bus.createMATLABStruct` 生成，并保持有限、类型正确的安全占位值；物理未知量的 `NaN` 只保存在参数记录中。不得手工维护与 Bus 不一致的结构。

## 14. ProjectFoundation 接口验收

- [x] 所有 Bus 可从 `VehicleData.sldd` 解析；
- [x] 默认结构与 Bus 字段、维度和类型一致；
- [x] 四轮向量均为 `4x1` 且顺序一致；
- [x] Plant 的 `VehicleState` 未直接连接控制器；
- [x] `SensorBus` 不含轮胎真值力或真实摩擦系数；
- [x] `MotorTorqueRequest` 和再生/摩擦制动符号无重复叠加；
- [x] 所有占位速率均标记为暂定；
- [x] 顶层与四个引用模型更新图无类型或维度错误。

## 15. VehicleDynamicsCore `Vehicle7DOF` 内部物理接口

VehicleDynamicsCore 新增的组件接口不改变 ProjectFoundation 顶层 `VehiclePlant` Bus 合同。完整端口、单位、符号和方程见 [VehicleDynamicsCore 七自由度车辆动力学核心](plant/Vehicle7DOF.md)。

- 输入：四轮轮胎坐标力/回正力矩、四轮转角、四轮轮端施加转矩、车身外力与外部横摆力矩；
- 输出：七个动力学状态、全局位姿、状态导数、车身加速度和轮心速度；
- 四轮向量顺序继续使用 `[FL, FR, RL, RR]`；
- 轮胎、轮荷、驱动和气动模型只能通过这些物理量接入，不得绕过坐标变换直接改写积分状态。

## 16. SlipKinematics–BatteryPowerBaseline 内部组件接口

SlipKinematics–BatteryPowerBaseline 不改变 ProjectFoundation 顶层 Bus 合同；组件端口在 OpenLoopPlantIntegration 通过既有 Bus 字段映射。

| 组件 | 输入 | 输出 |
|---|---|---|
| `WheelSlipKinematics` | `WheelSpeed`、`WheelVelocityXWheel`、`WheelVelocityYWheel` | `SlipRatio`、`SlipAngle`、`RegularizedSpeed`、`LowSpeedBlend` |
| `TireSimple` | `SlipRatio`、`SlipAngle`、`CamberAngle`、`NormalLoad`、`RoadGripScale`、`RoadMuLimit` | `TireForceXWheel`、`TireForceYWheel`、`MuUtilization`、`SaturationScale` |
| `LoadTransferModel` | `Ax`、`Ay`、`DownforceFront`、`DownforceRear` | `NormalLoad`、前/后轴载荷、总载荷 |
| `AeroModel` | `Ux`、`Uy`、车身坐标风速、`AirDensity` | 车身气动力、前/后轴下压力、相对气流速度、动压 |
| `PowertrainModel` | 四轮电机转矩请求、轮速、驱动/再生功率缩放、使能 | 轮端转矩、电机状态、限功率前后电功率和约束诊断 |
| `BatteryModel` | 未约束电功率请求 | 允许电功率、缩放、电流、SOC 和裁剪量 |

所有四轮端口均为 `4x1 double` 且顺序为 `[FL, FR, RL, RR]`。详细字段与单位见 [ResultVisualization 结果信号参考](../simulation/Result_Signal_Reference.md)。

## 17. TireModel `TireModel` 接口

`TireSimple`、`TireMF62` 和 `TireTTCMap` 共用冻结接口，不改变 `VehiclePlant` 顶层 Bus：

| 端口 | 方向 | 维度 | 单位 | 说明 |
|---|---|---:|---|---|
| `SlipRatio` | In | 4x1 | 1 | 正驱动、负制动 |
| `SlipAngle` | In | 4x1 | rad | 符号见坐标系文档 |
| `CamberAngle` | In | 4x1 | rad | 轮顶向车辆外侧为正 |
| `NormalLoad` | In | 4x1 | N | 非负支持力幅值 |
| `RoadGripScale` | In | 4x1 | 1 | 相对参考轮胎能力的非负缩放 |
| `RoadMuLimit` | In | 4x1 | 1 | 可选绝对摩擦系数上限；`Inf` 关闭额外限幅 |
| `TireForceXWheel` | Out | 4x1 | N | 车轮局部 `+x` |
| `TireForceYWheel` | Out | 4x1 | N | 车轮局部 `+y` |
| `MuUtilization` | Out | 4x1 | 1 | 轮胎模型内在能力与可选绝对上限中的较高利用率，裁剪到 `[0,1]` |
| `SaturationScale` | Out | 4x1 | 1 | 联合滑移缩减与可选绝对限幅中的最小缩放 |

当前接口没有运行时胎压输入；TireModel 使用所选 profile 的标称胎压。`TireTTCMap` 的有效域只覆盖目标胎零纵向滑移的侧向/外倾数据，非零滑移率会被标记为域外并裁剪，不能替代纵向模型。

## 18. Vehicle10DOF `Vehicle10DOF` 扩展接口

`Vehicle10DOF` 保留 `Vehicle7DOF` 前 8 个输入和前 14 个输出的顺序与语义，
并增加下列悬架接口。四轮顺序继续使用 `[FL, FR, RL, RR]`。

| 端口 | 方向 | 维度 | 单位 | 说明 |
|---|---|---:|---|---|
| `DownforceFront/Rear` | In | 1 | N | 正值向下 |
| `RoadHeight` | In | 4x1 | m | 正值向上 |
| `RoadVelocity` | In | 4x1 | m/s | 正值向上 |
| `RollAngle` | Out | 1 | rad | 左侧车身抬高为正 |
| `PitchAngle` | Out | 1 | rad | 车头下沉为正 |
| `VerticalPosition` | Out | 1 | m | 质心向上为正 |
| `Az` | Out | 1 | m/s² | 质心向上为正 |
| `NormalLoad` | Out | 4x1 | N | 非负向上支持力幅值 |
| `CamberAngle` | Out | 4x1 | rad | 当前无 K&C 数值，输出零 |
| `SuspensionDeflection` | Out | 4x1 | m | 正值为压缩 |
| `DamperVelocity` | Out | 4x1 | m/s | 正压缩、负回弹 |
| `SuspensionForce` | Out | 4x1 | N | 相对静载的轮端增量力 |
| `RollRate/PitchRate` | Out | 1 | rad/s | 与角度正方向一致 |
| `VerticalVelocity` | Out | 1 | m/s | 向上为正 |

完整动力学、参数来源和限制见
[Vehicle10DOF 10DOF Plant 系统规格](plant/specs/10dof-plant/system.md)。

## 19. UnifiedControl `TorqueAllocator` 内部接口

UnifiedControl 保持 `DriverCommandBus`、`SensorBus`、`PowertrainStateBus`、
`ActuatorCommandBus` 和 `ControllerDebugBus` 的元素、类型和顺序不变。
`TorqueAllocator.slx` 是控制器内部引用模型，使用 `[FL, FR, RL, RR]` 轮序：

| 端口组 | 方向 | 维度/类型 | 单位 | 说明 |
|---|---|---|---|---|
| `DesiredLongitudinalForce` | In | scalar double | N | 正驱动、负制动 |
| `DesiredYawMoment` | In | scalar double | N*m | 正值产生左转横摆力矩 |
| `LongitudinalSpeed`, `AccelX`, `AccelY` | In | scalar double | m/s, m/s² | 控制器可用测量 |
| `WheelSpeed`, `MotorSpeed` | In | 4x1 double | rad/s | 车轮与电机测量 |
| `BatterySOC` | In | scalar double | 1 | 动力系统 SOC 估计 |
| `WheelSpeedValid` | In | 4x1 boolean | 1 | 单轮测量有效性 |
| `PowertrainValid` | In | scalar boolean | 1 | 动力系统测量有效性 |
| `EnableTV/TC/Regen/EnergyManagement/Actuation` | In | scalar boolean | 1 | 功能与安全门控 |
| `MotorTorqueRequest` | Out | 4x1 double | N*m | 电机轴端请求；负值为再生 |
| `FrictionBrakeTorqueRequest` | Out | 4x1 double | N*m | 非负轮端摩擦制动幅值 |
| `RegenTorqueRequest` | Out | 4x1 double | N*m | 非负再生诊断量，不由 Plant 再次叠加 |
| `TotalPowerRequest`, `AllocatedYawMoment` | Out | scalar double | W, N*m | 电功率与电机分配横摆力矩 |
| `WheelTorqueUnconstrained` | Out | 4x1 double | N*m | 约束前电机等效目标 |
| `ControllerSaturated`, `TCActive`, `RegenActive`, `EnergyManagementActive` | Out | scalar boolean | 1 | 约束与功能状态 |
| `ControllerMode` | Out | scalar uint8 | 1 | `0` 禁用、`2` 激活、`3` 降级 |
| `ConstraintFlags` | Out | scalar uint16 | 1 | UnifiedControl 本地诊断位掩码 |

UnifiedControl 控制器只从测量 Bus 和共享参数估算滑移、轮荷与轮胎余量，不读取
`VehicleStateBus` 的真实轮胎力或真实轮荷。完整合同见
[UnifiedControl 系统规格](controller/specs/unified-control-allocation/system.md)。
