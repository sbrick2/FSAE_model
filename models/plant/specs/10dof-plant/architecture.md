# Vehicle10DOF 10DOF 动态轮荷 Plant 架构规格

> 关联系统规格：[system.md](system.md)

## 1. 模型分层

| 层级 | 文件 | 职责 |
|---|---|---|
| 顶层 | `models/top/FSAE_Vehicle10DOF_ClosedLoop.slx` | 复用 TorqueVectoring 驾驶员、控制器和场景接口，引用 Vehicle10DOF Plant |
| Plant | `models/plant/VehiclePlant10DOF.slx` | 轮胎、空气动力、动力系统、制动和 10DOF 刚体集成 |
| 刚体 | `models/plant/Vehicle10DOF.slx` | 保留 7DOF 平面/轮速方程并增加 3DOF 簧载体动力学 |
| 算法 | `scripts/vehicle_10dof/vehicle10DOFSuspensionSFunction.m` | 弹簧、非线性阻尼、防倾杆、动态轮荷和状态导数 |
| 数据 | `data/VehicleData.sldd` | `Vehicle`、`Vehicle10DOFSuspension`、`Vehicle10DOFInitialState` |

旧 `LoadTransferModel` 在 Vehicle10DOF Plant 中仅保留为对比路径，其输出终止，不再驱动轮胎法向载荷。动态 `NormalLoad`、`RollAngle`、`PitchAngle`、`VerticalPosition` 和 `Az` 由 `Vehicle10DOF` 写入 `VehicleStateBus`。

## 2. 信号流

```text
Driver/Controller -> actuator commands -> Powertrain/Brake
                                           |
Environment -> Aero -----------------------+----> Vehicle10DOF planar inputs
Environment -> RoadHeight/RoadVelocity ----------> SuspensionDynamics
Vehicle10DOF -> wheel kinematics -> TireModel -> tire forces -> Vehicle10DOF
Vehicle10DOF -> NormalLoad/Camber ----------------^          -> VehicleStateBus
```

`Vehicle10DOF` 在新增的前/后下压力、路面高度和路面速度根输入处使用 Memory 隔离，以保持模型引用状态边界；Plant 不再重复增加下压力延迟。模型保持“尽量减少人为代数环”配置，以沿用 7DOF Plant 中轮速—轮胎—车身的模型引用调度方式。最终顶层更新图为 0 错误、0 警告。

## 3. 端口合同

| 端口组 | 维度 | 单位 | 方向/约定 |
|---|---:|---|---|
| `Ax`, `Ay` | 1 | m/s² | 车身坐标，正值分别向前/向左 |
| `DownforceFront/Rear` | 1 | N | 正值向下 |
| `RoadHeight` | 4x1 | m | 正值向上 |
| `RoadVelocity` | 4x1 | m/s | 正值向上 |
| `NormalLoad` | 4x1 | N | 非负向上幅值 |
| `Roll/PitchAngle` | 1 | rad | 左侧抬高/车头下沉为正 |
| `SuspensionDeflection` | 4x1 | m | 路面相对车身角点向上为正压缩 |
| `DamperVelocity` | 4x1 | m/s | 正值压缩，负值回弹 |

## 4. 参数替换边界

`Vehicle10DOFSuspension` 将轮上刚度、运动比、静态轮荷、前后附加侧倾刚度及压缩/回弹阻尼曲线集中为单一结构。替换阻尼器时只更新曲线和设置元数据，不修改 Simulink 拓扑；替换线性弹簧或防倾杆时保持 SI 单位和 `[FL,FR,RL,RR]` 顺序。

## 5. 失效模式与保护

| 失效模式 | 当前检测/保护 | 后续动作 |
|---|---|---|
| 阻尼速度断点无序 | S-function 参数断言 | 拒绝编译 |
| 惯量或刚度缺失 | 有限参数检查/编译失败 | 导入 CAD 或试验数据 |
| 轮荷为负 | `Fz` 下限和验证断言 | 14DOF 接触模型替代 |
| 载荷不守恒 | 动态残差断言 | 检查符号、下压力和钳位状态 |
| K&C 数据缺失 | 外倾输出固定零并记录限制 | 导入数值曲线 |
| 模型引用代码生成 | Normal 模式限定 | 提供可代码生成实现/TLC |
