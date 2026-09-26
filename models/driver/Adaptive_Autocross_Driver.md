# GGV 约束的 Adaptive Autocross 驾驶员

> 状态：预见制动、统一 TV/TC 控制链及 7DOF/10DOF 短时 MIL 已通过；新全圈资格待复跑<br>
> 更新日期：2026-09-23

## 1. 模型边界

`AdaptiveAutocrossDriver.slx` 的外部合同固定为两个输入和两个输出：直接接收
`PathTrackingTrackDataBus` 与 `SensorBus`，输出既有 `DriverCommandBus` 以及只读诊断
`DriverDebugBus`。独立顶层 `FSAE_AdaptiveAutocross_7DOF.slx` 和
`FSAE_AdaptiveAutocross_10DOF.slx` 将其连接到 `UnifiedControlVehicleController`、传感器以及
对应的 7DOF/10DOF Plant，不改变原 reference-speed 闭环模型。

两个 adaptive 顶层把 `DriverDebug` 追加为第 8 个顶层输出；前 7 个输出的名称、顺序和
合同保持不变。`DriverDebugBus` 仅用于检查驾驶员的速度约束、投影和离散状态，不进入
控制器或 Plant，不能作为性能结论的替代数据。

与旧实现不同，adaptive 模式会读取 `TrackData.ReferenceSpeed`。该信号由原始
GGV、驾驶员纵横向请求能力、尖弯曲率保护及前/后向传播生成，是不可超越的能力
上限；驾驶员仍在线计算投影、包络净空和控制误差，不把参考速度当作开环油门表。
没有有效参考速度时，几何前视规划器仅作为失效降级路径。

## 2. 控制逻辑

`AdaptiveDriverCore` 固定以 `AADControlSampleTime=0.01 s` 离散执行。显式离散采样
保证 persistent 投影状态、上一转角和 PI 积分器不会随变步长求解器的主步次数
变化。

### 2.1 启动复位与状态生命周期

外部端口不暴露复位输入。模型内部以 `Constant(false)` 驱动 `Unit Delay`
`StartOfRunReset`，其初始条件为 `true`、采样时间为 `AADControlSampleTime`；因此每次
仿真启动时恰有一个控制采样周期向核心发出复位，之后恒为 `false`。核心在该周期重建
投影索引、参考索引、上一转角和速度 PI 积分器，避免隐藏 persistent 状态跨运行残留。
`DriverDebug.ResetActive` 记录该复位周期，其余 `State*` 字段记录复位后的当前离散状态。

该机制是可观测性和可重复性的结构改动；本身不代表圈速已提升，也尚未以新的 standard
全圈资格结果替代本文件第 5 节的历史结果。

每个周期执行：

1. 在上一投影点附近对物理中心线线段做连续投影，得到物理站点和包络基准；同一
   站点可读取软赛车线坐标、航向和曲率；
2. 物理中心线与软赛车线按边界净空、曲率和 `ReferencePathMaximumBlend` 连续混合，
   再做航向前馈 + Stanley 型反馈；默认最大混合为 `0`，转角和转角速率均有限幅；
3. 将前后轴四个轮胎外缘点投影到赛道，取最小车辆包络净空；储备区内连续降低
   GGV 速度上限，紧急区禁止加速，包络到达边界时请求最大制动；
4. 读取当前位置的 GGV 参考速度上限，并由相邻 `v^2` 空间梯度生成仅正向的纵向
   加速前馈；另在默认 `45 m` 窗口内扫描前方参考速度，按
   `(v_now^2-v_target^2)/(2*distance)` 提前生成制动上限。零填充、尚未规划的
   `ReferenceSpeed` 不会被误判为停车目标，仍使用几何降级路径；
5. 根据实测横向加速度缩小纵向能力，用 PI + 抗饱和生成加减速请求；超速时清除
   正积分记忆，避免积分器继续加速；
6. 通过可配置的 `EnableTV`、`EnableTC` 和再生开关驱动统一控制器；默认启用 TV、
   TC 与再生制动。

尖弯保护从 `|curvature|=0.9*threshold` 开始用 smoothstep 渐入，达到阈值后使用
完整保护系数，避免单个曲率采样点造成刚性速度跳变。

速度规划前，`planAdaptiveRacingLine` 用宽度约束凸 QP 最小化路径二阶差分。偏移
受车辆半包络、`0.60 m` 预留、`0.02 m/m` 斜率和 `0.40 m` 绝对上限约束。
在线参考线不再把高曲率区的偏移上下界强制设为零，而是在阈值前 5% 连续增加
二次惩罚，并在前后 `15 m` 平滑衰减；本次软线高曲率硬锁点数为 `0`。

赛车线 QP、曲率邻域和平滑差分均按 `Track.IsClosed` 选择拓扑。闭合赛道使用周期
首尾连接；开放赛道使用非周期一阶/二阶差分，缓冲区和预瞄在终点截断，不会把终点
与起点相连，也不会从终点读取起点航向或速度。

为避免软线降低尖弯曲率峰值后改变已验证的制动相位，规划器同时计算一条仅供离线
GGV 使用的高曲率保护线。保护线不会进入在线转向控制，因此不产生弯道刚性或步内
优化开销；软线及其预测包络已写入 `PathTrackingTrackDataBus`，可用
`ReferencePathMaximumBlend` 从 `0` 开始逐步标定。

## 3. 当前配置

| 配置 | 当前值 |
|---|---:|
| `LateralAccelerationLimit` | `13.5 m/s^2` |
| `MaximumAcceleration/Deceleration` | `16/8 m/s^2` |
| `PlanningDeceleration` | `6.0 m/s^2` |
| `MaximumSpeed` | `32 m/s` |
| `SpeedPreviewDistance/Step` | `190/0.75 m` |
| `SpeedKp/SpeedKi/基础/退出前馈增益` | `4.0/0.20/0.60/0.60` |
| 预见制动启用/窗口/前馈增益/减速度比例 | `true / 45 m / 0.65 / 0.90` |
| `HighCurvatureThreshold/SafetyFactor` | `0.30 1/m / 1.50` |
| `BoundaryReserve/EmergencyBoundaryMargin` | `0.50/0.25 m` |
| `MaximumSteeringAngle/Rate` | `0.50 rad / 6 rad/s` |
| `GGVConstraintScale` | `1.0` |
| `MaximumGGVLapTimeRatio` | `1.20` |
| 赛车线预留/最大偏移/最大偏移率 | `0.60 m / 0.40 m / 0.02 m/m` |
| 赛车线软惩罚阈值/缓冲/权重 | `0.25 1/m / 15 m / 0.020` |
| 赛车线曲率混合起止/最大权重 | `0.15/0.30 1/m / 0.0` |
| 边界预瞄距离 | `6.0 m` |
| `EnableTV/EnableTC` | `true/true` |

车辆轴距、质心位置和轮距来自 `VehicleData.sldd`。轮胎包络宽度使用
`TireSectionWidth=0.1905 m`；更换轮胎后必须同步更新。

## 4. 验收

`validateAdaptiveAutocrossResult` 要求：

- 完成目标圈且圈时有限；
- 使用 GGV 速度上限；
- 四轮包络与质心均零越界；
- 原始 GGV 时间及其比值有限；仅闭合赛道要求闭环圈时不超过原始 GGV 的 `1.20`
  倍，开放赛道的时间比只作为诊断指标；
- 使用 `standard` 求解器、逐点输出，最大输出步长不超过配置上限。

原始 GGV 基准在施加驾驶员限值和尖弯保护前计算，调参不得通过降低 GGV 能力或
放宽 `1.20` 门槛制造通过结果。

## 5. 当前验证状态

2026-08-23 standard 历史结果：

`results/time_domain_closed_loop/autocross/20260823_162040_adaptive_ggv_120pct/`

| 指标 | 结果 |
|---|---:|
| 原始 GGV 圈时 / 120% 门槛 | `63.761665 / 76.513998 s` |
| 软参考线长度 / 最大偏移 / 硬锁点 | `998.583191 m / 0.0836 m / 0` |
| GGV 保护线长度 / 规划圈时 | `998.056191 m / 75.018089 s` |
| standard 闭环圈时 / GGV 比 | `76.513132 s / 1.199986` |
| 四轮包络最小净空 | `0.089730 m` |
| 最大包络越界 / 点数 | `0 m / 0` |
| standard 求解器 | PASS |
| 全圈资格 | PASS |

资格圈仅比门槛低约 `0.00087 s`，因此更改车辆、轮胎、赛道离散或求解器后必须
重新运行 standard 验收，不能沿用该通过结论。

2026-09-23 完成控制链改造后的验证边界如下：

- 预见制动/开关函数测试 `11/11` 通过，结果 Schema 测试 `6/6` 通过；
- YawController 完整编译 MIL `5/5` 场景、`15/15` 断言通过；
- TorqueAllocator 完整编译 MIL `8/8` 场景、`45/45` 断言通过；
- 规划参考速度下 7DOF/10DOF 各运行 `2 s` standard 无保存仿真，均加速到约
  `9.53 m/s`，结果有效且实际分配出非零横摆力矩；自然工况未触发 TC；
- 上述短时结果不替代 2026-08-23 的全圈资格。由于控制器、预见制动和结果合同均已
  变化，必须复跑完整 standard 圈速矩阵后才能声称圈速改善或继续沿用资格结论。

同日开放加速赛道验证：75 m 在 `4.285 s` 完成，四轮包络最小净空
`0.802617 m`、最大越界 `0 m`，standard 求解器和开放赛道资格均 PASS。其 GGV
时间比 `1.217` 仅用于诊断，不套用闭合 Autocross 的 `1.20` 圈速门槛；fast 无图形
入口在下一次 `1 s` 完成检查点停止，`StoppedAtFinish=true`，不再积分到 90 s 上限。

## 6. 运行

在 `simulation/time_domain_closed_loop/simulation/runLapSimulation.m` 中使用：

```matlab
cfg.Track.Name = "autocross";
cfg.Vehicle.DynamicsModel = "7DOF";
cfg.Driver.Model = "adaptive_autocross";
cfg.Simulation.SolverProfile = "standard";
```

当前赛道是由 FSEC 示意图归一化的 1 km、3 m 宽代理赛道；结果用于算法开发，
不代表官方赛道或实车成绩。四点包络覆盖轮胎外缘，不包含车身前后悬和翼片。
