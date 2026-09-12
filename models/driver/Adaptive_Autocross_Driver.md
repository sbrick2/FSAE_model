# GGV 约束的 Adaptive Autocross 驾驶员

> 状态：闭合赛道 standard 资格圈通过；开放赛道完赛与零越界验证通过<br>
> 更新日期：2026-08-23

## 1. 模型边界

`AdaptiveAutocrossDriver.slx` 直接接收 `PathTrackingTrackDataBus` 和 `SensorBus`，输出既有
`DriverCommandBus`。独立顶层 `FSAE_AdaptiveAutocross_7DOF.slx` 将其连接到
TorqueVectoring 控制器、传感器和 7DOF Plant，不改变原 TorqueVectoring/Vehicle10DOF 闭环模型。

与旧实现不同，adaptive 模式会读取 `TrackData.ReferenceSpeed`。该信号由原始
GGV、驾驶员纵横向请求能力、尖弯曲率保护及前/后向传播生成，是不可超越的能力
上限；驾驶员仍在线计算投影、包络净空和控制误差，不把参考速度当作开环油门表。
没有有效参考速度时，几何前视规划器仅作为失效降级路径。

## 2. 控制逻辑

`AdaptiveDriverCore` 固定以 `AADControlSampleTime=0.01 s` 离散执行。显式离散采样
保证 persistent 投影状态、上一转角和 PI 积分器不会随变步长求解器的主步次数
变化。

每个周期执行：

1. 在上一投影点附近对物理中心线线段做连续投影，得到物理站点和包络基准；同一
   站点可读取软赛车线坐标、航向和曲率；
2. 物理中心线与软赛车线按边界净空、曲率和 `ReferencePathMaximumBlend` 连续混合，
   再做航向前馈 + Stanley 型反馈；默认最大混合为 `0`，转角和转角速率均有限幅；
3. 将前后轴四个轮胎外缘点投影到赛道，取最小车辆包络净空；储备区内连续降低
   GGV 速度上限，紧急区禁止加速，包络到达边界时请求最大制动；
4. 读取当前位置的 GGV 参考速度上限，并由相邻 `v^2` 空间梯度生成仅正向的纵向
   前馈；减速保持闭环，避免负前馈与横向能力限制叠加后在发卡弯过制动；
5. 根据实测横向加速度缩小纵向能力，用 PI + 抗饱和生成加减速请求；超速时清除
   正积分记忆，避免积分器继续加速；
6. 保持 TV/TC/再生制动接口；当前 adaptive 基线启用再生制动。

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
| `HighCurvatureThreshold/SafetyFactor` | `0.30 1/m / 1.50` |
| `BoundaryReserve/EmergencyBoundaryMargin` | `0.50/0.25 m` |
| `MaximumSteeringAngle/Rate` | `0.50 rad / 6 rad/s` |
| `GGVConstraintScale` | `1.0` |
| `MaximumGGVLapTimeRatio` | `1.20` |
| 赛车线预留/最大偏移/最大偏移率 | `0.60 m / 0.40 m / 0.02 m/m` |
| 赛车线软惩罚阈值/缓冲/权重 | `0.25 1/m / 15 m / 0.020` |
| 赛车线曲率混合起止/最大权重 | `0.15/0.30 1/m / 0.0` |
| 边界预瞄距离 | `6.0 m` |

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

2026-08-23 standard 结果：

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
