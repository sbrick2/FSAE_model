# SlipKinematics–BatteryPowerBaseline 开环 Plant 系统规格

> 状态：SlipKinematics–BatteryPowerBaseline 已实现并通过组件验证  
> 保真度：L1 解析/低保真基线  
> 集成阶段：OpenLoopPlantIntegration

## 1. 目标与证据

本工作包补齐 `Vehicle7DOF` 周围的轮胎运动学、简化轮胎、准静态轮荷、空气动力、轮端动力系统和电池总功率约束。当前验证证据包括坐标/符号合同、守恒关系、解析工况和自动回归；尚无 TTC、CFD、电机地图、电池台架或实车数据。因此模型必须可替换、可解析验证，不得声明为已标定高精度模型。

## 2. 组件边界

| ID | 模型 | 主要输入 | 主要输出 | 分类 |
|---|---|---|---|---|
| SlipKinematics | `WheelSlipKinematics` | 轮速、轮心轮胎坐标速度 | 滑移率、侧偏角、低速权重 | z→轮胎内部 |
| SimpleTire | `TireSimple` | 滑移率、侧偏角、外倾、轮荷、路面 μ | `FxW`、`FyW`、摩擦利用率 | Plant 内部 |
| LoadTransfer | `LoadTransferModel` | `Ax`、`Ay`、前后轴下压力 | 四轮法向载荷 | Plant 内部 |
| Aerodynamics | `AeroModel` | 车身速度、车身坐标风速、空气密度 | 阻力、侧向阻力、前后下压力 | w→Plant |
| PowertrainLimits | `PowertrainModel` | 电机转矩请求、轮速、电池功率缩放、使能 | 轮端转矩、转速、功率、限制状态 | u→Plant |
| BatteryPowerBaseline | `BatteryModel` | 未约束电功率请求 | 允许功率、功率缩放、电流、SOC | Plant 内部/z |

所有四轮量均为 `4x1 double`，顺序 `[FL, FR, RL, RR]`。组件采用连续时间继承兼容配置；采样率在 OpenLoopPlantIntegration 根据控制器与部署目标统一。

## 3. 运行范围与初始化

- 覆盖静止、低速正反转、加速、制动、稳态回转和功率饱和；
- 初始轮速和车身状态由 `Vehicle7DOFInitialState` 管理；
- 电池初始 SOC 由 `Battery.InitialSOC` 管理；
- 未知车辆、轮胎、气动和动力系统参数保持 `NaN`，默认禁止以未标定参数运行；
- 自动验证使用 `Simulink.SimulationInput` 临时覆盖合成参数。

## 4. 明确不包含

- TTC/MF6.2、联合滑移经验拟合和轮胎松弛长度动态；
- 悬架几何、侧倾中心、俯仰/侧倾动态及车轮离地事件处理；
- 气动高度图、偏航气动地图和主动空气动力学；
- 电机转矩—转速查表、温度降额、逆变器开关、电池等效电路；
- OpenLoopPlantIntegration 顶层 Plant 闭环和代数环最终处理。
