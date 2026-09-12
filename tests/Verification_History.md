# 阶段验证历史

本文件集中保存 ProjectFoundation–QuasiStatic 阶段验收结论。TorqueVectoring–UnifiedControl 的详细设计、测试入口和当前限制分别由对应控制器文档与测试目录维护。

| 阶段 | 验证日期 | 结论 | 主要限制 |
|---|---|---|---|
| ProjectFoundation | 2026-07-24 | 通过 | 仅证明工程骨架与接口连通 |
| OpenLoopPlant | 2026-07-24 | 通过 | 使用合成车辆参数 |
| TireModel | 2026-07-24 | 通过 | 纵向与联合滑移使用代理数据 |
| PathTracking | 2026-07-26 | 通过 | 控制参数和部分赛道数据为临时基线 |
| QuasiStatic | 2026-07-26 | 通过 | 合成参数结果不代表实车性能 |

## ProjectFoundation 阶段验证反馈

> 验证日期：2026-07-24  
> 环境：MATLAB / Simulink R2026a Update 4  
> 结论：通过

### 验证结果

| 检查 | 结果 |
|---|---|
| MATLAB Project health checks | 12/12 PASS |
| Data Dictionary 与 Bus | 10/10 Bus 及默认结构可解析 |
| 模型结构 | 5/5 healthy |
| 引用与顶层模型仿真 | 5/5 到达停止时间 |
| 顶层输出 | 4 个有限、类型正确的 Bus |
| Gherkin 回归 | 1 个场景、2 个断言通过 |
| MATLAB Code Analyzer | 4 个脚本，0 issues |

### 反馈

CoordinateContract–FoundationReview 已完成，工程骨架、接口、数据字典、模型引用和初始化流程满足阶段要求。OpenLoopPlant 集成后的非回归仍通过。

本结论只证明工程与接口连通。生产字典中的未知物理参数仍为显式 `NaN` 占位，合成参数仿真不能用于车辆性能结论。

## OpenLoopPlant 阶段验证反馈

> 验证日期：2026-07-24  
> 范围：VehicleDynamicsCore–OpenLoopPlantIntegration
> 结论：通过

### 验证结果

| 检查 | 结果 |
|---|---|
| ProjectFoundation 非回归 | Project 12/12；5 个顶层模型通过 |
| VehicleDynamicsCore 解析回归 | 4/4 |
| 坐标变换 Gherkin | 2/2 场景，16/16 断言 |
| SlipKinematics–BatteryPowerBaseline 组件回归 | 6/6 |
| 电池功率限制 Gherkin | 3/3 场景，9/9 断言 |
| OpenLoopScenarios 开环回归 | 8/8 |
| 结果完整性 | 8/8 `Complete=true`，缺失信号 0 |
| 生产根模型结构审计 | 问题 0 |

### 反馈

七自由度车辆动力学、轮胎运动学、简化轮胎、准静态轮荷、气动、动力系统、电池和 `VehiclePlant` 集成满足 OpenLoopPlant 阶段要求。

验证使用合成参数，只证明结构、接口、解析关系和数值边界。真实车辆参数导入后必须重新验证，才能形成性能结论。

## TireModel 阶段验证反馈

> 验证日期：2026-07-24  
> 目标胎：Hoosier 43075 16x7.5-10 R20，7 英寸轮辋  
> 结论：通过，但纵向与联合滑移使用代理数据

### 验证结果

| 检查 | 结果 |
|---|---|
| TTC SI MAT 数据审计 | 97 个文件，5,335,202 个样本 |
| 拟合数据清洗 | 接受 374,724；拒绝 898 |
| 目标胎侧向验证 | Fy R² 0.9884，RMSE 164.386 N |
| 代理纵向验证 | Fx R² 0.9836，RMSE 169.784 N |
| 代理联合滑移验证 | Fy R² 0.9465，RMSE 123.397 N |
| 数值、边界与模式回归 | 8/8 |
| `VehiclePlant` 高保真模式集成 | 2/2 |
| ProjectFoundation/OpenLoopPlant 非回归 | 全部通过 |

### 反馈

TireModel 数据导入、清洗、目标胎侧向/外倾拟合、联合滑移等效模型、TTC Map 和三模式 `TireModel` 已完成。

43075 缺少直接 drive/brake 数据。纵向与联合滑移采用同 R20 配方的 43100 异尺寸代理，不得描述为目标胎实测标定；补充目标尺寸数据后应重新验证。

## PathTracking 阶段验证反馈

> 验证日期：2026-07-26  
> 环境：MATLAB / Simulink R2026a Update 4  
> 结论：通过，6/6 工作包完成

### 验证结果

| 工作包 | 状态 | 结果 |
|---|---|---|
| TrackContract 赛道合同与投影 | 通过 | 加速、八字、Autocross、Endurance、闭合跨界和八字分支共 9/9 |
| OpenLoopDriverScenarios 开环驾驶员工况 | 通过 | 阶跃、正弦、定转角、加速和制动 5/5，可配置且可重复 |
| LateralPathTracking 横向路径跟踪 | 通过 | 八字 RMS/最大误差 0.564/0.700 m；转向角 0.437 rad，速率 4.000 rad/s |
| LongitudinalSpeedControl 纵向速度控制 | 通过 | `20→5→15 m/s` 仅 2 次预期切换；驱动与制动/再生同时激活 0 次；饱和后最终误差 0.65 m/s |
| ClosedLoopScenarios 赛项场景 | 通过 | 75 m 加速、规则尺寸八字、1 km Autocross 和参数化 Endurance 均可重复生成 |
| ClosedLoopIntegration 闭环集成 | 通过 | 四个生产模型结构 healthy；四类闭环场景零越界、无非有限值 |

### 闭环基线

| 场景 | 结果 |
|---|---|
| 75 m 加速 | 5.190 s 穿越计时线；最大横向偏差 0 |
| 八字 | RMS 0.564 m；最大 0.700 m；越界 0 |
| 1 km Autocross | 完成 1 圈；RMS 0.299 m；最大 0.786 m；越界 0 |
| 两圈 Endurance | 完成 2 圈；RMS 0.301 m；最大 0.806 m；越界 0 |

八字在 `MaxStep/RelTol = 0.005/1e-4` 与 `0.02/1e-2` 两组扰动下均零越界、无非有限值，最大误差均为 0.700 m。转向请求始终满足 0.50 rad 角度限幅和 4.00 rad/s 速率限幅，纵向加速度请求始终位于 `[-8, 8] m/s²`。

### 完成性修复

- 数据字典改用可移植名称，并补齐 `PathTrackingTrackDataBus` 和 Stateflow 参数声明。
- 修正所有根 Outport 连接，增加 `DriverCommand` 与 `ActuatorCommand` 顶层观测输出。
- 采用兼容引用 Plant 的变步长求解器，并用两处 Memory 消除反馈代数环。
- 场景通过 `createPathTrackingSimulationInput` 显式写入顶层模型工作区，避免默认赛道遮蔽请求场景。
- 图像中心线增加支路规避、闭合平滑、曲率限速和纵向速度包络。

### 适用范围

验收使用合成车辆、动力系统和环境参数。Autocross 宽度、图像比例、目标速度与控制增益仍是明确标注的临时基线；本结论证明结构、接口、控制边界和数值闭环可用，不代表真实赛车性能。

验证在 MATLAB 内存中执行，未保留测试脚本、测试文件夹或结果文件；阶段验收记录现集中保存在本文件中。

## QuasiStatic 阶段验证反馈

> 验证日期：2026-07-26  
> 环境：MATLAB / Simulink R2026a  
> 结论：QuasiStatic 求解链与 PathTracking 实际时域接口验证通过；车辆参数仍为合成验证值，不代表实车性能。

### 通过项

| 工作包 | 结果 |
|---|---|
| SteadyStateBalance 稳态平衡与约束 | LP 稳态点通过；纵向力、横向力和横摆力矩残差满足验证阈值；轮荷总和守恒；前后轴相对质心力臂与参数合同一致 |
| SpeedDependentGGV 速度相关 GGV | 小规模速度网格生成通过；同时输出加速/制动边界和激活约束标签 |
| LapSpeedPlanning 前后向速度规划 | 闭合与开放赛道通过；开放赛道首末速度和距离守恒；左/右弯分别使用正/负横向边界 |
| ParameterSweepConfiguration 参数扫描配置 | 扫描路径、单位、来源和组合值可追踪；不改物理公式 |
| BatchAnalysisReporting 批量运行 | 2 组合扫描均成功；结果保留参数单位/来源、MATLAB 版本、Git 提交/脏状态、工况、Variant、求解器和失败原因 |
| TimeDomainCrossValidation 交叉验证 | 合成横向越界样例可被拒绝；PathTracking 加速场景 835 个实际时域样本通过 QuasiStatic GGV 纵/横向边界比较 |
| 回归与集成 | `QuasiStaticRegressionTest` 8/8；PathTracking 顶层成功编译并运行 9 个引用模型，缺失信号 0、非有限信号 0 |

### 数据边界

默认使用 TireModel 的 `Round9_43075_R20_Rim7` profile。43075 目标胎的 TTC Map 只有零纵向滑移的横向/外倾数据，因此 GGV 默认使用同一 profile 中明确标记为代理的 43100 R20 纵向/联合滑移 MF 配方。真实车辆参数、气动、动力系统和制动参数仍需用户数据替换。

### PathTracking → QuasiStatic 实际时域验证

PathTracking 控制器已将不受 Model Reference 支持的 `Vehicle`、`Tire`、`Powertrain` 结构参数替换为五个显式标量参数，并由 `createPathTrackingSimulationInput` 从共享参数记录写入引用模型工作区。使用 10 m/s 目标速度的 PathTracking 加速场景验证：

- 835 个有效样本；
- 缺失信号 0，非有限信号 0；
- QuasiStatic 交叉验证容差 `0.5 m/s^2`；
- 最大纵向上界超差 `0.0216113 m/s^2`；
- 纵向下界和正/负横向超差均为 0；
- 交叉验证结果：通过。

这证明 PathTracking/QuasiStatic 接口和边界检查可运行，不代表合成车辆与真实赛车具有相同性能。

运行入口：

```matlab
addpath(genpath(pwd));
report = runQuasiStaticVerification();
results = runtests(["tests/quasi_static/QuasiStaticRegressionTest.m", ...
    "tests/quasi_static/QuasiStaticRobustnessTest.m"]);
assertSuccess(results);
integrationReport = runQuasiStaticTimeDomainCrossValidation();
```
