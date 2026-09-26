# UnifiedControl 统一控制分配设计与验证

> 日期：2026-09-23
> 状态：功能 MIL 与 adaptive 7DOF/10DOF 短时集成完成；全圈、实车标定、SIL/PIL 待完成

## 1. 交付范围

UnifiedControl 在不修改共享 Bus 的前提下，新增独立的四轮统一控制分配链：

```text
AdaptiveAutocrossDriver / UnifiedControlPathTrackingDriver
        │ DriverCommandBus + feature switches
        v
UnifiedControlVehicleController
  ├─ YawController             -> DesiredYawMoment
  └─ TorqueAllocator
       ├─ wheel load/slip/capacity estimation
       ├─ projected Fx/Mz allocation
       └─ motor/regen/friction mixing
        │ ActuatorCommandBus
        v
VehiclePlant10DOF
```

生产资产为 `TorqueAllocator.slx`、`UnifiedControlVehicleController.slx`、
`UnifiedControlPathTrackingDriver.slx` 和 `FSAE_UnifiedControl_ClosedLoop.slx`。此外，
`FSAE_AdaptiveAutocross_7DOF.slx` 与 `FSAE_AdaptiveAutocross_10DOF.slx` 现已引用同一
UnifiedControl 控制器；历史 `FSAE_TorqueVectoring_ClosedLoop.slx` 保持原控制器，测试
包装模型不被生产模型引用。

## 2. 分配目标与约束

四轮纵向力向量 `f=[FL,FR,RL,RR]'` 同时追踪：

```text
Fx = [1 1 1 1] f
Mz = [-tf/2 tf/2 -tr/2 tr/2] f
```

固定 24 次投影加权最小二乘在同一计算中施加单电机转矩/转速、转矩变化率、
估算轮胎余量、单轮有效性、TC 滞环与削减、驱动功率和能量预算。当前权重为
横摆 `4`、纵向力 `1`、正则 `1e-4`，因此不可同时满足时优先保留横摆权威。
算法不依赖 Optimization Toolbox，不读取 Plant 的真实轮胎力或真实轮荷。

负纵向力先使用可用再生；SOC、TC、转矩变化率或 `10 kW` 再生功率限制导致的
缺口由非负摩擦制动补足。当前 Plant 只支持固定比例液压制动，因此制动横摆仍主要
由电机侧承担，这一边界已在饱和标志和限制说明中显式记录。

## 3. 状态、模式与安全行为

- Stateflow 持有四轮上一拍电机转矩和 TC 滞环状态；估算、投影、混合保持独立函数。
- TC 暂定阈值为 `0.12` 接通、`0.08` 断开、`0.25` 完全削减。
- 正常请求和 TC 解除后的扭矩恢复单步变化不超过
  `1050 N*m/s * 0.005 s = 5.25 N*m`；TC 安全降扭可立即越过该下限。
- 模式 `0` 为安全禁用并立即输出零；模式 `2` 为正常激活；模式 `3` 为测量降级。
- 无效轮速通道禁止正驱动；无效动力系统反馈进入降级模式；安全禁用重置内部状态。

## 4. 参数与适用边界

`scripts/unified_control/installUnifiedControlData.m` 将共享物理参数和 `UnifiedControlDesign` 写入
`VehicleData.sldd`。当前 `mu=1.0`、TC 阈值、`1050 N*m/s` 转矩变化率、
90% 能量功率比例及横摆/纵向权重都是设计暂定值。Vehicle10DOF 惯量、阻尼器实车旋钮和
K&C 外倾曲线也尚未关闭，因此 UnifiedControl 结果只能用于功能、约束和数值回归，不可直接
解释为真实赛车性能。

adaptive 入口通过 `Simulink.SimulationInput` 从本次车辆参数快照显式注入 UnifiedControl
运行变量，并将 `UnifiedControlMaximumAcceleration/Deceleration` 与驾驶员配置对齐；
字典值仍作为模型默认值。这样 7DOF/10DOF 切换不会依赖上一次 Base Workspace 状态。

## 5. 验证结果

| 层级 | 结果 | 量化证据 |
|---|---|---|
| 纯函数回归 | PASS，11 类工况 | 标称四轮差 `0`；正横摆输出 `100.000 N*m`；30% 单轮超滑同拍全削减到 `0 N*m`，其余轮保留驱动力 |
| 功率与恢复 | PASS | 驱动峰值 `66.316 kW`；再生峰值 `10.000 kW`；最大恢复步长 `5.25 N*m` |
| Gherkin 组件 MIL | 8/8 场景、45/45 断言 | 完整编译；覆盖标称、横摆、TC、驱动功率/能量、多约束、再生/摩擦、测量降级和安全禁用 |
| 模型结构 | PASS | 四个生产模型结构健康；分配器 Stateflow lint 健康；分配器与控制器 20 ms 仿真通过 |
| 10DOF 顶层 MIL | 3/3 场景 | `Skidpad_Baseline`、`Skidpad_AllFeatures`、`Skidpad_NoRegen` 各运行 0.5 s，信号有限且约束满足 |
| Adaptive 顶层短时 MIL | 7DOF/10DOF 各 2 s PASS | standard、无保存；末速约 `9.47 m/s`，峰值约 `9.53 m/s`；均实际交付非零横摆力矩，结果 Schema 1.2 有效且无缺失信号 |

顶层三场景中最大电机请求为 `8.098 N*m`，最大驱动功率约 `4.884 kW`，最小
轮荷 `476.713 N`，最大电机转矩单步变化 `5.25 N*m`。场景生成器另提供 7 个
可复现功能组合；本次关闭门禁使用上述 3 个代表性顶层组合，更多单/多约束组合由
8 个组件场景覆盖。

2 s adaptive 工况未自然触发 TC，因此顶层短时测试只证明开关、路由、TV 交付和信号链；
TC 介入与同拍全削减结论来自专门的超滑函数/组件场景。未运行新的完整 autocross 圈，
不能据此声称圈速提高或全圈资格通过。

## 6. 复现

```matlab
project = initProject();
addpath(fullfile(project.RootFolder, "scripts", "unified_control"));
addpath(fullfile(project.RootFolder, "tests", "unified_control"));
installUnifiedControlData();

functionReport = runUnifiedControlFunctionVerification();
scenarioMatrix = createUnifiedControlScenarioMatrix();
report = runUnifiedControlVerification( ...
    StopTime=0.5, UseFastRestart=true, SaveSummary=true);
```

顶层摘要保存到 `tests/unified_control/results/UnifiedControlVerificationSummary.mat`。持久组件测试定义位于
`tests/unified_control/TorqueAllocator.feature`，必须使用完整编译模式
(`draft_mode=false`)；`TorqueAllocatorTestWrapper.slx` 仅供测试。

## 7. 后续工作

1. 用实车或台架数据重新标定附着估计、TC 滞环、电机转矩变化率和能量预算。
2. 明确液压系统是否支持独立轮制动，再决定是否将制动横摆纳入完整分配自由度。
3. Deployment 引入传感器噪声/掉线与状态估计，并完成 SIL/PIL、WCET、数据类型和代码生成门禁。
4. 在 Vehicle10DOF 惯量、阻尼和 K&C 校准后复跑 Skidpad、Autocross 与制动入弯性能评审。

相关规格：
[系统](specs/unified-control-allocation/system.md)、
[架构](specs/unified-control-allocation/architecture.md)、
[实施计划](specs/unified-control-allocation/implementation-plan.md)、
[测试计划](specs/unified-control-allocation/test-plan.md)。
