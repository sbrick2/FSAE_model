# TorqueVectoring 基础扭矩矢量控制设计与验证

> 状态：完成  
> 日期：2026-07-31  
> 环境：MATLAB / Simulink R2026a

## 1. 范围与交付

TorqueVectoring 在已验证的 PathTracking 闭环与 TireModel `TireMF62` 上增加基础扭矩矢量控制，且不修改
PathTracking 生产基线。新增生产模型如下：

| 模型 | 职责 |
|---|---|
| `YawController.slx` | 参考横摆角速度、稳定边界、PI 横摆力矩与防积分饱和 |
| `TorqueVectoringAllocator.slx` | TV0/TV1 四轮转矩、功率、转速和摩擦制动约束 |
| `TorqueVectoringVehicleController.slx` | PathTracking 纵向需求、横摆控制与分配器集成 |
| `TorqueVectoringPathTrackingDriver.slx` | 在既有驾驶员命令上只覆盖 TorqueVectoring TV 开关 |
| `FSAE_TorqueVectoring_ClosedLoop.slx` | TorqueVectoring 顶层闭环，复用既有 Plant、Sensor 与 TTC/MF6.2 |

四轮数组顺序保持 `[FL, FR, RL, RR]`，正横摆力矩使右侧驱动转矩增加、左侧减小。
跨模型 Bus 合同未改变。

## 2. 横摆控制

运动学参考采用

```text
r_kin = Ux * delta / (L + Kus * Ux^2)
```

并同时受以下边界约束：

```text
|r_ref| <= mu*g/max(|Ux|, Ux_min)
|r_ref| <= ay_max/max(|Ux|, Ux_min)
|r_ref| <= r_max
```

`|Ux| < Ux_min` 时参考平滑归零。只要 IMU 有效，TV0 与 TV1 均计算同一参考和
横摆误差，保证成对评审公平；TV0 仅将积分和横摆力矩请求清零。

TV1 使用离散 PI：

```text
Mz_raw = Kp*(r_ref-r) + I
Mz_req = clamp(Mz_raw, -Mz_max, Mz_max)
```

积分器包含反算防饱和、独立限幅，以及在控制禁用或传感器无效时的复位。

## 3. 四轮分配

共同驱动转矩和左右差动为

```text
T_common = Fx_req*R/(4*i*eta_g)
dT = Mz_req*R/((track_f+track_r)*i*eta_g)
T_motor = T_common + [-dT, +dT, -dT, +dT]
```

分配器按顺序施加：

1. 单电机转矩和转速限制；
2. 80 kW 驱动与 10 kW 再生电功率限制，逆变器效率独立按无量纲量处理；
3. 优先保持左右差动、先削减共同转矩；
4. 剩余制动需求按既有四轮摩擦制动能力分配。

UnifiedControl 将把 TC、TV、再生制动和功率限制升级为统一约束分配；TorqueVectoring 不提前声称已完成
该优化问题。

## 4. 参数与 TTC 使用

TorqueVectoring 通过 `installTorqueVectoringControlData` 把 `TorqueVectoringControlDesign` 写入
`VehicleData.sldd`。当前 `Kp=360`、`Ki=90`、`Mz_max=350 N*m`、
`Ts=5 ms`、`mu_est=1.0` 等均标记为暂定设计值。

轮胎继续使用 TireModel 所选 `Round9_43075_R20_Rim7`：

- 横向与外倾来自 43075 R20 目标胎；
- 纵向与联合滑移来自 43100 R20 异尺寸代理；
- 抽样峰值没有被直接当作道路附着估计；
- 所有结论仅适用于当前合成车辆与场景，不代表实车标定。

## 5. 测试

| 层级 | 结果 |
|---|---:|
| `YawController.feature` 完整编译 | 5/5 场景，15/15 断言 |
| `TorqueVectoringAllocator.feature` 完整编译 | 5/5 场景，23/23 断言 |
| 生产模型结构检查 | 5/5 healthy |
| 顶层更新图与短时闭环 | 通过，无最终总线标签警告 |

测试覆盖 TV 禁用、参考边界、反馈极性、力矩限幅、防积分饱和、基准等分、
正横摆差动、单轮约束、驱动功率和制动分配。

项目未绑定默认 Model Advisor 配置，也未指定 MAB、JMAAB、MISRA SLSF 或
ISO 26262 等标准。根据 `AGENTS_FSAE_Simulink.md`，上述结构检查不能被描述为
标准符合性结论；标准化检查将在 OQ-014 确认后运行。

## 6. 成对场景结果

`createTorqueVectoringScenarioMatrix` 为 Skidpad 与 Autocross 分别生成 TV0/TV1 工况。
每对工况除 `EnableTV` 外使用同一场景和配置指纹。

| 赛项 | 指标 | TV0 | TV1 | TV1 相对变化 |
|---|---|---:|---:|---:|
| Skidpad | 横摆 RMSE (rad/s) | 0.044942 | 0.042626 | 改善 5.153% |
| Skidpad | 横向 RMSE (m) | 0.378480 | 0.379641 | 变差 0.307% |
| Skidpad | 驱动能量 (J) | 35621.4 | 35003.9 | 降低 1.733% |
| Skidpad | 控制饱和占比 | 0.6362 | 0.6873 | 增加 5.107 个百分点 |
| Skidpad | 越界占比 | 0 | 0 | 不变 |
| Autocross | 横摆 RMSE (rad/s) | 0.081524 | 0.081390 | 改善 0.164% |
| Autocross | 横向 RMSE (m) | 0.373557 | 0.373557 | 基本不变 |
| Autocross | 驱动能量 (J) | 193575.8 | 191339.4 | 降低 1.155% |
| Autocross | 控制饱和占比 | 0.3188 | 0.4333 | 增加 11.453 个百分点 |
| Autocross | 越界占比 | 0.006063 | 0.006107 | 增加 0.00446 个百分点 |

两类赛项的峰值轮胎利用率均达到该轮历史验证采用的 `1.0` 绝对附着上限。
现行轮胎接口默认使用参考轮胎能力（`RoadGripScale=1`、`RoadMuLimit=Inf`），
因此该历史结果不能直接代表现行默认路面。TV1 带来小幅横摆与能耗改善，但
饱和更频繁；Skidpad 路径误差略有恶化，Autocross 原有约 0.61% 越界也没有
被 TV 消除。因此当前增益可作为 TorqueVectoring 功能基线，不能作为最终标定。

## 7. 复现

```matlab
project = initProject();
addpath(fullfile(project.RootFolder, "scripts", "torque_vectoring"));
addpath(fullfile(project.RootFolder, "tests", "torque_vectoring"));
installTorqueVectoringControlData();
report = runTorqueVectoringVerification( ...
    Events=["Skidpad", "Autocross"], ...
    SaveSummary=true, ...
    UseFastRestart=true);
```

默认只保存汇总到 `tests/torque_vectoring/results/TorqueVectoringVerificationSummary.mat`，不把完整时域输出纳入
版本控制。
