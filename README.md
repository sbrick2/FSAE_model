# FSAE 整车仿真平台

本项目面向四轮独立驱动 FSAE 赛车，逐步建立可替换、可验证且参数来源可追踪的 MATLAB / Simulink 整车仿真平台。目标环境为 MATLAB / Simulink R2026a。使用ChatGPT-5.6协助开发。

## 当前状态

- project foundation 工程骨架与接口冻结已完成并通过自动验收。
- VehicleDynamicsCore 七自由度车辆动力学核心已完成：车身 3DOF、四轮转动、全局位姿和轮心坐标变换均已通过解析回归。
- SlipKinematics–BatteryPowerBaseline 六类组件已完成并通过组件门禁：滑移运动学、简化轮胎、轮荷转移、气动、驱动系统和电池基线。
- ResultVisualization 结果容器、标准绘图/导出函数和 OpenLoopScenarios 八类开环场景资产已完成；OpenLoopScenarios 八场景回归通过，OpenLoopPlantIntegration 整车 Plant 集成与结构验收完成。
- TireModel TTC 轮胎模型已完成：目标胎为 Hoosier 43075 16x7.5-10 R20、7 英寸轮辋；横向/外倾使用目标胎实测，纵向/联合滑移使用同 R20 配方的 43100 异尺寸代理数据。
- `TireModel` 已提供 `TireSimple`、`TireMF62`、`TireTTCMap` 三模式，默认使用 `TireMF62`；两种高保真模式均已通过 `VehiclePlant` 引用模型编译与短时仿真。
- PathTracking 路径跟踪闭环已完成：四个生产模型结构健康，直线加速、八字、1 km Autocross 和两圈 Endurance 闭环均无越界或数值发散。
- 独立无参考速度 Autocross 驾驶员已接入 7DOF，直接使用赛道几何与传感器。修复制动接口后需重新运行全圈；修复前的 `135.074866 s` 仅保留为归档对照，不再作为当前资格结论。
- QuasiStatic GGV、准稳态圈速、参数扫描和 PathTracking 时域交叉验证已完成。
- TorqueVectoring 基础扭矩矢量控制已完成：TV0/TV1 分配、参考横摆与 PI 反馈、Skidpad/Autocross 成对工况和量化评审均已实现；控制增益仍为暂定值。
- TorqueVectoring 行为测试、结构检查和更新图已通过；项目尚未绑定 Model Advisor 默认配置，标准/版本需在关闭合规门禁前确认。
- Vehicle10DOF 10DOF 动态轮荷功能基线已完成：垂向、侧倾、俯仰、E41 悬架刚度和 TTX25 阻尼查表已集成；惯量、实车阻尼设置和 K&C 外倾曲线仍待校准。
- UnifiedControl 统一控制分配功能基线已完成：`Fx/Mz` 四轮分配、TV/TC/再生/功率融合、速率恢复和降级模式已集成至独立 10DOF 闭环；8/8 组件场景、45/45 断言和 3/3 顶层场景通过。
- 当前无可复用的自定义 Simulink 库；后续将在 `models/libraries/` 建立项目库。
- `VehiclePlant` 已接入 VehicleDynamicsCore 与 SlipKinematics–BatteryPowerBaseline 组件；`VehicleStateBus`、`PowertrainStateBus` 连接已闭合，顶层结构检查为 healthy。结果容器保留未暴露诊断信号清单，不使用零值静默补齐。
- 车辆真实质量、几何和惯量尚未确认；43075 的直接纵向/联合 TTC 数据仍缺失，不得把代理拟合描述为目标胎实测。
- 未知参数必须使用明确占位并登记，不得描述为实测值。

## 文档入口

- [整车仿真开发蓝图](ROADMAP.md)
- [整车模型使用说明](models/README.md)
- [系统架构](models/Architecture.md)、[坐标系](models/CoordinateSystem.md)与[信号接口](models/SignalInterfaces.md)
- [参数与数据来源](data/ParameterSources.md)
- [仿真入口使用说明](simulation/README.md)
- [圈速结果绘图使用指南](simulation/time_domain_closed_loop/plotting/README.md)
- [无参考速度 Autocross 驾驶员](models/driver/Adaptive_Autocross_Driver.md)
- [TorqueVectoring 扭矩矢量设计与验证](models/controller/Torque_Vectoring.md)
- [UnifiedControl 统一控制分配设计与验证](models/controller/Unified_Control_Allocation.md)
- [阶段验证历史](tests/Verification_History.md)
- [待确认问题](OPEN_QUESTIONS.md)
- [变更记录](CHANGELOG.md)

## 目录说明

| 目录 | 用途 |
|---|---|
| `models/` | 顶层、Plant、控制器、驾驶员、组件和项目库 |
| `data/` | 车辆数据字典及轮胎、动力系统、气动和赛道数据 |
| `scripts/` | 初始化、数据处理、GGV、圈速、仿真和报告脚本 |
| `apps/` | MATLAB 图形化仿真与结果分析界面 |
| `simulation/` | 按“准静态/时域闭环”和“仿真/绘图”组织的用户入口 |
| `scenarios/` | 赛项工况 |
| `tests/` | 按里程碑或功能组织的验证代码、测试资产及本地验证结果 |
| `results/` | 准静态按功能、时域闭环按赛事保存可复用结果；不存放验证汇总 |
| `tools/` | 项目辅助工具 |

## 快速开始

```matlab
projectRoot = pwd;
openProject(fullfile(projectRoot, "FSAE_Simulation.prj"));
initProject();
```

### 图形化仿真中心

在项目根目录运行：

```matlab
app = launchFSAESimulationApp();
```

界面顶层分为“准静态”和“时域闭环”两个主要模块。准静态模块提供基于已有
GGV 的单次圈速、GGV/参数扫描入口、历史结果和速度轨迹；时域闭环模块提供
车辆动力学、驾驶员、轮胎模型与求解精度配置，以及闭环结果和现有分页分析图。
原有脚本运行方式保持兼容。

### 圈速仿真与结果分析

规范化圈速时域仿真入口为
`simulation/time_domain_closed_loop/simulation/runLapSimulation.m`，结果绘图入口为
`simulation/time_domain_closed_loop/plotting/plotLapSimulationResults.m`。两者的配置
都集中在脚本顶部；仿真结果默认写入
`results/time_domain_closed_loop/<event>/`，绘图脚本可在关闭
Simulink 后独立读取 MAT 文件。启用 `Visualization.Enabled` 时，仿真完成后会
显示包含赛道边界、速度着色车辆轨迹和结果摘要的最终赛道窗口。
默认自适应驾驶员配置只保存七组顶层 `yout`、关闭重复的 `logsout`、采用逐点输出
（`OutputDecimation=1`），并按 `Simulation.FinishCheckPeriod` 检查实际车辆位置、
在完成赛项后停止求解器；该设置不改变车辆动力学积分。运行期间的进度在命令行显示，
轨迹图只在求解完成后生成。
默认 `Simulation.SolverProfile="standard"` 使用 `RelTol=1e-3` 做最终复核；
日常快速估算可切换为 `"fast"`（`RelTol=1e-2`）。`Driver.Model` 提供两条路径：
`"adaptive_autocross"` 直接扫描赛道几何、车辆包络和传感器状态，并把原始 GGV
前后向规划结果作为不可超越的速度能力上限。宽度约束 QP 生成无硬锁的软赛车线，
并保留一条只供离线 GGV 使用的高曲率保护线；实时转向默认跟踪物理中心线，可通过
`ReferencePathMaximumBlend` 渐进启用赛车线跟踪。其 MATLAB Function
Chart 固定以 10 ms 离散执行，避免 persistent 控制状态依赖求解器主步。
`"reference_speed"` 使用受参考速度专用
纵横向能力、最高速度和安全系数约束的 GGV 前后向传播，并由 TorqueVectoring/Vehicle10DOF 驾驶员跟踪；
该组限值与 `adaptive_autocross` 的在线边界保护相互独立。前者使用独立 7DOF
顶层；两种模式都以四轮名义外缘零越界作为硬验收门槛。
当前标准档代理赛道资格圈为 `76.513132 s`，相对原始 GGV `63.761665 s` 的比值
为 `1.199986`，四轮包络零越界；详见 `models/driver/Adaptive_Autocross_Driver.md`。
结果层对 Skidpad 等自交赛道使用全赛道最近中心线计算物理边界距离，路径进度
仍使用分支连续投影，避免在中心交叉口误锁到入口/出口直线而产生虚假越界。

```matlab
projectRoot = pwd;
openProject(fullfile(projectRoot, "FSAE_Simulation.prj"));
addpath(genpath(projectRoot));
initProject();
run(fullfile(projectRoot, "simulation", "time_domain_closed_loop", ...
    "simulation", "runLapSimulation.m"));

% 修改绘图脚本顶部参数后，可不启动模型直接分析最近一次结果。
run(fullfile(projectRoot, "simulation", "time_domain_closed_loop", ...
    "plotting", "plotLapSimulationResults.m"));
```

当前车辆、气动、动力系统和部分轮胎参数仍可能是占位值；结果中的性能指标不能
直接解释为实车性能结论。

## MATLAB 窗口与模型工具附着

本项目的 MATLAB 模型工具使用“附着现有会话”模式。仅打开 MATLAB
窗口还不够，当前会话必须先执行 `satk_initialize`，使其成为可附着的
共享会话。项目已提供入口脚本 `tools/startSATKSession.m`。

### 推荐方式：在已打开的 MATLAB 中执行

先将 MATLAB 当前文件夹切换到项目根目录，然后在命令窗口运行：

```matlab
addpath(fullfile(pwd, "tools"));
startSATKSession();

% 附着成功后加载项目路径、数据字典和 Bus 定义。
addpath(genpath(pwd));
initProject();
```

看到以下信息表示共享会话已建立：

```text
SATK session ready for project: E:\FSAE_model
```

随后重新调用模型读取、结构检查或仿真工具即可。当前应使用执行了
`startSATKSession` 的 MATLAB 窗口；普通启动但未初始化的窗口不能被附着。

### Windows 直接启动共享 MATLAB 窗口

需要从 PowerShell 启动时，可使用以下命令。请按本机 MATLAB 和项目的
实际安装路径修改两个路径：

```powershell
Start-Process `
  -FilePath "D:\Program Files\MATLAB\R2026a\bin\matlab.exe" `
  -ArgumentList '-desktop -r "addpath(''E:\FSAE_model\tools''); startSATKSession"'
```

`-r` 后面的 MATLAB 命令必须作为一个完整的带引号参数传入；若被拆分，
MATLAB 虽然会启动，但 `startSATKSession` 不会执行。

### 常见问题

| 现象 | 检查与处理 |
|---|---|
| `failed to attach to MATLAB session` | 确认 MATLAB 命令窗口已经执行 `startSATKSession()`，而不是只打开了 MATLAB。 |
| MATLAB 已启动但仍不能附着 | 关闭未共享的旧窗口，只保留重新执行 `startSATKSession()` 的会话，然后重试。 |
| 能附着，但提示找不到 `VehicleData.sldd` 或引用模型 | 在共享会话中执行 `addpath(genpath(pwd)); initProject();`。 |
| `satk_initialize` 未定义 | 检查 `C:\Users\<用户名>\.matlab\agentic-toolkits\simulink` 是否存在并已加入 MATLAB 路径。 |
| 多次启动后出现多个 MATLAB 窗口 | 保留输出了 `SATK session ready` 的共享窗口，其余窗口可在确认无未保存工作后手动关闭。 |

历史阶段结论保存在 `tests/Verification_History.md`，TorqueVectoring 设计与结果记录在 `models/controller/Torque_Vectoring.md`。没有实车质量、惯量和控制标定证据时，不形成实车性能结论。

## 许可证

除另有明确说明外，本项目原创代码、Simulink 模型和原创文档采用
[Apache License 2.0](LICENSE) 授权，SPDX 标识为 `Apache-2.0`。

TTC 轮胎数据、赛事手册图片、车队文件、用户提供资料和厂商资料不因本项目采用
Apache-2.0 而获得重新授权；这些材料的适用边界见
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)。仓库的公开或私有访问状态不改变
各项材料原有的授权条件。
