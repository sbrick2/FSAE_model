# 仿真入口使用说明

`simulation/` 集中存放用户可以直接打开并运行的 MATLAB 入口，先按仿真方法
分为准静态和时域闭环，再按功能分为仿真和绘图：

```text
simulation/
├─ quasi_static/
│  ├─ simulation/   # GGV、准静态圈速、参数扫描
│  └─ plotting/     # 准静态结果绘图
├─ tire/
│  └─ plotting/     # 轮胎模型本体特性
└─ time_domain_closed_loop/
   ├─ simulation/   # Simulink 闭环仿真
   └─ plotting/     # 闭环结果绘图与分析
```

算法、场景、结果整理和绘图辅助函数仍保留在 `scripts/` 与 `scenarios/`。

各功能区的局部说明见：

- [准静态 GGV、圈速与参数扫描](quasi_static/README.md)；
- [时域闭环赛项仿真](time_domain_closed_loop/README.md)；
- [轮胎特性分析](tire/README.md)；
- [统一结果信号参考](Result_Signal_Reference.md)。
各功能区的局部说明见：

- [准静态 GGV、圈速与参数扫描](quasi_static/README.md)；
- [时域闭环赛项仿真](time_domain_closed_loop/README.md)；
- [轮胎特性分析](tire/README.md)；
- [统一结果信号参考](Result_Signal_Reference.md)。

根目录 `results/` 按结果用途和时域赛事组织：

```text
results/
├─ quasi_static/
│  ├─ parameter_sweep/
│  ├─ ggv/
│  └─ lap_time/
├─ tire/
│  └─ characteristics/
└─ time_domain_closed_loop/
   ├─ autocross/
   ├─ endurance/
   ├─ acceleration/
   └─ skidpad/
```

## 1. 运行前准备

推荐使用 MATLAB / Simulink R2026a，并从项目根目录打开 MATLAB Project：

```matlab
project = openProject(fullfile(pwd, "FSAE_Simulation.prj"));
initProject();
```

如果 MATLAB 当前目录不是项目根目录，请将 `pwd` 替换为项目根目录的绝对路径。
初始化会加载 Project Path、`VehicleData.sldd` 和共享 Bus 定义。

### 图形化入口

需要从同一窗口完成配置、仿真和结果浏览时，可在项目根目录运行：

```matlab
app = launchFSAESimulationApp();
```

图形界面顶层分为“车辆参数”“准静态”和“时域闭环”三个主要模块，并调用本章
列出的规范仿真和绘图函数，不维护第二套模型或结果格式。长时间仿真的进度仍会
同步显示在 MATLAB 命令窗口。

“车辆参数”页直接读取 `data/VehicleData.sldd` 中整车、轮胎、空气动力、动力总成、
电池、制动和仿真七类实数标量记录。参数名使用中文，完整字段路径保留在相邻列中。
可按分类或文本筛选，查看字段路径、单位、边界、
来源、置信度和说明，并在表格中暂存数值修改。点击“保存到数据字典”时会统一检查
有限性与记录上下限，并先将原字典备份到 `results/vehicle_parameter_backups/`；向量、
地图、Bus 和其他复合数据不在标量编辑器中直接修改。

## 2. 可直接运行的脚本

| 功能域 | 入口文件 | 用途 | 默认输出 |
|---|---|---|---|
| 准静态/仿真 | `plotCurrentGGV3D.m` | 生成当前参数三维 GGV | `results/quasi_static/ggv/` |
| 准静态/仿真 | `runQuasiStaticLapTime.m` | 使用已有 GGV 计算一次赛项时间 | `results/quasi_static/lap_time/<event>/` |
| 准静态/仿真 | `runLapTimeParameterSweep.m` | 单参数或双参数圈时扫描 | `results/quasi_static/parameter_sweep/<event>/` |
| 准静态/绘图 | `plotQuasiStaticTrackParameterMap.m` | 赛道按所选参数着色 | 图窗；可选 MATLAB FIG |
| 准静态/绘图 | `plotQuasiStaticParameterTrace.m` | 两个所选参数的折线图 | 图窗；可选 MATLAB FIG |
| 准静态/绘图 | `plotQuasiStaticAnalysisResults.m` | QuasiStatic GGV 能力扫描结果 | 单图窗多标签页；可选 MATLAB FIG |
| 轮胎/绘图 | `plotTireModelCharacteristics.m` | 纯滑移、轮荷/外倾角影响和联合滑移特性 | 单图窗多标签页、特性指标表；可选 MATLAB FIG |
| 时域闭环/仿真 | `runLapSimulation.m` | 可选 7DOF/10DOF 的闭环赛项仿真 | `results/time_domain_closed_loop/<event>/` |
| 时域闭环/绘图 | `plotLapSimulationResults.m` | 自由选择已保存圈速字段绘图 | 单图窗多标签页；可选 MATLAB FIG |
| 时域闭环/绘图 | `plotVehicleDynamicsDashboard.m` | 整车动力学综合分析 | 单图窗多标签页；可选 MATLAB FIG |
| 时域闭环/绘图 | `plotPathTrackingPerformance.m` | 路径、速度、横摆和转向跟踪 | 单图窗多标签页；可选 MATLAB FIG |
| 时域闭环/绘图 | `plotTireUtilization.m` | 四轮轮胎力、滑移和摩擦利用率 | 单图窗多标签页；可选 MATLAB FIG |
| 时域闭环/绘图 | `plotSuspensionLoads.m` | 动态轮荷、车身姿态和悬架信号 | 单图窗多标签页；可选 MATLAB FIG |
| 时域闭环/绘图 | `plotMotorOperatingMap.m` | 四电机工作点和限制状态 | 单图窗多标签页；可选 MATLAB FIG |
| 时域闭环/绘图 | `plotEnergyAnalysis.m` | 电池功率、SOC 和能量回收 | 单图窗多标签页；可选 MATLAB FIG |
| 时域闭环/绘图 | `plotControlAllocation.m` | 横摆/纵向目标与四轮分配 | 单图窗多标签页；可选 MATLAB FIG |
| 时域闭环/绘图 | `plotLapPerformance.m` | 速度包络、累计时间和分段圈时 | 单图窗多标签页；可选 MATLAB FIG |
| 时域闭环/绘图 | `compareSimulationRuns.m` | 两次仿真的性能差异 | 单图窗多标签页；指标表 |
| 时域闭环/绘图 | `plotModelValidation.m` | 仿真与实测数据对齐 | 单图窗多标签页；误差指标表 |
| 时域闭环/绘图 | `plotOpenLoopSimulationResults.m` | 兼容读取既有开环结果 | 单图窗多标签页；可选 MATLAB FIG |

脚本可以在 MATLAB 编辑器中点击 **Run**，也可以使用 `run`：

```matlab
projectRoot = string(project.RootFolder);
run(fullfile(projectRoot, "simulation", "time_domain_closed_loop", ...
    "simulation", "runLapSimulation.m"));
```

## 3. 时域闭环赛项仿真

运行：

```matlab
run(fullfile(project.RootFolder, "simulation", "time_domain_closed_loop", ...
    "simulation", "runLapSimulation.m"));
```

运行前在脚本顶部的“圈速仿真参数选择区”修改配置。主要选项包括：

- `cfg.Track.Name`：`"acceleration"`、`"skidpad"`、`"autocross"` 或 `"endurance"`；
- `cfg.Track.NumberOfLaps`：耐久赛目标圈数；
- `cfg.Vehicle.DynamicsModel`：`"7DOF"` 或 `"10DOF"`；
- `cfg.Driver.Model`：`"adaptive_autocross"` 或 `"reference_speed"`；
- `cfg.Vehicle.TireModel`：GGV 规划支持 `"mf62"` 或 `"ttc_map"`；
- `cfg.SpeedPlanner.SpeedStep`：GGV 速度网格间隔，单位 `m/s`；
- `cfg.SpeedPlanner.ConstraintScale`：GGV 边界使用比例，默认 `0.90`；
- `cfg.SpeedPlanner.ReferenceLateralAccelerationLimit`：参考速度闭环横向限值，默认 `4.10 m/s^2`；
- `cfg.SpeedPlanner.ReferencePlanningDeceleration`：参考速度闭环规划制动，默认 `2.50 m/s^2`；
- `cfg.SpeedPlanner.ReferenceSpeedSafetyFactor`：参考速度专用安全系数，默认 `1.00`；
- `cfg.SpeedPlanner.UseCache`：是否复用 `cache/speed_planner/` 中的匹配 GGV；
- `cfg.Simulation.SolverProfile`：日常估算使用 `"fast"`，最终复核使用 `"standard"`；
- `cfg.Visualization.Enabled`：仿真完成后是否显示最终轨迹图；
- `cfg.Output.RunName`：本次运行名称。

`adaptive_autocross` 按整车自由度路由到独立的
`FSAE_AdaptiveAutocross_7DOF` 或 `FSAE_AdaptiveAutocross_10DOF`，两者共用
`AdaptiveAutocrossDriver`。驾驶员直接
使用赛道几何和传感器，在线扫描曲率、制动可达性和四轮外缘边界净空；它不读取
`Track.ReferenceSpeed`，该字段保持全零仅用于兼容结果 Bus。包络最小净空、
最大越界、计数和比例会写入 MAT/JSON，最终门槛是全程包络零越界。

`reference_speed` 模式下，7DOF 路由到 `FSAE_TorqueVectoring_ClosedLoop`，10DOF 路由到
`FSAE_Vehicle10DOF_ClosedLoop`；脚本根据当前动力系统生成 GGV，再结合曲率执行前向加速与
后向制动传播得到 `Track.ReferenceSpeed(s)`。规划结果还会与
`SpeedPlanner.Reference*` 最高速度、最大加速、规划制动、横向加速度和安全系数
取交集，并继续受驾驶员实际加速/制动/最高车速限幅，保证参考曲线可由闭环实现。
该组参数不再继承 `adaptive_autocross` 的全局降速裕度。参考速度模式同样执行四轮
包络零越界资格检查。两种模式和两种自由度组合都会把驾驶员、自由度、
顶层模型和 Plant 写入运行配置。详细算法和限制见
[`models/driver/Adaptive_Autocross_Driver.md`](../models/driver/Adaptive_Autocross_Driver.md)。

## 4. 已保存结果绘图

运行：

```matlab
run(fullfile(project.RootFolder, "simulation", ...
    "time_domain_closed_loop", "plotting", "plotLapSimulationResults.m"));
```

在脚本顶部修改：

- `plotCfg.Result.File`：指定 MAT 结果；留空时自动选择对应赛事的最新结果；
- `plotCfg.Result.Track`：结果所属赛事；
- `plotCfg.X.Parameter`：时间、距离、赛道进度或其他标量信号；
- `plotCfg.Y.Parameters`：需要绘制的车辆、车轮、驾驶员或执行器信号；
- `plotCfg.Wheel.Channels`：选择 `FL/FR/RL/RR`；
- `plotCfg.Export`：控制 MATLAB FIG 导出；输出文件可在 MATLAB 中直接打开和编辑。

该脚本只分析已保存数据，不启动 Simulink。

## 5. 当前参数三维 GGV

运行：

```matlab
run(fullfile(project.RootFolder, "simulation", "quasi_static", ...
    "simulation", "plotCurrentGGV3D.m"));
```

脚本直接读取 `data/VehicleData.sldd`。主要配置包括：

- `SpeedMax`：最大车速；速度层固定为 `1 m/s` 间隔；
- `LateralPointCount`：每个速度层的横向采样数；
- `VehicleDynamicsModel`：`"7DOF"` 或 `"10DOF"`；
- `TireModel`：默认 `"MF62"`；
- `AllocationMode`：扫描建议 `"FAST"`；
- `EnvelopeLoadPointCount`：MF62 联合滑移包络的法向载荷查表点数，默认 `81`；
- `ParallelPoolType`：默认 `"Threads"`，不兼容时可改为 `"Processes"`；
- `ParallelWorkerCount`：并行工作单元数，`0` 表示使用默认配置；
- `OutputFolder`、`SaveMat`：输出位置和 MAT 保存开关。

脚本先按法向载荷并行预计算一次 MF62 联合滑移方向支撑查表，再按速度层并行
计算 GGV；线程池可共享只读查表并减少进程间序列化。若线程池启动失败会回退
到进程池。超出预计算法向载荷范围时停止并报告错误，不执行外推。脚本输出
MATLAB FIG 和可选 MAT 文件；若数据字典仍包含参与计算的 `NaN` 占位参数，脚本
同样会停止并报告错误。

## 6. 已有 GGV 的单工况准静态圈速

运行：

```matlab
run(fullfile(project.RootFolder, "simulation", ...
    "quasi_static", "simulation", "runQuasiStaticLapTime.m"));
```

这个入口不重新计算 GGV，也不执行参数扫描。它读取一份已保存的 GGV，按照赛道
曲率插值得到横向速度限制，再通过前向加速和后向制动传播，直接给出一个赛项
时间。脚本顶部可配置：

- `eventName`：`"acceleration"`、`"skidpad"` 或 `"autocross"`；
- `ggvFile`：指定包含变量 `ggv` 的 MAT 文件；留空时优先读取
  `results/quasi_static/ggv/` 下最新的有效 GGV；
- `sampleDistance`：赛道离散间隔；`NaN` 使用各赛项默认值；
- `initialSpeed`、`finalSpeed`：开放赛道的起终点速度约束；
- `passCount`：前向/后向传播次数；
- `maximumSpeed`：额外最高车速限制，实际速度仍受 GGV 网格上限约束；
- `saveResults`、`outputFolder`：结果保存开关和目录。

运行后工作区包含 `summary` 和 `quasiStaticResult`。默认保存结构化 MAT 与可编辑
MATLAB FIG；运行结束只绘制赛道按速度着色图。结果只反映所选 GGV、场景赛道和
准静态点质量模型，不包含驾驶员跟踪误差、控制器动态、轮胎温度、能量衰减或
瞬态悬架效应。

需要单独绘制赛道参数云图时调用：

```matlab
[figureHandle, plotInfo] = plotQuasiStaticTrackParameterMap( ...
    Event="autocross", ...
    Field="SpeedProfile.LateralAcceleration", ...
    DistanceLimits=[100 700], ...
    ColorLimits=[-12 12], ...
    XLimits=[NaN NaN], ...
    YLimits=[NaN NaN]);
```

需要参数折线图时调用：

```matlab
[figureHandle, plotInfo] = plotQuasiStaticParameterTrace( ...
    Event="autocross", ...
    XField="SpeedProfile.ProgressS", ...
    YField="SpeedProfile.Speed", ...
    XLimits=[100 700], ...
    YLimits=[0 32]);
```

两个函数的第一个参数都可以是 `quasiStaticResult` 结构或 MAT 文件路径；留空时读取
所选赛事的最新结果。字段路径相对于 `quasiStaticResult`，常用项包括
`SpeedProfile.Speed`、`SpeedProfile.LateralAcceleration`、
`SpeedProfile.CurvatureSpeedLimit`、`SpeedProfile.Time`、`Track.Curvature` 和
`Track.ReferenceSpeed`。范围中使用 `NaN` 表示自动；通过 `SaveFigure=true` 可将
可编辑 FIG 保存到 `results/quasi_static/lap_time/<event>/plots/`。

## 7. 准稳态圈时参数扫描

推荐从 `launchFSAESimulationApp` 启动界面，在“准静态 > 参数扫描”页选择赛道、
单/双参数以及每个参数的起点、终点和步长。参数下拉项同时显示中文名称与实际字段
路径；界面会预览扫描点数和总工况数，并在并行扫描期间显示完成比例、当前参数值与圈时。
参数目录由 `VehicleData.sldd` 动态生成，包含准静态求解器实际使用的全部有限数值标量；
当前字典为 20 项。向量、文本、缺失值和不会进入准静态求解的字段不会列入标量扫描。

脚本入口仍可独立运行：

运行：

```matlab
run(fullfile(project.RootFolder, "simulation", ...
    "quasi_static", "simulation", "runLapTimeParameterSweep.m"));
```

直接运行脚本时，在脚本顶部选择：

- `scanMode`：`"single"` 或 `"double"`；
- `eventName`：`"acceleration"`、`"skidpad"` 或 `"autocross"`；
- `parameter1Path`、`parameter2Path`：例如 `Aero.CdA`、`Vehicle.Mass`；
- 每个参数的 `Start`、`Stop`、`Step` 和单位；
- GGV 分辨率、轮胎模式和分配模式；
- `vehicleDynamicsModel`：`"7DOF"` 或 `"10DOF"`；
- 是否保存每个工况的紧凑 GGV、速度曲线和汇总图。

扫描通过 `parfor` 并行执行。失败工况不会被丢弃，而是在结果中保留
`Success=false` 和 `FailureReason`。

准静态 `10DOF` 选项求解 Vehicle10DOF 悬架的零速度升沉/侧倾/俯仰平衡，使用
`Vehicle10DOFSuspension.WheelRate` 和 `AdditionalRollStiffness`；阻尼在平衡点为零力。
它不是垂向瞬态仿真，动态阻尼响应仍应使用时域 `10DOF` 路径。

## 8. 赛车开发分析入口

轮胎模型本体特性不需要整车仿真结果。打开脚本后在顶部参数选择区设置 TireModel
Profile、模型、轮荷和滑移范围，然后直接点击 **Run**：

```matlab
run(fullfile(project.RootFolder, "simulation", "tire", "plotting", ...
    "plotTireModelCharacteristics.m"));
```

运行后工作区保留 `tireCharacteristicFigures`、`tireCharacteristics` 和
`tireModelSummary`。脚本默认 `RoadGripScale=1`、`RoadMuLimit=Inf`，直接观察
参考路面上的轮胎模型自身峰值；如需研究路面变化，可分别设置相对抓地缩放和
可选的绝对摩擦系数上限。TTC Map 超出试验
数据域的点不会作为外推结果绘制。当前 Round 9 TTC Map 的滑移率维度恒为零，
因此 TTCMap 模式会自动绘制横向/外倾特性和 Map 采样域覆盖，不会生成无数据
依据的纯纵滑或联合滑移曲线；完整纵向与联合滑移特性应使用 MF62 模式。启用
`AutoSmooth` 后，脚本按 `SmoothingFraction` 自动选择奇数点 Savitzky–Golay
窗口平滑特性曲线，但保留原始 Map 覆盖散点。每张特性图使用同一图窗中的独立标签页；启用保存后，
各 MATLAB FIG 默认写入 `results/tire/characteristics/`。

新增分析文件都是可直接调用的函数。无参数时默认读取最新的 Autocross 规范化
结果；也可以把结果结构或 `lap_simulation_result.mat` 路径作为第一个参数：

```matlab
plotVehicleDynamicsDashboard();
plotTireUtilization("E:/FSAE_model/results/time_domain_closed_loop/" + ...
    "autocross/.../lap_simulation_result.mat");

[figureHandles, sectorTable, summary] = plotLapPerformance( ...
    "", Track="skidpad", SectorCount=8, SaveFigure=true);
```

通用选项包括 `Track`、`Axis`、`Visible`、`SaveFigure` 和 `OutputFolder`；具体
函数只暴露与该分析有关的选项。每张分析图使用同一图窗中的独立标签页；默认不保存文件，
启用 `SaveFigure=true` 后将整个标签页图窗写入一个 MATLAB FIG，默认位置为
`results/time_domain_closed_loop/<event>/plots/<analysis-name>/`。

两次结果对比在未给文件时自动选择同一赛事下最新的两次结果：

```matlab
[figureHandles, comparison] = compareSimulationRuns( ...
    "", "", Track="autocross", LabelA="Baseline", LabelB="Candidate");
```

实测验证要求输入采用 SI 单位，并至少包含 `Time`，可选字段为 `Speed`、`Ax`、
`Ay`、`YawRate`。未指定文件时会搜索 `data/ValidationData/`：

```matlab
[figureHandles, metrics] = plotModelValidation( ...
    "", "data/ValidationData/autocross_measurement.csv");
```

上述绘图入口共用的结果加载、坐标轴、绘图主题和有限值统计逻辑统一保存在
`scripts/reporting/`，不再使用 `private` 目录。`scripts/simulation/` 和
`scripts/tire/` 中的其他辅助函数保持原位。

## 9. 功能边界

- `scripts/ggv/`、`scripts/lap_time/`、`scripts/simulation/`、
  `scripts/reporting/` 和 `scenarios/` 中的文件是这些入口调用的辅助函数；
- `runQuasiStaticVerification`、`runTorqueVectoringVerification`、`runVehicle10DOFVerification` 和
  `runUnifiedControlVerification` 是带参数的验证 API，统一保存在 `tests/<milestone>/`；
- 验证输出写入对应的 `tests/<milestone>/results/`，根目录 `results/` 仅保存
  对整车开发有复用价值的圈速、GGV 和分析结果；
- 当前物理参数和控制标定仍有占位或代理数据，输出不能直接解释为实车性能。
