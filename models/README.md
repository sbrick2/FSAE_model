# FSAE 整车模型使用手册

本文面向主要从事车辆动力学、轮胎、气动或动力系统开发，但不希望先阅读大量 MATLAB/Simulink 代码的工程师。手册中的命令均以项目当前目录结构和 MATLAB/Simulink R2026a 为准。

本项目有两条互相配合的分析链：

| 分析链 | 入口 | 适合回答的问题 |
|---|---|---|
| 时域闭环 | `simulation/time_domain_closed_loop/simulation/runLapSimulation.m`，可选 TorqueVectoring/7DOF 或 Vehicle10DOF/10DOF | 车辆在具体赛道、驾驶员和控制器作用下如何运动 |
| QuasiStatic 准静态 | `createQuasiStaticConfiguration` → `generateGGV` → 圈速/扫参，可选 7DOF/10DOF 静态降阶 | 车辆在理想准稳态平衡下的性能边界和设计趋势 |
| Vehicle10DOF 动态轮荷 | `models/top/FSAE_Vehicle10DOF_ClosedLoop.slx` + `runVehicle10DOFVerification` | 悬架、侧倾/俯仰和四轮动态载荷如何响应 |

QuasiStatic GGV 是离线准稳态工具，不是 `VehiclePlant` 的替代品；时域结果包含控制器、执行器动态和路径跟踪误差。两条链应使用同一套车辆、轮胎、气动和动力系统数据，不能用两套互相矛盾的参数。

## 1. 先了解项目边界

推荐先阅读以下文件：

- [项目架构](Architecture.md)
- [坐标系与符号](CoordinateSystem.md)
- [参数来源](../data/ParameterSources.md)
- [信号接口](SignalInterfaces.md)
- [QuasiStatic GGV 与准稳态参考](../simulation/quasi_static/GGV_LapTime_Reference.md)

固定约定如下：

- 所有内部长度、速度、力和功率使用 SI 单位；角度使用 `rad`，但气动偏航角网格 `Aero.YawAngleGrid` 使用 `deg`。
- 四轮数组顺序始终是 `[FL, FR, RL, RR]`，即前左、前右、后左、后右。
- 车身 `+x` 向前、`+y` 向左；左转、正横摆角速度和正横向加速度为正。
- 电机转矩正值表示驱动，负值表示再生；摩擦制动转矩是非负的制动幅值。
- `VehicleStateBus` 是车辆真值，只用于 Plant、传感器和验证；最终控制器应使用 `SensorBus`。

## 2. 第一次使用：打开项目

在 MATLAB 当前文件夹设为项目根目录 `E:\FSAE_model`，执行：

```matlab
projectRoot = pwd;
openProject(fullfile(projectRoot, "FSAE_Simulation.prj"));
initProject();
addpath(genpath(projectRoot));
```

`initProject` 会检查 `data/VehicleData.sldd`、加载项目路径，并将 Simulink 缓存和代码生成文件放到 `<projectRoot>/cache`。正常情况下不需要运行 `setupProject`；该函数用于第一次创建或刷新项目骨架。刷新时会更新 Bus 架构，但已存在的审计物理参数组会被保留。

推荐的运行输出目录是 `results/`。该目录用于本地结果，不应当作为车辆源数据或参数的长期存储位置。

## 3. 修改车辆数据

### 3.1 用 Data Dictionary Editor 修改

打开 `data/VehicleData.sldd`，进入 `Design Data`。车辆模型共享的主要参数组如下：

| 参数组 | 典型字段 | 主要用途 |
|---|---|---|
| `Vehicle` | `Mass`、`Wheelbase`、`TrackFront/Rear`、`CGHeight`、`CGToFrontAxle`、`InertiaYaw`、转向几何、`SpringRateFront/Rear`、`WheelRateFront/Rear`、`RollStiffnessFront/Rear` | 质量、几何、刚体动力学及悬架/转向参数 |
| `Tire` | `EffectiveRadius`、`UnloadedRadius`、`WheelInertia`、低速正则化参数 | 轮速、滑移率和轮胎计算 |
| `Aero` | `CdA`、`ClAFront`、`ClARear`、`YawAngleGrid`、`YawDownforceScale` | 阻力、前后轴下压力和偏航修正 |
| `Powertrain` | `GearRatio`、`GearEfficiency`、`MotorTorqueLimit`、`MotorSpeedLimit`、`InverterEfficiency` | 电机、减速器和转速/转矩约束 |
| `Battery` | `NominalVoltage`、`Capacity`、`PowerLimitDrive`、`PowerLimitRegen`、SOC 边界 | 总功率和电池约束 |
| `Brake` | 制动压力和前后制动转矩换算 | 摩擦制动能力 |
| `Simulation` | 各模块采样时间、重力和冒烟测试时长 | 仿真配置 |
| `Variant` | Plant、轮胎、驾驶员和反馈 Variant 选择 | 模型结构选择 |

每个需要追踪的物理参数不是单个数字，而是一个记录结构，至少包含：

```matlab
Parameter.Value
Parameter.LowerBound
Parameter.UpperBound
Parameter.Unit
Parameter.Source
Parameter.Confidence
Parameter.LastUpdated
Parameter.IsMeasured
Parameter.IsPlaceholder
Parameter.Notes
```

修改时至少完成以下动作：

1. 把数值换算成 SI 单位后填入 `Value`。
2. 更新 `Source`、`Confidence`、`LastUpdated` 和 `Notes`。
3. 只有实测或直接 CAD 提取的数据才将 `IsMeasured` 设为 `true`。
4. 暂估值或尚未验证的设计值保持 `IsPlaceholder=true`，并填写可接受范围。
5. 不要直接在 Simulink 块参数框中另存一份同名车辆参数。

`Value` 已经是有限数值，并不代表它是实测值。当前字典中部分质量、几何、气动和电机字段已经有 `Design`、`Estimated`、`TypicalValue` 或厂商数据来源，但仍有质心惯量、轮胎有效半径、电池和制动等关键字段为占位值；请以 `IsPlaceholder` 和 `Source` 为准。

### 3.2 用 MATLAB 脚本批量修改

对于一组车辆数据，建议用脚本修改 Data Dictionary，而不是手工逐个点击。下面的示例只修改记录中的数值和追踪信息，字段名与当前字典一致：

```matlab
projectRoot = pwd;
dictionaryPath = fullfile(projectRoot, "data", "VehicleData.sldd");

dictionary = Simulink.data.dictionary.open(dictionaryPath);
designData = getSection(dictionary, "Design Data");
vehicleEntry = getEntry(designData, "Vehicle");
vehicle = getValue(vehicleEntry);

vehicle.Mass.Value = 320.0;                 % kg
vehicle.Mass.Unit = "kg";
vehicle.Mass.Source = "Measured corner-weight sum";
vehicle.Mass.Confidence = "High";
vehicle.Mass.LastUpdated = "2026-07-27";
vehicle.Mass.IsMeasured = true;
vehicle.Mass.IsPlaceholder = false;
vehicle.Mass.Notes = "Ready-to-run vehicle mass, including the stated test configuration.";

vehicle.Wheelbase.Value = 1.540;             % m
vehicle.TrackFront.Value = 1.200;            % m
vehicle.TrackRear.Value = 1.160;             % m
vehicle.CGToFrontAxle.Value = 0.847;         % m

setValue(vehicleEntry, vehicle);
saveChanges(dictionary);
close(dictionary);
```

同样的方式可以修改 `Tire`、`Aero`、`Powertrain`、`Battery` 和 `Brake` 条目。修改后重新执行 `initProject`，并在仿真前检查没有不应存在的 `NaN`。

### 3.3 从字典读出 PathTracking/QuasiStatic 共用参数

`createPathTrackingSimulationInput` 和 `createQuasiStaticConfiguration` 接收的是结构化参数，而不是直接接收 Data Dictionary 对象。可用下面的通用片段读出共享参数：

```matlab
dictionary = Simulink.data.dictionary.open( ...
    fullfile(pwd, "data", "VehicleData.sldd"));
designData = getSection(dictionary, "Design Data");

parameters = struct();
for groupName = ["Vehicle", "Tire", "Aero", "Powertrain", "Battery", "Brake"]
    parameters.(char(groupName)) = getValue( ...
        getEntry(designData, char(groupName)));
end
% 使用 10DOF 准静态降阶时还需读取 Vehicle10DOFSuspension。
parameters.Vehicle10DOFSuspension = getValue( ...
    getEntry(designData, "Vehicle10DOFSuspension"));
close(dictionary);
```

记录型字段可以直接传给本项目的入口函数；入口会读取其中的 `.Value`。如果 `Value` 是 `NaN`，PathTracking 控制器入口或 QuasiStatic 配置入口会拒绝运行，而不是静默补零。

### 3.4 轮胎 profile 和轮胎 Variant

当前已安装的轮胎 profile 是 `Round9_43075_R20_Rim7`。可用下列函数选择轮胎模型：

```matlab
selection = selectTireProfile( ...
    "Round9_43075_R20_Rim7", 1, ConfirmVehicleSelection = true);
clear getSelectedTireProfile
```

`ModelMode` 含义为：

| 值 | 模型 | 用途 |
|---:|---|---|
| `0` | `TireSimple` | 接口、数值稳定性和快速调试 |
| `1` | `TireMF62` | 常规时域仿真和 QuasiStatic GGV，推荐基线 |
| `2` | `TireTTCMap` | 目标胎零纵向滑移的横向/外倾参考 |

当前 43075 profile 的横向和外倾数据来自目标胎；纵向和联合滑移使用同配方 43100 R20 代理数据。也就是说，使用 `MF62` 运行加速或 GGV 时，纵向能力不能描述为 43075 目标尺寸的直接实测结果。

不要手动编辑 `TireMFParameters` 或 `TireMapPoints`。若有新的 TTC 数据，应走 TireModel 轮胎数据处理流程，再将选定 profile 安装回字典。

## 4. 运行 Vehicle10DOF 10DOF 动态轮荷模型

Vehicle10DOF 使用隔离的新模型文件，不会替换 TorqueVectoring/7DOF 基线。标准初始化和验证命令为：

```matlab
initProject();
installE41VehicleParameters();
installVehicle10DOFSuspensionData();
report = runVehicle10DOFVerification( ...
    RunClosedLoop=true, SaveSummary=true);
```

结果保存在 `tests/vehicle_10dof/results/`：

- `Vehicle10DOFVerificationSummary.mat`：静平衡、模态、载荷守恒、快速转向/制动入弯代理、平面回归和闭环指标；
- `Vehicle_7DOF_10DOF_Planar_Comparison.fig`：相同输入下的 7DOF/10DOF 平面输出叠图；
- `Vehicle_10DOF_Dynamic_Wheel_Loads.fig`：组合纵横向阶跃下四轮动态载荷。

当前阻尼基线使用 TTX25 MkII C12/R12 页的 `10-4.3-10-4.3` 曲线，但资料没有确认实车旋钮设置。`Vehicle.InertiaRoll/Pitch` 也仍是暂估值，K&C 外倾输出暂为零。因此 Vehicle10DOF 可用于接口、算法和相对趋势开发，不能作为已标定的实车性能结论。

若只运行组件静平衡回归，可使用 `tests/vehicle_10dof/Vehicle10DOF.feature`。`Vehicle10DOF` 采用 Level-2 MATLAB S-function，模型引用需要 Normal 仿真模式，当前不支持代码生成。

日常整车赛项仿真可直接编辑
`simulation/time_domain_closed_loop/simulation/runLapSimulation.m`：

```matlab
cfg.Vehicle.DynamicsModel = "7DOF";   % 或 "10DOF"
cfg.Driver.Model = "adaptive_autocross"; % 或 "reference_speed"
```

`reference_speed` 使用 `FSAE_TorqueVectoring_ClosedLoop` 或
`FSAE_Vehicle10DOF_ClosedLoop`；`adaptive_autocross` 使用
`FSAE_AdaptiveAutocross_7DOF` 或 `FSAE_AdaptiveAutocross_10DOF`。
四种组合共用 TorqueVectoring 控制器；7DOF 使用 `VehiclePlant`，10DOF 使用
`VehiclePlant10DOF`。结果中的
`Config.Vehicle` 会记录 `DynamicsModel`、`TopModel`、`PlantModel` 和 `CoreModel`。

## 5. 生成三维 GGV 图

### 5.1 创建 QuasiStatic 配置

QuasiStatic 配置会检查必需的有限参数。至少要准备：

- `Vehicle.Mass`、`Wheelbase`、前后轮距、`CGHeight`、`CGToFrontAxle`、`RollStiffnessDistributionFront`；
- `Tire.EffectiveRadius`；
- `Aero.CdA`、`ClAFront`、`ClARear`、`YawAngleGrid`、`YawDownforceScale`；
- `Powertrain.GearRatio`、`GearEfficiency`、`MotorTorqueLimit`、`MotorSpeedLimit`；
- `Battery.PowerLimitDrive`、`PowerLimitRegen`。

```matlab
profile = getSelectedTireProfile();
config = createQuasiStaticConfiguration(parameters, ...
    TireProfile = profile, ...
    TireModel = "MF62", ...
    DynamicsModel = "7DOF", ...       % 或 "10DOF"
    AllocationMode = "LP", ...
    EnvelopePointCount = 48, ...
    KappaGrid = linspace(-0.40, 0.40, 41), ...
    AlphaGrid = linspace(-0.35, 0.35, 41));
```

`AllocationMode="LP"` 适合单点研究和验证；密集 GGV 或大批量扫描可使用 `"Fast"` 加速。`EnvelopePointCount`、`KappaGrid` 和 `AlphaGrid` 是求解分辨率，不是车辆参数。

`DynamicsModel="7DOF"` 保留原有代数载荷转移。`"10DOF"` 使用
`Vehicle10DOFSuspension` 的轮上刚度和附加侧倾刚度，求解零速度的升沉/侧倾/俯仰静态
平衡；阻尼器在该平衡点的相对速度为零，因此阻尼力为零。该选项用于让 GGV
轮荷分配与 Vehicle10DOF 悬架参数一致，不代表 GGV 会计算路面激励或阻尼瞬态。

### 5.2 生成速度相关 GGV

`generateGGV` 会在每个车速、每个横向加速度分数下搜索纵向加速和制动边界：

```matlab
ggv = generateGGV(config, ...
    SpeedGrid = 0:2:40, ...             % m/s
    LateralPointCount = 21, ...
    MaxLateralAcceleration = 20.0, ...  % m/s^2, 搜索上界
    MaxLongitudinalAcceleration = 20.0, ...
    MaxBisectionIterations = 32, ...
    AllocationMode = "Fast");
```

关键输出为：

- `ggv.Speed`：速度网格；
- `ggv.AyPositive`、`ggv.AyNegative`：左右转最大横向加速度；
- `ggv.LateralAcceleration`：每个速度和横向分数对应的 `Ay`；
- `ggv.AxMax`：驱动方向最大 `Ax`；
- `ggv.AxMin`：制动方向最小 `Ax`；
- `ggv.ActiveConstraintMax/Min`：轮胎、功率、电机转矩、制动或电机转速等激活约束。

### 5.3 绘制三维 GGV 曲面

项目内置的 `plotQuasiStaticResults` 主要绘制二维横向边界和能力面；三维图可直接使用 GGV 结构中的矩阵：

```matlab
speed = repmat(ggv.Speed, 1, numel(ggv.LateralFraction));
ay = ggv.LateralAcceleration;

figure("Color", "white");
surf(ay, ggv.AxMax, speed, "FaceAlpha", 0.85, ...
    "EdgeColor", "none", "DisplayName", "Drive boundary");
hold on;
surf(ay, ggv.AxMin, speed, "FaceAlpha", 0.85, ...
    "EdgeColor", "none", "DisplayName", "Braking boundary");
grid on;
xlabel("A_y (m/s^2)");
ylabel("A_x (m/s^2)");
zlabel("Vehicle speed (m/s)");
title("Speed-dependent quasi-steady GGV");
legend("Location", "best");
view(45, 25);
colorbar;

outputFolder = fullfile(pwd, "results", "quasi_static", "ggv");
if ~isfolder(outputFolder), mkdir(outputFolder); end
savefig(gcf, fullfile(outputFolder, "GGV_3D.fig"));
```

图中每个曲面点的横坐标是 `Ay`，纵坐标是可达到的 `Ax`，高度是车速。`AxMax` 曲面表示驱动边界，`AxMin` 曲面表示制动边界。若要先看二维诊断，可将单个 GGV 包装成一个结果记录：

```matlab
ggvResult = struct( ...
    "Success", true, "GGV", ggv, ...
    "PeakAx", max(ggv.AxMax, [], "all", "omitnan"), ...
    "PeakAy", max([ggv.AyPositive; ggv.AyNegative]));
plotQuasiStaticResults(ggvResult, Visible = "on", ...
    OutputFolder = fullfile(pwd, "results", "quasi_static", "ggv", "plots"));
```

### 5.4 从 GGV 计算准静态速度曲线和圈速

对已有 PathTracking 赛道结构，可将 GGV 转为基于曲率的前向加速/后向制动速度曲线：

```matlab
scenario = createPathTrackingAutocrossScenario( ...
    SampleDistance = 0.5, TargetSpeed = 12.0);
track = scenario.Track;

speedProfile = calculateSpeedProfile(track, ggv, ...
    UseReferenceSpeed = false, PassCount = 4);
lap = calculateLapTime(speedProfile, track);

fprintf("Quasi-steady lap time: %.3f s\n", lap.LapTime);
plot(speedProfile.ProgressS, speedProfile.Speed, "LineWidth", 1.2);
grid on; xlabel("Track progress s (m)"); ylabel("Speed (m/s)");
```

闭合赛道会循环进行前向和后向传播；开放赛道可通过 `InitialSpeed` 和 `FinalSpeed` 指定首末速度。该结果没有路径跟踪误差、转向执行器延迟或驾驶员控制误差，应与 PathTracking 时域结果进行对照。

## 6. 准静态参数扫描

### 6.1 设置参数、范围和步长

`createQuasiStaticParameterSweep` 的每一行对应一个参数路径，`Values` 是要扫描的离散值。范围和步长用 MATLAB 冒号表达式指定：

```matlab
sweep = createQuasiStaticParameterSweep( ...
    CdA = 0.80:0.05:1.00, ...
    ClAFront = 1.00:0.10:1.40, ...
    ClARear = 1.00:0.10:1.40, ...
    RollStiffnessDistributionFront = 0.40:0.05:0.60, ...
    GearEfficiency = 0.94:0.01:0.98);
```

参数会形成笛卡尔积。上例有 `5×5×5×5×5 = 3125` 个组合；第一次运行建议每个参数只放 2–3 个值。若必须包含精确数量的点，使用 `linspace(start, stop, pointCount)`，因为 `start:step:stop` 在终点不能整除时不会强行加入终点。

当前 helper 暴露的扫描路径是：

| 名称 | 实际路径 | 单位 |
|---|---|---|
| `CdA` | `Aero.CdA` | `m^2` |
| `ClAFront` | `Aero.ClAFront` | `m^2` |
| `ClARear` | `Aero.ClARear` | `m^2` |
| `RollStiffnessDistributionFront` | `Vehicle.RollStiffnessDistributionFront` | `1` |
| `GearEfficiency` | `Powertrain.GearEfficiency` | `1` |

在 `10DOF` 准静态模式下，前后载荷转移分配由
`Vehicle10DOFSuspension.WheelRate/AdditionalRollStiffness` 决定，扫描
`Vehicle.RollStiffnessDistributionFront` 不会改变结果；应改扫对应的 Vehicle10DOF 悬架字段。

### 6.2 扫描更多参数

`runQuasiStaticParameterSweep` 支持任意至少两层的路径。可以手工建立相同格式的表：

```matlab
sweep = table( ...
    ["Mass"; "MotorTorqueLimit"; "DrivePowerLimit"], ...
    ["Vehicle.Mass"; "Powertrain.MotorTorqueLimit"; ...
     "Battery.PowerLimitDrive"], ...
    {280:10:340; 18:1:24; 60000:5000:90000}, ...
    ["kg"; "N*m"; "W"], ...
    ["Design study"; "Manufacturer range"; "Battery test range"], ...
    VariableNames = ["Name", "Path", "Values", "Unit", "Source"]);
```

注意：当前 `runQuasiStaticParameterSweep` 是对已经归一化的 `config` 做字段替换。`CGToRearAxle` 是在 `createQuasiStaticConfiguration` 中由 `Wheelbase-CGToFrontAxle` 计算的派生量；因此扫描 `Wheelbase` 或 `CGToFrontAxle` 时不能只改一个字段后继续使用原 `config`，应为每个组合重新构造配置，或者同步更新派生量。质量、气动系数、功率限制、效率和侧倾刚度分配等不含此类几何派生关系的参数适合直接扫描。

### 6.3 运行、保存和查看扫参结果

```matlab
outputFolder = fullfile(pwd, "results", "quasi_static", ...
    "parameter_sweep", "ggv");
if ~isfolder(outputFolder), mkdir(outputFolder); end

sweepOptions = struct( ...
    "SpeedGrid", 0:5:40, ...
    "LateralPointCount", 9, ...
    "MaxLateralAcceleration", 16.0, ...
    "MaxLongitudinalAcceleration", 16.0, ...
    "MaxBisectionIterations", 24);

results = runQuasiStaticParameterSweep(config, sweep, ...
    GenerateGGVOptions = sweepOptions, ...
    OutputFile = fullfile(outputFolder, "QuasiStatic_parameter_sweep.mat"), ...
    RunMetadata = struct("ParameterSet", "vehicle_design_v1"));

plotQuasiStaticResults(results, Visible = "on", OutputFolder = outputFolder);

success = [results.Success];
fprintf("Successful cases: %d/%d\n", nnz(success), numel(results));
for k = 1:numel(results)
    if ~results(k).Success
        fprintf("Case %d failed: %s\n", ...
            results(k).Index, results(k).FailureReason);
    end
end
```

每个结果记录保留：

- 实际选中的参数值和 `ParameterTrace`；
- 单位、来源和扫描路径；
- GGV、峰值 `Ax`、峰值 `Ay` 和最大制动能力；
- MATLAB 版本、Git commit、工作树是否 dirty、Variant、求解器和生成时间；
- 失败组合的 `FailureReason`。

不要把失败组合当成零性能；失败必须先检查是负轮荷、轮胎包络、功率限制还是参数缺失。

## 7. 不同场景下的时域闭环仿真

### 7.1 选择生产闭环模型

用于当前 PathTracking 路径跟踪闭环的顶层模型是：

```text
models/top/FSAE_PathTracking_ClosedLoop.slx
```

它连接 `PathTrackingDriver`、`PathTrackingVehicleController`、`PathTrackingSensorModel` 和 `VehiclePlant`。`models/top/FSAE_ClosedLoop.slx` 是早期 ProjectFoundation 接口骨架，默认输出为占位信号，不应作为赛车性能分析入口。

如果 PathTracking 生产模型尚未生成，可运行：

```matlab
buildPathTrackingPathTrackingModels();
```

当前仓库已经包含这些模型时不需要重复构建。

### 7.2 生成场景

项目自带四类可配置场景：

```matlab
acceleration = createPathTrackingAccelerationScenario(TargetSpeed = 30.0);
skidpad = createPathTrackingSkidpadScenario(TargetSpeed = 8.0);
autocross = createPathTrackingAutocrossScenario( ...
    SampleDistance = 0.5, TargetSpeed = 12.0);
endurance = createPathTrackingEnduranceScenario( ...
    NumberOfLaps = 2, SampleDistance = 0.5, TargetSpeed = 12.0);
```

每个场景都包含 `Track`、`Environment`、`InitialState`、`StopTime` 和 `NumberOfLaps`。Endurance 默认从 `scenarios/Endurance/assets/2024_fsec_endurance_track.png` 重新提取中心线；Autocross 暂时将同一几何作为仿真代理使用，并在 `Track.EventUsage` 中明确标识。原图、官方文档 URL、SHA-256 和固定提取参数见 [赛道数据说明](../data/TrackData/README.md)。

自定义赛道可以先用 `makePathTrackingParametricTrack` 生成中心线，也可以传入带红色起点标记的图像，或仿照 `loadPathTrackingAutocrossCsv` 构造同样的赛道结构。赛道弧长 `s` 必须沿行驶方向递增，左弯曲率为正，左右半宽必须是非负值。

默认 Skidpad 是开放的完整赛项路径：20 m 入口直道、右圈两圈、左圈两圈和
20 m 出口直道；入口/出口长度可用 `EntryStraightLength` 和
`ExitStraightLength` 修改。Autocross 与 Endurance 在场景构造时统一为逆时针
方向；若原始闭合轨迹为顺时针，起点保持不变，采样顺序反转，同时左右边界互换。

### 7.3 运行单个闭环场景

下面示例从字典读出参数，选择 MF62 轮胎，然后运行 Autocross：

```matlab
% parameters 由第 3.3 节代码片段读出
scenario = createPathTrackingAutocrossScenario( ...
    SampleDistance = 0.5, TargetSpeed = 12.0);

in = createPathTrackingSimulationInput(scenario, parameters, ...
    ModelName = "FSAE_PathTracking_ClosedLoop", ...
    TireSelection = struct("ModelMode", uint8(1)));
out = sim(in);

runInfo = struct( ...
    "ScenarioName", scenario.ID, ...
    "ParameterSet", "vehicle_dictionary", ...
    "ModelVariant", "PathTracking_MF62", ...
    "Solver", "scenario-defined");
result = collectOpenLoopResults(out, runInfo);

plotOptions = struct( ...
    "Visible", "on", ...
    "SaveFigures", true, ...
    "SaveSummary", true, ...
    "OutputDirectory", fullfile(pwd, "results", "PathTracking", char(scenario.ID)), ...
    "FilePrefix", "autocross_");
figures = plotOpenLoopResults(result, plotOptions);
```

`createPathTrackingSimulationInput` 已经配置了 `StopTime`、`yout`、`logsout`、代数环诊断和求解器诊断。不要用 Base Workspace 临时覆盖 `PathTrackingTrackData` 或 `PathTrackingEnvironment`；场景入口会把它们明确写入顶层模型工作区。

检查结果是否可信：

```matlab
assert(result.Meta.Valid, ...
    "Result invalid: inspect result.Meta for missing or non-finite signals.");
disp(result.Meta.DerivedMetrics);
```

`collectOpenLoopResults` 不会把缺失信号静默补成零。重点检查 `result.Meta.MissingSignals`、`NonFiniteSignals`、`TimeIssue` 和 `Valid`。标准绘图包括全局轨迹、车身状态、四轮运动学、轮胎/轮荷、动力系统、电池和气动。

### 7.4 批量运行多个场景

需要比较不同赛项时，可先生成 `SimulationInput` 数组，再批量仿真：

```matlab
scenarios = { ...
    createPathTrackingAccelerationScenario(TargetSpeed = 30.0), ...
    createPathTrackingSkidpadScenario(TargetSpeed = 8.0), ...
    createPathTrackingAutocrossScenario(TargetSpeed = 12.0), ...
    createPathTrackingEnduranceScenario(NumberOfLaps = 2, TargetSpeed = 12.0)};

inputs(numel(scenarios), 1) = ...
    Simulink.SimulationInput("FSAE_PathTracking_ClosedLoop");
for k = 1:numel(scenarios)
    inputs(k) = createPathTrackingSimulationInput(scenarios{k}, parameters, ...
        ModelName = "FSAE_PathTracking_ClosedLoop", ...
        TireSelection = struct("ModelMode", uint8(1)));
end

outputs = sim(inputs, "UseFastRestart", "on");
% 参数和场景不适合在当前机器上并行时，可改用：
% outputs = parsim(inputs, "UseFastRestart", "on");

summary = table('Size', [numel(scenarios), 4], ...
    'VariableTypes', ["string", "double", "double", "logical"], ...
    'VariableNames', ["Scenario", "MaxAx", "MaxAy", "Valid"]);
for k = 1:numel(scenarios)
    result = collectOpenLoopResults(outputs(k), ...
        struct("ScenarioName", scenarios{k}.ID, ...
        "ParameterSet", "vehicle_dictionary", ...
        "ModelVariant", "PathTracking_MF62"));
    summary.Scenario(k) = scenarios{k}.ID;
    summary.MaxAx(k) = result.Meta.DerivedMetrics.MaxLongitudinalAcceleration;
    summary.MaxAy(k) = result.Meta.DerivedMetrics.MaxLateralAcceleration;
    summary.Valid(k) = result.Meta.Valid;

    scenarioFolder = fullfile(pwd, "results", "PathTracking", char(scenarios{k}.ID));
    plotOpenLoopResults(result, struct( ...
        "Visible", "off", "OutputDirectory", scenarioFolder, ...
        "FilePrefix", string(scenarios{k}.ID) + "_"));
end
disp(summary);
```

批量仿真前要确认每个场景的 `StopTime` 足够覆盖整条赛道或全部圈数。`parsim` 需要 Parallel Computing Toolbox；先用 `sim` 跑通一个场景，再考虑并行。

### 7.5 开环驾驶员测试

若要研究阶跃转向、正弦转向、定转角、加速或制动，可以使用：

```matlab
driverScenarios = createPathTrackingDriverCommandScenarios( ...
    Duration = 8.0, StepSteering = 0.05, ...
    SineSteeringAmplitude = 0.04, SineFrequency = 0.5, ...
    DriveAcceleration = 3.0, BrakeAcceleration = -3.0);
```

该函数生成 `SteeringRackAngle` 和 `LongitudinalAcceleration` 时间序列，适合作为开环驾驶员/控制器测试输入。它不替代路径跟踪场景；用于赛项结果时应优先使用 `createPathTracking*Scenario` 和 `FSAE_PathTracking_ClosedLoop`。

## 8. 常见错误和排查顺序

| 现象 | 原因和处理 |
|---|---|
| `FSAE:PathTrackingInvalidControllerParameter` | `Vehicle.Mass`、`Tire.EffectiveRadius`、`Powertrain.GearRatio`、`GearEfficiency` 或 `MotorTorqueLimit` 仍为 `NaN`；先填有限值并核对单位。 |
| `FSAE:QuasiStatic:PlaceholderParameter` | QuasiStatic 必需参数仍为占位值；查看 `createQuasiStaticConfiguration` 所列的必需字段。 |
| `TireEnvelope` 或提示 TTC Map 没有纵向能力 | GGV 使用了 `TireModel="TTCMap"`；换用 `MF62`，并保留当前 43075/43100 代理数据限制。 |
| 出现负轮荷或大量失败点 | 检查 `CGHeight`、质心纵向位置、侧倾刚度分配、气动平衡和扫描范围。 |
| `result.Meta.Valid=false` | 查看 `MissingSignals`、`NonFiniteSignals` 和 `TimeIssue`；缺失量不能当成零。 |
| 结果只有占位零信号 | 运行的是 `FSAE_ClosedLoop.slx` ProjectFoundation 骨架，或使用了未构建的占位模型；切换到 `FSAE_PathTracking_ClosedLoop`。 |
| 修改轮胎 profile 后模型仍用旧数据 | `getSelectedTireProfile` 使用持久缓存；执行 `clear getSelectedTireProfile` 后再运行。 |
| 扫参耗时过长 | 减少 `SpeedGrid`、`LateralPointCount`、`MaxBisectionIterations` 或参数点数；先用 `Fast`，最终候选点再用 `LP` 复核。 |

## 9. 建议的赛车开发工作流

1. 先在 Data Dictionary 中填入有来源的质量、几何、轮胎有效半径、动力系统和环境数据。
2. 用 `TireSimple` 跑静止、直线和零输入冒烟测试，确认单位、坐标系和初始状态。
3. 切换到 `TireMF62`，先跑 75 m 加速、八字和 Autocross 单场景。
4. 用 `generateGGV` 检查速度相关横向、驱动和制动能力，再用 `calculateSpeedProfile` 得到准静态圈速基线。
5. 对气动平衡、质量、功率限制、传动效率和轮荷转移参数做小范围准静态扫参。
6. 对有潜力的设计点运行 PathTracking 多场景闭环，比较轨迹、路径误差、轮胎利用率、转矩饱和、功率限制和 SOC。
7. 用 PathTracking 时域结果与 QuasiStatic GGV 交叉检查；若明显不一致，先查轮胎、路面附着、速度网格、初始状态和控制器限制，不要直接调大限幅。
8. 每次正式结论同时保存参数来源、模型 Variant、求解器设置、Git commit 和结果有效性状态。

当前阶段的 PathTracking/QuasiStatic 验证使用了合成车辆参数；验证通过只说明接口、求解链和闭环结构可运行，不代表真实赛车性能。只有在关键占位参数和轮胎纵向/联合滑移数据完成实测或辨识后，才可以把结果用于正式赛车设计决策。
