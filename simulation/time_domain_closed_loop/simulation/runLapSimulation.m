%% FSAE 圈速时域仿真主程序
%RUNLAPSIMULATION 使用可选 7DOF/10DOF Plant 执行闭环圈速仿真。
%   本脚本不依赖 Base Workspace 残留变量，会自动初始化 FSAE_Simulation
%   MATLAB Project，生成所选场景、配置 SimulationInput，并在求解完成后显示赛道图窗，
%   整理七组基线或八组 adaptive 顶层 Bus 输出并保存到
%   results/time_domain_closed_loop/<event>。
%
%   输出文件包含：
%       lap_simulation_result.mat  变量名固定为 result
%       run_config.json             运行配置和场景元数据
%       simulation_summary.json     结果摘要和缺失信号
%       final_track_view.fig        可选的 MATLAB FIG 最终赛道视图
%
%   运行示例：
%       run(fullfile(projectRoot, "simulation", "time_domain_closed_loop", ...
%           "simulation", "runLapSimulation.m"));
%   单位：位置/距离 m，时间 s，速度 m/s，加速度 m/s^2，角度 rad。
%   四轮顺序固定为 [FL, FR, RL, RR]。
%
%   配置结构分组与主要字段：
%     Track       - Name、SampleDistance、NumberOfLaps、ImagePath、UseSavedData、DataFolder
%     Vehicle     - TireModel（mf62/ttc_map）、DynamicsModel（7DOF/10DOF）
%     SpeedPlanner- MotorSpeedUtilization、SpeedStep、ConstraintScale、
%                   LateralPointCount、EnvelopePointCount、PassCount、
%                   AllocationMode（fast/lp）、各加速度和搜索迭代上限、UseCache
%     Driver      - Model、路径/边界增益、曲率扫描范围、车辆性能约束
%     Simulation  - StopTime、FinishCheckPeriod、SolverProfile、
%                   SignalLogging、OutputDecimation
%     Visualization- Enabled
%     Output      - RunName、ResultsRoot、SaveRawSimulationOutput、
%                   SaveFinalTrackView、Overwrite
%   正常使用只需修改下方“圈速仿真参数选择区”；result 的可绘图字段包括
%   Track、Vehicle、Wheel、Tire、Powertrain、Battery、Driver、Actuator、
%   Controller 和 Sensor，空缺信号会列入 result.Meta.MissingSignals。

%% 圈速仿真参数选择区

% 图形界面可以在调用 RUN 前提供完整配置；普通脚本运行仍使用下方默认值。
useInjectedConfig = exist("FSAE_GUI_RUN_CONFIG", "var") == 1 && ...
    isstruct(FSAE_GUI_RUN_CONFIG) && isscalar(FSAE_GUI_RUN_CONFIG);
if useInjectedConfig
    cfg = FSAE_GUI_RUN_CONFIG;
    clear FSAE_GUI_RUN_CONFIG
else

% 赛道名称，可选："acceleration" / "skidpad" / "autocross" / "endurance"
cfg.Track.Name = "autocross";

% 赛道离散采样距离，单位 m
cfg.Track.SampleDistance = 0.5;

% 耐久赛圈数；仅 endurance 使用，必须为正整数
cfg.Track.NumberOfLaps = 1;

% 赛道图片路径；空字符串表示使用项目内置 FSEC 耐久赛道原图
cfg.Track.ImagePath = "";

% 保存并复用 trackdata/<赛道名>.mat；几何配置变化时更新对应数据文件
cfg.Track.UseSavedData = true;

% 空字符串表示保存到 <项目>/trackdata；也可指定其他赛道数据目录
cfg.Track.DataFolder = "";

% 轮胎模型，可选："mf62" / "ttc_map"；GGV 规划暂不支持 simple
cfg.Vehicle.TireModel = "mf62";

% 整车动力学模型，可选："7DOF" / "10DOF"
% 7DOF -> FSAE_TorqueVectoring_ClosedLoop/VehiclePlant
% 10DOF -> FSAE_Vehicle10DOF_ClosedLoop/VehiclePlant10DOF
cfg.Vehicle.DynamicsModel = "7DOF";

% GGV 速度规划：速度网格上限由电机最高转速、传动比和轮胎半径自动推导
cfg.SpeedPlanner.MotorSpeedUtilization = 0.95;

% GGV 速度离散间隔，单位 m/s；越小越精细，但首次生成越慢
cfg.SpeedPlanner.SpeedStep = 3.0;

% 使用 GGV 边界的比例；小于 1 为时域跟踪和模型差异保留裕度
cfg.SpeedPlanner.ConstraintScale = 0.90;

% GGV 横向切片数和轮胎包络离散数
cfg.SpeedPlanner.LateralPointCount = 9;
cfg.SpeedPlanner.EnvelopePointCount = 32;

% 沿赛道前向加速/后向制动传播次数
cfg.SpeedPlanner.PassCount = 6;

% GGV 求解配置；FAST 用于规范圈速入口的在线规划
cfg.SpeedPlanner.AllocationMode = "fast";
cfg.SpeedPlanner.MaxLateralAcceleration = 20.0;
cfg.SpeedPlanner.MaxLongitudinalAcceleration = 20.0;
cfg.SpeedPlanner.MaxBisectionIterations = 24;
cfg.SpeedPlanner.LongitudinalBracketPointCount = 25;
cfg.SpeedPlanner.MaxSearchExpansionCount = 3;

% 仅对尖弯增加局部曲率保护，并重做前/后向速度传播
cfg.SpeedPlanner.HighCurvatureThreshold = 0.30;
cfg.SpeedPlanner.HighCurvatureSafetyFactor = 1.50;

% 参数和求解配置不变时复用 cache/speed_planner 下的 GGV
cfg.SpeedPlanner.UseCache = true;

% reference_speed 专用闭环能力标定
cfg.SpeedPlanner.ReferenceMaximumSpeed = 22.5;
cfg.SpeedPlanner.ReferenceMaximumAcceleration = 6.25;
cfg.SpeedPlanner.ReferencePlanningDeceleration = 2.50;
cfg.SpeedPlanner.ReferenceLateralAccelerationLimit = 4.10;
cfg.SpeedPlanner.ReferenceSpeedSafetyFactor = 1.00;

% 驾驶员："adaptive_autocross" 使用 GGV 速度能力上限并在线执行路径/边界控制；
% "reference_speed" 保留原 TorqueVectoringPathTrackingDriver 行为。
cfg.Driver.Model = "adaptive_autocross";

% 最近点横向控制和边界恢复参数
cfg.Driver.HeadingGain = 1.8;
cfg.Driver.CrossTrackGain = 4.0;
cfg.Driver.BoundaryGain = 2.0;
cfg.Driver.MaximumSteeringRate = 6.0;
cfg.Driver.MaximumSteeringAngle = 0.50;
cfg.Driver.MinimumSteeringSpeed = 1.0;
cfg.Driver.ProjectionSearchDistance = 15.0;
cfg.Driver.MinimumSteeringPreview = 1.00;
cfg.Driver.SteeringPreviewTime = 0.18;
cfg.Driver.MaximumSteeringPreview = 4.0;

% 自适应驾驶员能力上限；GGV 仍施加轮胎、动力与制动的实际约束
cfg.Driver.LateralAccelerationLimit = 13.5;
cfg.Driver.MaximumAcceleration = 16.0;
cfg.Driver.MaximumDeceleration = 8.0;
cfg.Driver.PlanningDeceleration = 6.0;
cfg.Driver.MaximumSpeed = 32.0;
cfg.Driver.SpeedPreviewDistance = 190.0;
cfg.Driver.SpeedPreviewStep = 0.75;
cfg.Driver.CurvatureFilterHalfWindow = 2;
cfg.Driver.CurvatureSafetyFactor = 1.0;
cfg.Driver.SpeedSafetyFactor = 1.0;
cfg.Driver.SpeedKp = 4.0;
cfg.Driver.SpeedKi = 0.20;
cfg.Driver.SpeedFeedforwardGain = 0.60;
cfg.Driver.ExitSpeedFeedforwardGain = 0.60;
cfg.Driver.PreviewBrakingEnabled = true;
cfg.Driver.PreviewBrakingActivationDistance = 45.0;
cfg.Driver.PreviewBrakeFeedforwardGain = 0.65;
cfg.Driver.PreviewBrakingDecelerationScale = 0.90;
cfg.Driver.AntiWindupGain = 1.0;
cfg.Driver.GGVConstraintScale = 1.0;
cfg.Driver.MaximumGGVLapTimeRatio = 1.20;
cfg.Driver.RacingLineBoundaryReserve = 0.60;
cfg.Driver.RacingLineMaximumOffset = 0.40;
cfg.Driver.RacingLineOffsetVariationWeight = 0.05;
cfg.Driver.RacingLineMaximumOffsetRate = 0.02;
cfg.Driver.RacingLineLockCurvatureThreshold = 0.25;
cfg.Driver.RacingLineLockBufferDistance = 15.0;
cfg.Driver.RacingLineCurvaturePenaltyWeight = 0.020;
cfg.Driver.RacingLineStartLockDistance = 15.0;
cfg.Driver.ReferencePathCurvatureBlendStart = 0.15;
cfg.Driver.ReferencePathCurvatureBlendEnd = 0.30;
cfg.Driver.ReferencePathMaximumBlend = 0.0;
cfg.Driver.BoundaryPreviewDistance = 6.0;

% 在线边界判定使用前/后轴轮胎外缘；下列值是额外安全净空，单位 m
cfg.Driver.TireSectionWidth = 0.1905; % 7.5 in nominal section width
cfg.Driver.BoundaryReserve = 0.50;
cfg.Driver.EmergencyBoundaryMargin = 0.25;
cfg.Driver.EnableTV = true;
cfg.Driver.EnableTC = true;

% 本入口默认验证 autocross 的 120% GGV 圈速目标；90 s 已超出
% 当前允许上限，未在此前完赛即无需继续计算到场景默认时长。
cfg.Simulation.StopTime = 90.0;

% 终点检测的暂停周期，单位 s；完赛时间由完整输出信号插值，
% 该周期只决定上层调度频率，不改变求解器步长或输出采样精度。
cfg.Simulation.FinishCheckPeriod = 1.00;

% 求解精度："fast" 使用 RelTol=1e-2；"standard" 使用项目默认 1e-3
cfg.Simulation.SolverProfile = "standard";

% 是否额外记录 logsout；七组基线/八组 adaptive 顶层 Bus 已由 yout 保存，默认关闭重复日志
cfg.Simulation.SignalLogging = false;

% 输出抽取倍数；10 表示每 10 个求解器输出点保存一次，不改变积分计算
cfg.Simulation.OutputDecimation = 1;

% 是否在仿真完成后显示最终轨迹图；关闭后用于批处理
cfg.Visualization.Enabled = false;

% 结果运行名；保存前会移除非法路径字符
cfg.Output.RunName = "test";

% 结果根目录；空字符串表示
% <项目根>/results/time_domain_closed_loop
cfg.Output.ResultsRoot = "";

% 是否另存原始 Simulink.SimulationOutput；原始文件可能很大
cfg.Output.SaveRawSimulationOutput = false;

% 是否保存最终赛道图 MATLAB FIG
cfg.Output.SaveFinalTrackView = true;

% 是否允许使用同一时间戳和运行名覆盖已有文件
cfg.Output.Overwrite = false;
end

%% 项目初始化和流程编排

projectRoot = string(fileparts(fileparts(fileparts(fileparts( ...
    mfilename("fullpath"))))));
addpath(fullfile(projectRoot, "scripts", "initialization"), "-begin");
addpath(fullfile(projectRoot, "scripts", "simulation"), "-begin");
addpath(fullfile(projectRoot, "scripts", "track"), "-begin");
addpath(fullfile(projectRoot, "scripts", "tire"), "-begin");
addpath(fullfile(projectRoot, "scripts", "ggv"), "-begin");
addpath(fullfile(projectRoot, "scripts", "lap_time"), "-begin");
addpath(fullfile(projectRoot, "scenarios", "Acceleration"), "-begin");
addpath(fullfile(projectRoot, "scenarios", "Skidpad"), "-begin");
addpath(fullfile(projectRoot, "scenarios", "Autocross"), "-begin");
addpath(fullfile(projectRoot, "scenarios", "Endurance"), "-begin");

cfg = validateRunConfig(cfg);
project = initProject();
projectRoot = string(project.RootFolder);
if cfg.Driver.Model == "adaptive_autocross"
    adaptiveCache = fullfile(projectRoot, "cache", ...
        "adaptive_autocross_session");
    if ~isfolder(adaptiveCache)
        mkdir(adaptiveCache);
    end
    Simulink.fileGenControl("set", ...
        "CacheFolder", adaptiveCache, ...
        "CodeGenFolder", adaptiveCache, ...
        "createDir", true);
end

parameters = readLapVehicleParameters(projectRoot, cfg.Vehicle.DynamicsModel);
scenario = createLapScenario(cfg);
if isfield(scenario.Track, "TrackData") && ...
        scenario.Track.TrackData.Enabled
    if scenario.Track.TrackData.Loaded
        dataAction = "已加载";
    else
        dataAction = "已生成并保存";
    end
    fprintf("%s赛道数据：%s\n", ...
        dataAction, scenario.Track.TrackData.File);
end
if cfg.Driver.Model == "adaptive_autocross"
    racingLineDriver = cfg.Driver;
    racingLineDriver.CGToFrontAxle = ...
        parameterNumericValue(parameters, "Vehicle", "CGToFrontAxle");
    racingLineDriver.CGToRearAxle = ...
        parameterNumericValue(parameters, "Vehicle", "Wheelbase") - ...
        racingLineDriver.CGToFrontAxle;
    racingLineDriver.FrontHalfEnvelope = ...
        0.5 * parameterNumericValue(parameters, "Vehicle", "TrackFront") + ...
        0.5 * cfg.Driver.TireSectionWidth;
    racingLineDriver.RearHalfEnvelope = ...
        0.5 * parameterNumericValue(parameters, "Vehicle", "TrackRear") + ...
        0.5 * cfg.Driver.TireSectionWidth;
    [scenario.Track, racingLineReport] = planAdaptiveRacingLine( ...
        scenario.Track, racingLineDriver);
    scenario.RacingLineMetadata = racingLineReport;
    fprintf("优化赛车线：长度 %.3f -> %.3f m（缩短 %.3f m；" + ...
        "最大偏移 %.3f m；QP 约束违反 %.3g）\n", ...
        racingLineReport.CenterlineLength, ...
        racingLineReport.RacingLineLength, ...
        racingLineReport.LengthReduction, ...
        racingLineReport.MaximumAbsOffset, ...
        racingLineReport.ConstraintViolation);
    [scenario, speedPlan] = applyAdaptiveGGVSpeedCeiling( ...
        scenario, parameters, cfg);
    fprintf("自主驾驶员 GGV 上限：%.3g..%.3g m/s（预测 %.3g s；" + ...
        "原始 GGV %.3g s）\n", ...
        speedPlan.Metadata.MinimumReferenceSpeed, ...
        speedPlan.Metadata.MaximumReferenceSpeed, ...
        speedPlan.Metadata.PredictedLapTime, ...
        speedPlan.Metadata.BenchmarkGGVLapTime);
    if scenario.Track.IsClosed
        maximumAllowedLapTime = cfg.Driver.MaximumGGVLapTimeRatio * ...
            speedPlan.Metadata.BenchmarkGGVLapTime;
        cfg.Simulation.StopTime = min(cfg.Simulation.StopTime, ...
            maximumAllowedLapTime + 5.0);
        fprintf("本次验收停止时间：%.3g s\n", ...
            cfg.Simulation.StopTime, ...
            100.0 * cfg.Driver.MaximumGGVLapTimeRatio);
    else
        fprintf("开放赛道停止时间：%.3g s（终点门线检测将提前停止）\n", ...
            cfg.Simulation.StopTime);
    end
else
    [scenario, speedPlan] = applyGGVReferenceSpeed( ...
        scenario, parameters, cfg);
    fprintf("GGV 参考速度：%.3g..%.3g m/s（%s，预测圈时 %.3g s）\n", ...
        speedPlan.Metadata.MinimumReferenceSpeed, ...
        speedPlan.Metadata.MaximumReferenceSpeed, ...
        speedPlan.Metadata.GGVSource, speedPlan.Metadata.PredictedLapTime);
end
in = createLapSimulationInput(scenario, parameters, cfg);

try
    [simulationOutput, runInfo] = runSimulationWithProgress( ...
        in, scenario, cfg);
    result = collectLapSimulationResults(simulationOutput, scenario, cfg);
    result = deriveVehicleEnvelopeBoundaryMetrics( ...
        result, scenario, parameters, ...
        TireSectionWidth = cfg.Driver.TireSectionWidth);
    if cfg.Driver.Model == "adaptive_autocross"
        result = deriveAdaptivePerformanceMetrics(result, cfg.Driver);
    end
    result.Meta.UserAborted = runInfo.UserAborted;
    result.Meta.Completed = runInfo.Completed;
    result.Meta.StoppedAtFinish = runInfo.StoppedAtFinish;
    result.Meta.TrackIsClosed = logical(scenario.Track.IsClosed);
    result.Meta.SpeedControlMode = cfg.Driver.Model;
    result.Meta.ReferenceSpeedUsed = true;
    if cfg.Driver.Model == "adaptive_autocross"
        result.Meta.GGVSpeedCeilingUsed = true;
        result.Meta.RacingLineUsed = true;
        result.Meta.RacingLine = racingLineReport;
        result.Metrics.GGVLapTime = ...
            speedPlan.Metadata.BenchmarkGGVLapTime;
        result.Metrics.GGVLapTimeRatio = result.Metrics.LapTime / ...
            result.Metrics.GGVLapTime;
        result.Metrics.MaximumAllowedGGVLapTimeRatio = ...
            cfg.Driver.MaximumGGVLapTimeRatio;
        result.Metrics.MaximumAllowedLapTime = ...
            cfg.Driver.MaximumGGVLapTimeRatio * ...
            result.Metrics.GGVLapTime;
        qualification = validateAdaptiveAutocrossResult( ...
            result, ThrowOnFailure = false);
        result.Meta.AdaptiveQualificationPassed = qualification.Passed;
        result.Meta.AdaptiveQualificationFailedChecks = ...
            qualification.FailedChecks;
        result.Metrics.MaximumOutputStep = ...
            qualification.MaximumOutputStep;
    else
        qualification = validateReferenceSpeedResult( ...
            result, ThrowOnFailure = false);
        result.Meta.ReferenceQualificationPassed = qualification.Passed;
        result.Meta.ReferenceQualificationFailedChecks = ...
            qualification.FailedChecks;
        result.Metrics.MaximumOutputStep = ...
            qualification.MaximumOutputStep;
    end
    result.Meta.Valid = result.Meta.Valid && qualification.Passed;
    if cfg.Visualization.Enabled
        displayFinalTrackView(scenario, result, cfg);
    end
    output = saveLapSimulationResults(result, scenario, cfg, ...
        SimulationOutput = simulationOutput);

    fprintf("FSAE 圈速仿真完成。\n");
    fprintf("车辆动力学：%s（%s / %s）\n", ...
        cfg.Vehicle.DynamicsModel, cfg.Vehicle.TopModel, cfg.Vehicle.PlantModel);
    fprintf("结果目录：%s\n", output.Folder);
    fprintf("赛事时间：%.6g s\n", result.Metrics.EventTime);
    fprintf("圈时：%.6g s\n", result.Metrics.LapTime);
    fprintf("圈数：%d/%d\n", maxFinite(result.Track.LapIndex), scenario.NumberOfLaps);
    fprintf("最大合加速度：%.6g m/s^2\n", result.Metrics.MaximumAcceleration);
    fprintf("结果有效性：%s\n", string(result.Meta.Valid));
    fprintf("车辆包络最小净空：%.6g m\n", ...
        result.Metrics.MinimumVehicleEnvelopeClearance);
    fprintf("车辆包络最大越界：%.6g m（%d 点，%s）\n", ...
        result.Metrics.MaximumVehicleEnvelopeViolation, ...
        result.Metrics.VehicleEnvelopeViolationCount, ...
        passFail(result.Meta.VehicleEnvelopeWithinBoundary));
    if cfg.Driver.Model == "adaptive_autocross"
        if scenario.Track.IsClosed
            fprintf("GGV 圈时比：%.6g / %.6g = %.3f（上限 %.3f）\n", ...
                result.Metrics.LapTime, result.Metrics.GGVLapTime, ...
                result.Metrics.GGVLapTimeRatio, ...
                result.Metrics.MaximumAllowedGGVLapTimeRatio);
        else
            fprintf("GGV 时间比：%.6g / %.6g = %.3f（开放赛道诊断项）\n", ...
                result.Metrics.LapTime, result.Metrics.GGVLapTime, ...
                result.Metrics.GGVLapTimeRatio);
        end
        fprintf("自适应驾驶员全圈资格：%s", ...
            passFail(result.Meta.AdaptiveQualificationPassed));
        if ~result.Meta.AdaptiveQualificationPassed
            fprintf("（%s）", strjoin( ...
                result.Meta.AdaptiveQualificationFailedChecks, ", "));
        end
        fprintf("\n");
    else
        fprintf("参考速度驾驶员全圈资格：%s", ...
            passFail(result.Meta.ReferenceQualificationPassed));
        if ~result.Meta.ReferenceQualificationPassed
            fprintf("（%s）", strjoin( ...
                result.Meta.ReferenceQualificationFailedChecks, ", "));
        end
        fprintf("\n");
    end
    if ~isempty(result.Meta.MissingSignals)
        fprintf("缺失信号：%s\n", strjoin(result.Meta.MissingSignals, ", "));
    end
catch exception
    fprintf(2, "FSAE 圈速仿真失败：%s\n", exception.message);
    rethrow(exception);
end

%% 本地流程辅助函数

function [simulationOutput, runInfo] = runSimulationWithProgress( ...
        in, scenario, cfg)
% 使用 R2024a+ Simulation 对象在仿真暂停点检查终点并刷新命令行进度。

runInfo = struct("UserAborted", false, "Completed", false, ...
    "StoppedAtFinish", false);

% 闭合 adaptive 入口一次性求解，避免 Simulation 对象
% 反复暂停/恢复带来的大量调度开销。开放赛道保留终点门线即时停止。
if cfg.Driver.Model == "adaptive_autocross" && ...
        scenario.Track.IsClosed
    simulationOutput = sim(in);
    finalResult = collectLapSimulationResults(simulationOutput, scenario, cfg);
    runInfo.Completed = completedLap(finalResult, scenario);
    return
end

simulationObject = simulation(in);
cleanup = onCleanup(@() cleanupSimulationObject(simulationObject));
initialize(simulationObject);
completionState = struct;
effectiveStopTime = scenario.StopTime;
if isfinite(cfg.Simulation.StopTime)
    effectiveStopTime = cfg.Simulation.StopTime;
end
nextPauseTime = min(cfg.Simulation.FinishCheckPeriod, effectiveStopTime);
stopTimeTolerance = max(1e-9, ...
    10 * eps(max(abs(effectiveStopTime), 1)));
progressBar = createSimulationProgress(scenario);
progressCleanup = onCleanup(@() closeSimulationProgress( ...
    progressBar.Enabled));

while true
    finalStep = step(simulationObject, ...
        PauseTime = nextPauseTime);
    status = string(simulationObject.Status);
    partialOutput = simulationObject.SimulationOutput;
    partialResult = collectLapSimulationSnapshot(partialOutput);
    [completionState, eventComplete] = updateLapCompletionState( ...
        completionState, scenario, partialResult);
    stopTimeReached = nextPauseTime >= ...
        effectiveStopTime - stopTimeTolerance;
    forceProgressUpdate = eventComplete || finalStep || ...
        status == "inactive" || stopTimeReached;
    progressBar = updateSimulationProgress(progressBar, ...
        completionState, partialResult, eventComplete, forceProgressUpdate);
    if eventComplete
        if status == "inactive"
            simulationOutput = partialOutput;
        else
            simulationOutput = stop(simulationObject);
        end
        runInfo.Completed = true;
        runInfo.StoppedAtFinish = true;
        return
    end
    if finalStep || status == "inactive" || stopTimeReached
        break
    end
    nextPauseTime = min(nextPauseTime + ...
        cfg.Simulation.FinishCheckPeriod, effectiveStopTime);
end

if string(simulationObject.Status) == "inactive"
    simulationOutput = simulationObject.SimulationOutput;
else
    simulationOutput = stop(simulationObject);
end
runInfo.Completed = completionState.Completed && ~runInfo.UserAborted;
end

function progressBar = createSimulationProgress(scenario)
% 创建命令行仿真进度条并输出初始状态。

targetLaps = 1;
if scenario.Track.IsClosed
    targetLaps = double(scenario.NumberOfLaps);
end
progressBar = struct( ...
    "Enabled", true, ...
    "Width", 30, ...
    "UpdatePeriod", 0.25, ...
    "Timer", [], ...
    "LastTextLength", 0, ...
    "TargetDistance", double(scenario.Track.Length) * targetLaps);
if progressBar.Enabled
    progressBar = renderSimulationProgress(progressBar, 0, 0);
end
end

function progressBar = updateSimulationProgress(progressBar, ...
        completionState, snapshot, eventComplete, forceUpdate)
% 按赛程距离显示完成比例，并限制命令行刷新频率以免拖慢仿真。

if ~progressBar.Enabled || (~forceUpdate && ...
        toc(progressBar.Timer) < progressBar.UpdatePeriod)
    return
end

eventDistance = 0;
fraction = 0;
if isfield(completionState, "EventDistance") && ...
        isfinite(completionState.EventDistance)
    eventDistance = max(0, double(completionState.EventDistance));
end
if isfield(completionState, "Progress") && ...
        isfinite(completionState.Progress)
    fraction = min(max(double(completionState.Progress), 0), 1);
end
if eventComplete
    eventDistance = progressBar.TargetDistance;
    fraction = 1;
end

simulationTime = 0;
if ~isempty(snapshot.Time) && isfinite(snapshot.Time(end))
    simulationTime = double(snapshot.Time(end));
end
progressBar = renderSimulationProgress(progressBar, ...
    fraction, simulationTime, eventDistance);
end

function progressBar = renderSimulationProgress(progressBar, ...
        fraction, simulationTime, eventDistance)
% 使用退格符原地重绘纯 ASCII 进度条，兼容 MATLAB 命令行窗口。

arguments
    progressBar (1, 1) struct
    fraction (1, 1) double
    simulationTime (1, 1) double
    eventDistance (1, 1) double = 0
end

filledWidth = min(progressBar.Width, ...
    floor(fraction * progressBar.Width));
barText = [repmat('#', 1, filledWidth), ...
    repmat('-', 1, progressBar.Width - filledWidth)];
progressText = sprintf( ...
    "Simulation [%s] %6.2f%% | %8.1f/%8.1f m | t=%8.2f s", ...
    barText, 100 * fraction, eventDistance, ...
    progressBar.TargetDistance, simulationTime);
if progressBar.LastTextLength > 0
    fprintf(1, "%s", repmat(sprintf('\b'), 1, ...
        progressBar.LastTextLength));
end
fprintf(1, "%s", progressText);
progressBar.LastTextLength = strlength(progressText);
progressBar.Timer = tic;
end

function closeSimulationProgress(enabled)
% 保证正常返回或异常退出后，后续命令行信息从新行开始。

if enabled
    fprintf(1, "\n");
end
end

function displayFinalTrackView(scenario, result, cfg)
% 最终图显示失败不应阻止已完成的仿真结果保存。

try
    createFinalTrackViewFigure(scenario, result, cfg, Visible = "on");
catch exception
    warning("FSAE:Lap:FinalViewDisplayFailed", ...
        "最终赛道图显示失败，仿真结果仍将保存：%s", exception.message);
end
end

function cleanupSimulationObject(simulationObject)
try
    status = string(simulationObject.Status);
    if any(status == ["running", "paused", "initialized", "initializing"])
        terminate(simulationObject);
    end
catch
    % 清理阶段不覆盖原始仿真异常。
end
end

function parameters = readLapVehicleParameters(projectRoot, dynamicsModel)
dictionaryPath = fullfile(projectRoot, "data", "VehicleData.sldd");
assert(isfile(dictionaryPath), "FSAE:Lap:MissingDictionary", ...
    "车辆数据字典不存在：%s。", dictionaryPath);
dictionary = Simulink.data.dictionary.open(dictionaryPath);
cleanup = onCleanup(@() close(dictionary));
designData = getSection(dictionary, "Design Data");
groups = ["Vehicle", "Tire", "Aero", "Powertrain", "Battery", "Brake"];
parameters = struct;
for groupName = groups
    entry = getEntry(designData, char(groupName));
    parameters.(char(groupName)) = getValue(entry);
end
if ~isfield(parameters.Brake, "MaxTotalForce")
    parameters.Brake.MaxTotalForce = Inf;
end
selection = resolveVehicleDynamicsModel(dynamicsModel);
if selection.Name == "10DOF"
    for groupName = ["Vehicle10DOFSuspension", "Vehicle10DOFInitialState"]
        try
            entry = getEntry(designData, char(groupName));
        catch exception
            error("FSAE:Lap:Missing10DOFParameter", ...
                "10DOF 仿真缺少数据字典条目 %s：%s", ...
                groupName, exception.message);
        end
        parameters.(char(groupName)) = getValue(entry);
    end
end
end

function cfg = validateRunConfig(cfg)
if ~isfield(cfg, "Track") || ~isfield(cfg, "Vehicle") || ...
        ~isfield(cfg, "SpeedPlanner") || ~isfield(cfg, "Driver") || ...
        ~isfield(cfg, "Simulation") || ~isfield(cfg, "Visualization") || ...
        ~isfield(cfg, "Output")
    error("FSAE:Lap:MissingConfigGroup", ...
        "cfg 必须包含 Track、Vehicle、SpeedPlanner、Driver、Simulation、Visualization、Output 七组。");
end
if ~isfield(cfg.Vehicle, "DynamicsModel")
    error("FSAE:Lap:MissingDynamicsModel", ...
        "Vehicle.DynamicsModel 必须为 7DOF 或 10DOF。");
end
dynamicsSelection = resolveVehicleDynamicsModel( ...
    string(cfg.Vehicle.DynamicsModel));
cfg.Vehicle.DynamicsModel = dynamicsSelection.Name;
cfg.Vehicle.TopModel = dynamicsSelection.TopModel;
cfg.Vehicle.PlantModel = dynamicsSelection.PlantModel;
cfg.Vehicle.CoreModel = dynamicsSelection.CoreModel;
if ~isfield(cfg.Driver, "Model")
    cfg.Driver.Model = "reference_speed";
end
cfg.Driver.Model = lower(string(cfg.Driver.Model));
if ~isscalar(cfg.Driver.Model) || ...
        ~any(cfg.Driver.Model == ["reference_speed", "adaptive_autocross"])
    error("FSAE:Lap:InvalidDriverModel", ...
        "Driver.Model 必须为 reference_speed 或 adaptive_autocross。");
end
if cfg.Driver.Model == "adaptive_autocross"
    cfg.Vehicle.TopModel = ...
        "FSAE_AdaptiveAutocross_" + cfg.Vehicle.DynamicsModel;
else
    cfg.SpeedPlanner = validateSpeedPlannerConfig( ...
        cfg.SpeedPlanner, cfg.Vehicle.TireModel);
end
for fieldName = ["HeadingGain", "CrossTrackGain", ...
        "MaximumSteeringRate", "MaximumDeceleration"]
    if ~isfield(cfg.Driver, fieldName)
        error("FSAE:Lap:MissingDriverConfig", ...
            "cfg.Driver 缺少字段 %s。", fieldName);
    end
    value = cfg.Driver.(char(fieldName));
    if ~isscalar(value) || ~isfinite(value) || value <= 0
        error("FSAE:Lap:InvalidDriverConfig", ...
            "Driver.%s 收到 %g，必须为正有限数。", fieldName, value);
    end
end
if cfg.Driver.Model == "adaptive_autocross"
    cfg.Driver = validateAdaptiveDriverConfig(cfg.Driver);
end

if ~isfield(cfg.Simulation, "StopTime") || ~isscalar(cfg.Simulation.StopTime) || ...
        ~(isnan(cfg.Simulation.StopTime) || ...
        (isfinite(cfg.Simulation.StopTime) && cfg.Simulation.StopTime > 0))
    error("FSAE:Lap:InvalidStopTime", ...
        "Simulation.StopTime 收到 %g，必须为 NaN 或正数（单位：s）。", ...
        cfg.Simulation.StopTime);
end
if ~isfield(cfg.Simulation, "FinishCheckPeriod") || ...
        ~isscalar(cfg.Simulation.FinishCheckPeriod) || ...
        ~isfinite(cfg.Simulation.FinishCheckPeriod) || ...
        cfg.Simulation.FinishCheckPeriod <= 0
    error("FSAE:Lap:InvalidFinishCheckPeriod", ...
        "Simulation.FinishCheckPeriod 收到 %g，必须为正有限数（单位：s）。", ...
        cfg.Simulation.FinishCheckPeriod);
end
if ~isfield(cfg.Simulation, "SolverProfile")
    error("FSAE:Lap:MissingSolverProfile", ...
        "Simulation.SolverProfile 必须为 fast 或 standard。");
end
cfg.Simulation.SolverProfile = lower(string(cfg.Simulation.SolverProfile));
if ~isscalar(cfg.Simulation.SolverProfile) || ...
        ~any(cfg.Simulation.SolverProfile == ["fast", "standard"])
    error("FSAE:Lap:InvalidSolverProfile", ...
        "Simulation.SolverProfile 收到 '%s'，允许值为 fast 或 standard。", ...
        cfg.Simulation.SolverProfile);
end
if ~isfield(cfg.Simulation, "SignalLogging") || ...
        ~islogical(cfg.Simulation.SignalLogging) || ...
        ~isscalar(cfg.Simulation.SignalLogging)
    error("FSAE:Lap:InvalidSignalLogging", ...
        "Simulation.SignalLogging 必须为 logical 标量。");
end
if ~isfield(cfg.Simulation, "OutputDecimation") || ...
        ~isscalar(cfg.Simulation.OutputDecimation) || ...
        ~isfinite(cfg.Simulation.OutputDecimation) || ...
        cfg.Simulation.OutputDecimation < 1 || ...
        cfg.Simulation.OutputDecimation ~= round(cfg.Simulation.OutputDecimation)
    error("FSAE:Lap:InvalidOutputDecimation", ...
        "Simulation.OutputDecimation 收到 %g，必须为正整数。", ...
        cfg.Simulation.OutputDecimation);
end
if ~isfield(cfg.Output, "RunName") || strlength(string(cfg.Output.RunName)) == 0
    error("FSAE:Lap:InvalidRunName", "Output.RunName 不能为空。");
end
if ~islogical(cfg.Visualization.Enabled) || ~isscalar(cfg.Visualization.Enabled)
    error("FSAE:Lap:InvalidLogicalOption", ...
        "Visualization.Enabled 必须为 logical 标量。");
end
end

function driver = validateAdaptiveDriverConfig(driver)
previewDefaults = struct( ...
    "PreviewBrakingEnabled", false, ...
    "PreviewBrakingActivationDistance", 45.0, ...
    "PreviewBrakeFeedforwardGain", 0.65, ...
    "PreviewBrakingDecelerationScale", 0.90, ...
    "EnableTC", false);
previewNames = string(fieldnames(previewDefaults));
for fieldName = previewNames.'
    if ~isfield(driver, char(fieldName))
        driver.(char(fieldName)) = previewDefaults.(char(fieldName));
    end
end
positiveFields = ["BoundaryGain", "MaximumSteeringAngle", ...
    "MinimumSteeringSpeed", "ProjectionSearchDistance", ...
    "MinimumSteeringPreview", "SteeringPreviewTime", ...
    "MaximumSteeringPreview", "LateralAccelerationLimit", ...
    "MaximumAcceleration", "PlanningDeceleration", "MaximumSpeed", ...
    "SpeedPreviewDistance", ...
    "SpeedPreviewStep", "CurvatureSafetyFactor", "SpeedSafetyFactor", ...
    "SpeedKp", "SpeedKi", "SpeedFeedforwardGain", ...
    "ExitSpeedFeedforwardGain", "PreviewBrakingActivationDistance", ...
    "PreviewBrakeFeedforwardGain", "PreviewBrakingDecelerationScale", ...
    "AntiWindupGain", "BoundaryReserve", ...
    "EmergencyBoundaryMargin", "TireSectionWidth", ...
    "GGVConstraintScale", "MaximumGGVLapTimeRatio", ...
    "RacingLineBoundaryReserve", "RacingLineMaximumOffset", ...
    "RacingLineOffsetVariationWeight", ...
    "RacingLineMaximumOffsetRate", ...
    "RacingLineLockCurvatureThreshold", ...
    "RacingLineLockBufferDistance", ...
    "RacingLineCurvaturePenaltyWeight", ...
    "RacingLineStartLockDistance", ...
    "ReferencePathCurvatureBlendStart", ...
    "ReferencePathCurvatureBlendEnd", ...
    "BoundaryPreviewDistance"];
missing = positiveFields(~isfield(driver, cellstr(positiveFields)));
if ~isempty(missing)
    error("FSAE:AAD:MissingDriverConfig", ...
        "adaptive_autocross 驾驶员缺少字段：%s。", strjoin(missing, ", "));
end
for fieldName = positiveFields
    value = driver.(char(fieldName));
    if ~isscalar(value) || ~isfinite(value) || value <= 0
        error("FSAE:AAD:InvalidDriverConfig", ...
            "Driver.%s 必须为正有限标量。", fieldName);
    end
end
if driver.MinimumSteeringPreview > driver.MaximumSteeringPreview
    error("FSAE:AAD:InvalidSteeringPreview", ...
        "MinimumSteeringPreview 不得大于 MaximumSteeringPreview。");
end
if driver.SpeedSafetyFactor > 1
    error("FSAE:AAD:InvalidSpeedSafetyFactor", ...
        "SpeedSafetyFactor 必须位于 (0, 1]。 ");
end
if driver.CurvatureSafetyFactor < 1
    error("FSAE:AAD:InvalidCurvatureSafetyFactor", ...
        "CurvatureSafetyFactor 必须大于或等于 1。");
end

if driver.GGVConstraintScale > 1
    error("FSAE:AAD:InvalidGGVConstraintScale", ...
        "GGVConstraintScale 必须位于 (0, 1]。");
end
if driver.SpeedFeedforwardGain > 1 || ...
        driver.ExitSpeedFeedforwardGain > 1 || ...
        driver.ExitSpeedFeedforwardGain < driver.SpeedFeedforwardGain
    error("FSAE:AAD:InvalidSpeedFeedforwardGain", ...
        "速度前馈增益必须位于 (0, 1]，且出弯增益不得低于基础增益。");
end
if driver.PreviewBrakeFeedforwardGain > 1 || ...
        driver.PreviewBrakingDecelerationScale > 1
    error("FSAE:AAD:InvalidPreviewBrakingGain", ...
        "预见制动前馈增益和减速度比例必须位于 (0, 1]。");
end
if ~islogical(driver.PreviewBrakingEnabled) || ...
        ~isscalar(driver.PreviewBrakingEnabled)
    error("FSAE:AAD:InvalidPreviewBrakingEnable", ...
        "Driver.PreviewBrakingEnabled 必须为 logical 标量。");
end
if driver.ReferencePathCurvatureBlendStart >= ...
        driver.ReferencePathCurvatureBlendEnd
    error("FSAE:AAD:InvalidReferencePathCurvatureBlend", ...
        "ReferencePathCurvatureBlendStart 必须小于 BlendEnd。");
end
if ~isfield(driver, "ReferencePathMaximumBlend") || ...
        ~isscalar(driver.ReferencePathMaximumBlend) || ...
        ~isfinite(driver.ReferencePathMaximumBlend) || ...
        driver.ReferencePathMaximumBlend < 0 || ...
        driver.ReferencePathMaximumBlend > 1
    error("FSAE:AAD:InvalidReferencePathMaximumBlend", ...
        "ReferencePathMaximumBlend 必须位于 [0, 1]。");
end
if driver.PlanningDeceleration > driver.MaximumDeceleration
    error("FSAE:AAD:InvalidPlanningDeceleration", ...
        "PlanningDeceleration 不得大于 MaximumDeceleration。");
end
if driver.SpeedPreviewDistance / driver.SpeedPreviewStep > 255
    error("FSAE:AAD:PreviewCapacityExceeded", ...
        "SpeedPreviewDistance/SpeedPreviewStep 不得大于 255。");
end
if ~isfield(driver, "CurvatureFilterHalfWindow") || ...
        driver.CurvatureFilterHalfWindow < 0 || ...
        driver.CurvatureFilterHalfWindow ~= ...
        round(driver.CurvatureFilterHalfWindow) || ...
        driver.CurvatureFilterHalfWindow > 20
    error("FSAE:AAD:InvalidCurvatureWindow", ...
        "CurvatureFilterHalfWindow 必须为 0..20 的整数。");
end
if ~isfield(driver, "EnableTV") || ...
        ~islogical(driver.EnableTV) || ~isscalar(driver.EnableTV)
    error("FSAE:AAD:InvalidEnableTV", ...
        "Driver.EnableTV 必须为 logical 标量。");
end
if ~islogical(driver.EnableTC) || ~isscalar(driver.EnableTC)
    error("FSAE:AAD:InvalidEnableTC", ...
        "Driver.EnableTC 必须为 logical 标量。");
end
end

function value = parameterNumericValue(parameters, groupName, fieldName)
assert(isfield(parameters, groupName) && ...
    isfield(parameters.(groupName), fieldName), ...
    "FSAE:Lap:MissingParameter", ...
    "Missing parameter %s.%s.", groupName, fieldName);
value = parameters.(groupName).(fieldName);
if isstruct(value) && isfield(value, "Value")
    value = value.Value;
end
assert(isnumeric(value) && isscalar(value) && isfinite(value), ...
    "FSAE:Lap:InvalidParameter", ...
    "Parameter %s.%s must be finite and scalar.", groupName, fieldName);
value = double(value);
end

function completed = completedLap(result, scenario)
completed = false;
if ~isfield(result, "Track") || ...
        ~isfield(result.Track, "EventDistance") || ...
        isempty(result.Track.EventDistance)
    return
end
targetDistance = double(scenario.Track.Length);
if scenario.Track.IsClosed
    targetDistance = targetDistance * double(scenario.NumberOfLaps);
end
distance = double(result.Track.EventDistance(:));
tolerance = sqrt(eps) * max(targetDistance, 1.0);
completed = any(isfinite(distance) & distance >= targetDistance - tolerance);
end

function [scenario, plan] = applyAdaptiveGGVSpeedCeiling( ...
        scenario, parameters, cfg)
plannerCfg = cfg;
plannerCfg.SpeedPlanner.ConstraintScale = cfg.Driver.GGVConstraintScale;
plannerCfg.SpeedPlanner.ReferenceMaximumSpeed = cfg.Driver.MaximumSpeed;
plannerCfg.SpeedPlanner.ReferenceMaximumAcceleration = ...
    cfg.Driver.MaximumAcceleration;
plannerCfg.SpeedPlanner.ReferencePlanningDeceleration = ...
    cfg.Driver.PlanningDeceleration;
plannerCfg.SpeedPlanner.ReferenceLateralAccelerationLimit = ...
    cfg.Driver.LateralAccelerationLimit;
plannerCfg.SpeedPlanner.ReferenceSpeedSafetyFactor = 1.0;
[scenario, plan] = applyGGVReferenceSpeed( ...
    scenario, parameters, plannerCfg);
scenario.ReferenceSpeedSource = ...
    "GGV speed-planning line with centerline path and boundary control";
scenario.Track.ReferenceSpeedSource = scenario.ReferenceSpeedSource;
end

function planner = validateSpeedPlannerConfig(planner, tireModel)
required = ["MotorSpeedUtilization", "SpeedStep", "ConstraintScale", ...
    "LateralPointCount", "EnvelopePointCount", "PassCount", ...
    "AllocationMode", "MaxLateralAcceleration", ...
    "MaxLongitudinalAcceleration", "MaxBisectionIterations", ...
    "LongitudinalBracketPointCount", "MaxSearchExpansionCount", ...
    "UseCache", "ReferenceMaximumSpeed", ...
    "ReferenceMaximumAcceleration", "ReferencePlanningDeceleration", ...
    "ReferenceLateralAccelerationLimit", "ReferenceSpeedSafetyFactor"];
missing = required(~isfield(planner, cellstr(required)));
if ~isempty(missing)
    error("FSAE:Lap:MissingSpeedPlannerConfig", ...
        "cfg.SpeedPlanner 缺少字段：%s。", strjoin(missing, ", "));
end
if ~any(lower(string(tireModel)) == ["mf62", "ttc_map"])
    error("FSAE:Lap:GGVTireModel", ...
        "GGV 速度规划要求 Vehicle.TireModel 为 mf62 或 ttc_map。");
end
for fieldName = ["SpeedStep", "MaxLateralAcceleration", ...
        "MaxLongitudinalAcceleration", "ReferenceMaximumSpeed", ...
        "ReferenceMaximumAcceleration", "ReferencePlanningDeceleration", ...
        "ReferenceLateralAccelerationLimit"]
    value = planner.(char(fieldName));
    if ~isscalar(value) || ~isfinite(value) || value <= 0
        error("FSAE:Lap:InvalidSpeedPlannerValue", ...
            "SpeedPlanner.%s 必须为正有限标量。", fieldName);
    end
end
for fieldName = ["MotorSpeedUtilization", "ConstraintScale", ...
        "ReferenceSpeedSafetyFactor"]
    value = planner.(char(fieldName));
    if ~isscalar(value) || ~isfinite(value) || value <= 0 || value > 1
        error("FSAE:Lap:InvalidSpeedPlannerScale", ...
            "SpeedPlanner.%s 必须在 (0, 1] 内。", fieldName);
    end
end
for fieldName = ["LateralPointCount", "EnvelopePointCount", ...
        "PassCount", "MaxBisectionIterations", ...
        "LongitudinalBracketPointCount"]
    value = planner.(char(fieldName));
    if ~isscalar(value) || ~isfinite(value) || value < 1 || ...
            value ~= round(value)
        error("FSAE:Lap:InvalidSpeedPlannerInteger", ...
            "SpeedPlanner.%s 必须为正整数。", fieldName);
    end
end
if planner.LateralPointCount < 3 || planner.EnvelopePointCount < 12 || ...
        planner.LongitudinalBracketPointCount < 9 || ...
        mod(planner.LongitudinalBracketPointCount, 2) == 0
    error("FSAE:Lap:InvalidSpeedPlannerResolution", ...
        "GGV 分辨率要求 LateralPointCount>=3、EnvelopePointCount>=12，" + ...
        "且 LongitudinalBracketPointCount>=9 并为奇数。");
end
if ~isscalar(planner.MaxSearchExpansionCount) || ...
        ~isfinite(planner.MaxSearchExpansionCount) || ...
        planner.MaxSearchExpansionCount < 0 || ...
        planner.MaxSearchExpansionCount ~= round(planner.MaxSearchExpansionCount)
    error("FSAE:Lap:InvalidSpeedPlannerExpansion", ...
        "SpeedPlanner.MaxSearchExpansionCount 必须为非负整数。");
end
planner.AllocationMode = lower(string(planner.AllocationMode));
if ~isscalar(planner.AllocationMode) || ...
        ~any(planner.AllocationMode == ["fast", "lp"])
    error("FSAE:Lap:InvalidSpeedPlannerAllocation", ...
        "SpeedPlanner.AllocationMode 必须为 fast 或 lp。");
end
if ~islogical(planner.UseCache) || ~isscalar(planner.UseCache)
    error("FSAE:Lap:InvalidSpeedPlannerCache", ...
        "SpeedPlanner.UseCache 必须为 logical 标量。");
end
end

function value = maxFinite(data)
if isempty(data)
    value = NaN;
else
    finiteData = double(data(isfinite(data)));
    if isempty(finiteData)
        value = NaN;
    else
        value = max(finiteData);
    end
end
end

function label = passFail(condition)
if condition
    label = "PASS";
else
    label = "FAIL";
end
end
