%% FSAE 仿真结果绘图分析主程序
%PLOTLAPSIMULATIONRESULTS 独立读取已保存的圈速结果并绘图。
%   本脚本不重新运行 Simulink。Result.File 为空时，在指定赛事目录下选择
%   最新 lap_simulation_result.mat；否则加载用户指定的结果文件。支持
%   Time/Distance/Track.EventDistance 等横轴，标量和 [FL, FR, RL, RR]
%   四轮纵轴，范围裁剪、m/s↔km/h、rad↔deg、rad/s↔rpm 显示转换，以及
%   MATLAB FIG 导出。
%
%   运行示例：
%       run(fullfile(projectRoot, "simulation", "time_domain_closed_loop", ...
%           "plotting", "plotLapSimulationResults.m"));
%   角度在 MAT 文件内始终保存为 rad，转换只发生在显示层。
%
%   常用可选字段（实际可用项以脚本运行时 listLapResultFields 输出为准）：
%     横轴：Time、Distance、Track.EventDistance、Track.PathS、Track.Progress、
%           Vehicle.X、Vehicle.Y，或任意有效的单通道时序字段。
%     整车：Vehicle.Speed/Ux/Uy/Ax/Ay/Az/YawRate/RollAngle/PitchAngle。
%     赛道：Track.ReferenceSpeed/Curvature/LateralError/BoundaryViolation。
%     四轮：Wheel.Speed/RPM/SteerAngle/NormalLoad/SlipRatio/SlipAngle/CamberAngle，
%           Tire.FxWheel/FyWheel/MuUtilization。
%     动力：Powertrain.MotorTorqueActual/MotorSpeed/MotorRPM/MotorMechanicalPower，
%           Battery.Voltage/Current/Power/SOC。
%     控制：Driver.*、Actuator.*、Controller.*、Sensor.* 中有效的时序字段。
%   四轮字段为 N×4，列顺序固定 [FL, FR, RL, RR]；字段为空时不会伪造零值。

%% 仿真结果绘图参数选择区

% 结果文件；空字符串表示在指定赛事目录中读取最新结果
plotCfg.Result.File = "";

% 赛事目录名称，可选："acceleration" / "skidpad" / "autocross" / "endurance"
plotCfg.Result.Track = "autocross";

% 横轴参数；可选 Time、Distance、Track.EventDistance、Track.PathS、
% Track.Progress、Vehicle.X、Vehicle.Y，以及结果中存在的标量时序字段
plotCfg.X.Parameter = "Distance";

% 横轴显示范围；[NaN NaN] 表示自动范围，单位随横轴字段
plotCfg.X.Range = [0, 50];

% 纵轴参数；每个字段默认使用同一图窗中的一个标签页
plotCfg.Y.Parameters = [
    "Vehicle.Ax"
    "Vehicle.Ay"
    "Wheel.NormalLoad"
];

% 每行依次对应一个纵轴字段；多余行自动忽略，不足行自动补为 [NaN NaN]
% 自动范围，单位随字段
plotCfg.Y.Ranges = [
     0, NaN
];

% 四轮通道，可选："FL" / "FR" / "RL" / "RR"，顺序决定绘图通道
plotCfg.Wheel.Channels = ["FL", "FR", "RL", "RR"];

% 显示单位：速度可选 "m/s" / "km/h"
plotCfg.Units.Speed = "m/s";

% 显示单位：角度可选 "rad" / "deg"
plotCfg.Units.Angle = "deg";

% 显示单位：转速可选 "rad/s" / "rpm"
plotCfg.Units.RotationalSpeed = "rpm";

% 绘图布局，可选："tabs" / "overlay"
plotCfg.Layout = "tabs";

% 是否显示图例和网格
plotCfg.ShowLegend = true;
plotCfg.ShowGrid = true;

% 可选导出：仅支持 MATLAB FIG 格式
plotCfg.Export.Enabled = false;
plotCfg.Export.Formats = "fig";
plotCfg.Export.Directory = ""; % 空 -> <event>/plots/result_selection

%% 读取、校验、列举字段并绘图

projectRoot = string(fileparts(fileparts(fileparts(fileparts( ...
    mfilename("fullpath"))))));
addpath(fullfile(projectRoot, "scripts", "reporting"), "-begin");

resultFile = resolveResultFile(projectRoot, plotCfg.Result);
[result, resultFile] = loadLapSimulationResult(resultFile);
fprintf("已加载结果：%s\n", resultFile);
listLapResultFields(result, Display = true);

if plotCfg.Export.Enabled && strlength(string(plotCfg.Export.Directory)) == 0
    plotCfg.Export.Directory = fullfile(projectRoot, "results", ...
        "time_domain_closed_loop", lower(plotCfg.Result.Track), ...
        "plots", "result_selection");
end

figures = plotLapResultSelection(result, plotCfg);
fprintf("绘图完成：%d 个图窗。\n", numel(figures));

function filePath = resolveResultFile(projectRoot, resultCfg)
if strlength(string(resultCfg.File)) > 0
    filePath = string(resultCfg.File);
    if ~isfile(filePath)
        error("FSAE:Lap:ResultFileMissing", ...
            "Result.File 收到 '%s'，但文件不存在。", filePath);
    end
    return
end
validTracks = ["acceleration", "skidpad", "autocross", "endurance"];
trackName = lower(string(resultCfg.Track));
if ~any(trackName == validTracks)
    error("FSAE:Lap:InvalidResultTrack", ...
        "Result.Track 收到 '%s'，允许值为：%s。", ...
        resultCfg.Track, strjoin(validTracks, ", "));
end
resultRoot = fullfile(projectRoot, "results", "time_domain_closed_loop", ...
    trackName);
candidates = dir(fullfile(resultRoot, "**", "lap_simulation_result.mat"));
if isempty(candidates)
    error("FSAE:Lap:NoSavedResults", ...
        "赛事 '%s' 下没有找到 lap_simulation_result.mat：%s。", ...
        trackName, resultRoot);
end
[~, newest] = max([candidates.datenum]);
filePath = string(fullfile(candidates(newest).folder, candidates(newest).name));
end
