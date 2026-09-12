function figures = plotOpenLoopResults(result, options)
%PLOTOPENLOOPRESULTS Plot and export normalized OpenLoopPlant open-loop results.

arguments
    result (1, 1) struct
    options struct = struct
end

options = applyDefaults(options, result);
for groupName = ["Vehicle", "Wheel", "Tire", "Powertrain", "Battery", "Aero"]
    if ~isfield(result, groupName)
        result.(groupName) = struct;
    end
end
wheelNames = ["FL", "FR", "RL", "RR"];
colors = [0.0000 0.4470 0.7410; 0.8500 0.3250 0.0980; ...
    0.4660 0.6740 0.1880; 0.6350 0.0780 0.1840];
figures = struct("ExportedFiles", strings(0, 1));
figures.Main = figure("Name", "Open-loop simulation results", ...
    "NumberTitle", "off", "Visible", options.Visible, ...
    "Color", "white", "WindowStyle", "normal");
options.FigureHandle = figures.Main;
options.TabGroup = uitabgroup("Parent", figures.Main);

if hasAny(result.Vehicle, ["X", "Y"])
    [figures.Trajectory, axesHandles] = makeFigureSet( ...
        "Trajectory", "Global trajectory", options);
    ax = axesHandles(1);
    vehicleX = getFieldOrEmpty(result.Vehicle, "X");
    vehicleY = getFieldOrEmpty(result.Vehicle, "Y");
    if ~isempty(vehicleX) && ~isempty(vehicleY)
        plot(ax, vehicleX, vehicleY, "LineWidth", 1.4);
        axis(ax, "equal");
        xlabel(ax, "X (m)"); ylabel(ax, "Y (m)");
    end
    title(ax, "Global trajectory"); grid(ax, "on");
end

if hasAny(result.Vehicle, ["Ux", "Uy", "YawRate", "Ax", "Ay"])
    plotNames = ["Ux", "Uy", "Yaw rate", "Ax", "Ay"];
    [figures.VehicleState, axesHandles] = makeFigureSet( ...
        "Vehicle state", plotNames, options);
    plotScalar(axesHandles(1), result.Time, ...
        getFieldOrEmpty(result.Vehicle, "Ux"), "Ux (m/s)");
    plotScalar(axesHandles(2), result.Time, ...
        getFieldOrEmpty(result.Vehicle, "Uy"), "Uy (m/s)");
    plotScalar(axesHandles(3), result.Time, ...
        getFieldOrEmpty(result.Vehicle, "YawRate"), "Yaw rate (rad/s)");
    plotScalar(axesHandles(4), result.Time, ...
        getFieldOrEmpty(result.Vehicle, "Ax"), "Ax (m/s^2)");
    plotScalar(axesHandles(5), result.Time, ...
        getFieldOrEmpty(result.Vehicle, "Ay"), "Ay (m/s^2)");
end

if hasAny(result.Wheel, ["Speed", "SlipRatio", "SlipAngle"])
    plotNames = ["Wheel speed", "Slip ratio", "Slip angle"];
    [figures.WheelKinematics, axesHandles] = makeFigureSet( ...
        "Wheel kinematics", plotNames, options);
    plotWheel(axesHandles(1), result.Time, ...
        getFieldOrEmpty(result.Wheel, "Speed"), ...
        "Wheel speed (rad/s)", wheelNames, colors);
    plotWheel(axesHandles(2), result.Time, ...
        getFieldOrEmpty(result.Wheel, "SlipRatio"), ...
        "Slip ratio (1)", wheelNames, colors);
    plotWheel(axesHandles(3), result.Time, ...
        getFieldOrEmpty(result.Wheel, "SlipAngle"), ...
        "Slip angle (rad)", wheelNames, colors);
end

if hasAny(result.Tire, ["FxWheel", "FyWheel", "MuUtilization"]) || ...
        ~isempty(getFieldOrEmpty(result.Wheel, "NormalLoad"))
    plotNames = ["Normal load", "Longitudinal force", ...
        "Lateral force", "Friction utilization"];
    [figures.TireLoad, axesHandles] = makeFigureSet( ...
        "Tire load", plotNames, options);
    plotWheel(axesHandles(1), result.Time, ...
        getFieldOrEmpty(result.Wheel, "NormalLoad"), ...
        "Normal load (N)", wheelNames, colors);
    plotWheel(axesHandles(2), result.Time, ...
        getFieldOrEmpty(result.Tire, "FxWheel"), ...
        "Fx wheel (N)", wheelNames, colors);
    plotWheel(axesHandles(3), result.Time, ...
        getFieldOrEmpty(result.Tire, "FyWheel"), ...
        "Fy wheel (N)", wheelNames, colors);
    plotWheel(axesHandles(4), result.Time, ...
        getFieldOrEmpty(result.Tire, "MuUtilization"), ...
        "Friction utilization (1)", wheelNames, colors);
end

if hasAny(result.Powertrain, ["MotorTorqueActual", "MotorSpeed", "MotorMechanicalPower"])
    plotNames = ["Motor torque", "Motor speed", "Motor mechanical power"];
    [figures.Powertrain, axesHandles] = makeFigureSet( ...
        "Powertrain", plotNames, options);
    plotWheel(axesHandles(1), result.Time, ...
        getFieldOrEmpty(result.Powertrain, "MotorTorqueActual"), ...
        "Motor torque actual (N*m)", wheelNames, colors);
    plotWheel(axesHandles(2), result.Time, ...
        getFieldOrEmpty(result.Powertrain, "MotorSpeed"), ...
        "Motor speed (rad/s)", wheelNames, colors);
    plotWheel(axesHandles(3), result.Time, ...
        getFieldOrEmpty(result.Powertrain, "MotorMechanicalPower"), ...
        "Motor mechanical power (W)", wheelNames, colors);
end

if hasAny(result.Battery, ["AllowedElectricalPower", "Current", "SOC"])
    plotNames = ["Allowed electrical power", "Current", "SOC", ...
        "Drive clipped power"];
    [figures.Battery, axesHandles] = makeFigureSet( ...
        "Battery", plotNames, options);
    plotScalar(axesHandles(1), result.Time, ...
        getFieldOrEmpty(result.Battery, "AllowedElectricalPower"), ...
        "Allowed electrical power (W)");
    plotScalar(axesHandles(2), result.Time, ...
        getFieldOrEmpty(result.Battery, "Current"), "Battery current (A)");
    plotScalar(axesHandles(3), result.Time, ...
        getFieldOrEmpty(result.Battery, "SOC"), "SOC (1)");
    plotScalar(axesHandles(4), result.Time, ...
        getFieldOrEmpty(result.Battery, "DrivePowerClipped"), ...
        "Drive clipped power (W)");
end

if hasAny(result.Aero, ["ForceXBody", "ForceYBody", "DownforceFront", ...
        "DownforceRear", "DynamicPressure"])
    plotNames = ["Force X body", "Force Y body", "Front downforce", ...
        "Rear downforce", "Dynamic pressure"];
    [figures.Aero, axesHandles] = makeFigureSet("Aero", plotNames, options);
    plotScalar(axesHandles(1), result.Time, ...
        getFieldOrEmpty(result.Aero, "ForceXBody"), "Force X body (N)");
    plotScalar(axesHandles(2), result.Time, ...
        getFieldOrEmpty(result.Aero, "ForceYBody"), "Force Y body (N)");
    plotScalar(axesHandles(3), result.Time, ...
        getFieldOrEmpty(result.Aero, "DownforceFront"), "Front downforce (N)");
    plotScalar(axesHandles(4), result.Time, ...
        getFieldOrEmpty(result.Aero, "DownforceRear"), "Rear downforce (N)");
    plotScalar(axesHandles(5), result.Time, ...
        getFieldOrEmpty(result.Aero, "DynamicPressure"), "Dynamic pressure (Pa)");
end

figures.ExportedFiles = [figures.ExportedFiles; ...
    exportFigure(figures.Main, "summary", options)];

if options.SaveSummary
    if ~isfolder(options.OutputDirectory)
        mkdir(options.OutputDirectory);
    end
    summaryPath = fullfile(options.OutputDirectory, options.FilePrefix + "summary.json");
    fid = fopen(summaryPath, "w");
    assert(fid >= 0, "FSAE:ResultSummaryWrite", "Cannot write '%s'.", summaryPath);
    cleanup = onCleanup(@() fclose(fid));
    fwrite(fid, jsonencode(result.Meta), "char");
    figures.ExportedFiles(end + 1, 1) = string(summaryPath);
end
end

function options = applyDefaults(options, result)
if ~isfield(options, "Visible") || isempty(options.Visible), options.Visible = "off"; end
if ~isfield(options, "SaveFigures") || isempty(options.SaveFigures), options.SaveFigures = true; end
if ~isfield(options, "SaveSummary") || isempty(options.SaveSummary), options.SaveSummary = true; end
if ~isfield(options, "Formats") || isempty(options.Formats), options.Formats = "fig"; end
if ~isfield(options, "FilePrefix") || isempty(options.FilePrefix), options.FilePrefix = "open_loop_"; end
if ~isfield(options, "OutputDirectory") || isempty(options.OutputDirectory)
    scenario = "Unspecified";
    if isfield(result, "Meta") && isfield(result.Meta, "ScenarioName")
        scenario = string(result.Meta.ScenarioName);
    end
    projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
    trackName = inferResultTrack(result, scenario);
    options.OutputDirectory = fullfile(projectRoot, "results", ...
        "time_domain_closed_loop", trackName, "plots", "open_loop", ...
        sanitizeName(scenario));
end
options.OutputDirectory = string(options.OutputDirectory);
options.FilePrefix = string(options.FilePrefix);
options.Formats = string(options.Formats);
end

function name = sanitizeName(name)
name = regexprep(string(name), "[^A-Za-z0-9_-]", "_");
end

function trackName = inferResultTrack(result, scenarioName)
validTracks = ["acceleration", "skidpad", "autocross", "endurance"];
trackName = "autocross";
if isfield(result, "Meta")
    for fieldName = ["Event", "Track"]
        if isfield(result.Meta, char(fieldName))
            candidate = lower(string(result.Meta.(char(fieldName))));
            if any(candidate == validTracks)
                trackName = candidate;
                return
            end
        end
    end
end
scenarioKey = lower(string(scenarioName));
for candidate = validTracks
    if contains(scenarioKey, candidate)
        trackName = candidate;
        return
    end
end
end

function [handles, axesHandles] = makeFigureSet(groupName, plotNames, options)
names = groupName + " - " + string(plotNames(:));
handles = options.FigureHandle;
axesHandles = gobjects(numel(names), 1);
for index = 1:numel(names)
    tabHandle = uitab(options.TabGroup, "Title", names(index));
    axesHandles(index) = axes("Parent", tabHandle);
end
end

function tf = hasAny(group, fields)
tf = false;
for field = string(fields)
    if isfield(group, char(field)) && ~isempty(group.(char(field)))
        tf = true;
        return
    end
end
end

function value = getFieldOrEmpty(group, fieldName)
if isfield(group, fieldName)
    value = group.(fieldName);
else
    value = [];
end
end

function plotScalar(ax, time, data, label)
if ~isempty(data), plot(ax, time, data, "LineWidth", 1.2); end
xlabel(ax, "Time (s)"); ylabel(ax, label); title(ax, label); grid(ax, "on");
end

function plotWheel(ax, time, data, label, wheelNames, colors)
if ~isempty(data)
    data = normalizeWheelData(data, numel(time));
    hold(ax, "on");
    for wheel = 1:min(4, size(data, 2))
        plot(ax, time, data(:, wheel), ...
            "Color", colors(wheel, :), "LineWidth", 1.1);
    end
    hold(ax, "off");
    legend(ax, wheelNames(1:min(4, size(data, 2))), "Location", "best");
end
xlabel(ax, "Time (s)"); ylabel(ax, label); title(ax, label); grid(ax, "on");
end

function data = normalizeWheelData(data, sampleCount)
if isvector(data) && numel(data) == sampleCount
    data = data(:);
elseif size(data, 1) ~= sampleCount && size(data, 2) == sampleCount
    data = data.';
end
end

function files = exportFigure(handle, name, options)
files = strings(0, 1);
applyWhiteTheme(handle);
if ~options.SaveFigures, return; end
if ~isfolder(options.OutputDirectory), mkdir(options.OutputDirectory); end
for figureIndex = 1:numel(handle)
    figureStem = string(name);
    if numel(handle) > 1
        figureStem = figureStem + "_" + compose("%02d", figureIndex);
    end
    for format = options.Formats
        if format ~= "fig"
            warning("FSAE:UnknownPlotFormat", ...
                "仅保存 MATLAB FIG 文件；忽略不支持的格式 '%s'。", format);
            continue
        end
        path = fullfile(options.OutputDirectory, ...
            options.FilePrefix + figureStem + ".fig");
        switch format
            case "fig"
                savefig(handle(figureIndex), path);
        end
        files(end + 1, 1) = string(path); %#ok<AGROW>
    end
end
end

function applyWhiteTheme(handle)
axesHandles = findall(handle, "Type", "axes");
set(axesHandles, "Color", "white", "XColor", "black", ...
    "YColor", "black", "GridColor", [0.75, 0.75, 0.75]);
textHandles = findall(handle, "Type", "text");
set(textHandles, "Color", "black");
legendHandles = findall(handle, "Type", "legend");
set(legendHandles, "Color", "white", "TextColor", "black");
colorBars = findall(handle, "Type", "ColorBar");
set(colorBars, "Color", "black");
end
