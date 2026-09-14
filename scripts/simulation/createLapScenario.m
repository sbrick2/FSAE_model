function scenario = createLapScenario(cfg)
%CREATELAPSCENARIO 统一选择并生成圈速仿真的赛道场景。
%   SCENARIO = CREATELAPSCENARIO(CFG) 根据 CFG.Track.Name 调用项目中已有的
%   PathTracking 场景生成器。支持 acceleration、skidpad、autocross 和 endurance。
%   场景中的位置单位为 m，速度单位为 m/s，航向和曲率相关角度分别为
%   rad 和 1/m。场景生成阶段只建立几何；ReferenceSpeed 随后由 GGV
%   速度规划器覆盖，因此这里不接受手动目标速度。
%
%   示例：
%       scenario = createLapScenario(cfg);
%
%   四轮顺序（涉及路面宽度或环境向量时）固定为 [FL, FR, RL, RR]。

arguments
    cfg (1, 1) struct
end

projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
addpath(fullfile(projectRoot, "scripts", "track"), "-begin");
addpath(fullfile(projectRoot, "scenarios", "Acceleration"), "-begin");
addpath(fullfile(projectRoot, "scenarios", "Skidpad"), "-begin");
addpath(fullfile(projectRoot, "scenarios", "Autocross"), "-begin");
addpath(fullfile(projectRoot, "scenarios", "Endurance"), "-begin");

validateLapTrackConfig(cfg, projectRoot);
trackName = lower(string(cfg.Track.Name));
useSavedData = true;
if isfield(cfg.Track, "UseSavedData")
    useSavedData = cfg.Track.UseSavedData;
end
dataFolder = "";
if isfield(cfg.Track, "DataFolder")
    dataFolder = string(cfg.Track.DataFolder);
end

switch trackName
    case "acceleration"
        scenario = createPathTrackingAccelerationScenario( ...
            TargetSpeed = Inf, ...
            SampleDistance = cfg.Track.SampleDistance, ...
            UseSavedData = useSavedData, DataFolder = dataFolder);
    case "skidpad"
        scenario = createPathTrackingSkidpadScenario( ...
            TargetSpeed = Inf, ...
            SampleDistance = cfg.Track.SampleDistance, ...
            UseSavedData = useSavedData, DataFolder = dataFolder);
    case "autocross"
        scenario = createPathTrackingAutocrossScenario( ...
            TargetSpeed = Inf, ...
            SampleDistance = cfg.Track.SampleDistance, ...
            ImagePath = cfg.Track.ImagePath, UseSavedData = useSavedData, ...
            DataFolder = dataFolder);
    case "endurance"
        scenario = createPathTrackingEnduranceScenario( ...
            NumberOfLaps = cfg.Track.NumberOfLaps, ...
            TargetSpeed = Inf, ...
            SampleDistance = cfg.Track.SampleDistance, ...
            ImagePath = cfg.Track.ImagePath, UseSavedData = useSavedData, ...
            DataFolder = dataFolder);
    otherwise
        error("FSAE:Lap:InvalidTrack", ...
            "无效赛道名称 '%s'。允许值为：acceleration、skidpad、autocross、endurance。", ...
            cfg.Track.Name);
end

% 场景函数返回的停止时间是默认值；用户给出的正数由主程序覆盖。
scenario.RequestedTrackName = trackName;
scenario.RequestedNumberOfLaps = cfg.Track.NumberOfLaps;
scenario.TireModel = lower(string(cfg.Vehicle.TireModel));
scenario.Track.SampleDistance = double(cfg.Track.SampleDistance);
scenario.Track.ReferenceSpeed(:) = 0.0;
scenario.ReferenceSpeedSource = ...
    "unplanned track geometry; applyGGVReferenceSpeed is required";
if trackName == "endurance"
    scenario.NumberOfLaps = cfg.Track.NumberOfLaps;
end
end

function validateLapTrackConfig(cfg, projectRoot)
required = ["Name", "SampleDistance", "NumberOfLaps", "ImagePath"];
missing = required(~isfield(cfg.Track, cellstr(required)));
if ~isempty(missing)
    error("FSAE:Lap:MissingTrackConfig", ...
        "cfg.Track 缺少字段：%s。", strjoin(missing, ", "));
end

validNames = ["acceleration", "skidpad", "autocross", "endurance"];
trackName = lower(string(cfg.Track.Name));
if ~any(trackName == validNames)
    error("FSAE:Lap:InvalidTrack", ...
        "无效赛道名称 '%s'。允许值为：%s。", ...
        cfg.Track.Name, strjoin(validNames, ", "));
end
if cfg.Track.SampleDistance <= 0 || ~isscalar(cfg.Track.SampleDistance) || ...
        ~isfinite(cfg.Track.SampleDistance)
    error("FSAE:Lap:InvalidSampleDistance", ...
        "Track.SampleDistance 收到 %g，必须为正有限数（单位：m）。", ...
        cfg.Track.SampleDistance);
end
if cfg.Track.NumberOfLaps <= 0 || ~isscalar(cfg.Track.NumberOfLaps) || ...
        ~isfinite(cfg.Track.NumberOfLaps) || ...
        cfg.Track.NumberOfLaps ~= round(cfg.Track.NumberOfLaps)
    error("FSAE:Lap:InvalidLapCount", ...
        "Track.NumberOfLaps 收到 %g，必须为正整数。", cfg.Track.NumberOfLaps);
end
imagePath = string(cfg.Track.ImagePath);
if isfield(cfg.Track, "UseSavedData") && ...
        (~islogical(cfg.Track.UseSavedData) || ...
        ~isscalar(cfg.Track.UseSavedData))
    error("FSAE:Lap:InvalidTrackDataOption", ...
        "Track.UseSavedData 必须为 logical 标量。");
end
if isfield(cfg.Track, "DataFolder") && ...
        ~isscalar(string(cfg.Track.DataFolder))
    error("FSAE:Lap:InvalidTrackDataFolder", ...
        "Track.DataFolder 必须为字符串标量。");
end
if strlength(imagePath) > 0 && ~isfile(imagePath)
    error("FSAE:Lap:MissingTrackImage", ...
        "Track.ImagePath 收到 '%s'，但文件不存在。", imagePath);
end
if any(trackName == ["autocross", "endurance"]) && ...
        strlength(imagePath) == 0
    defaultImagePath = fullfile(projectRoot, "scenarios", "Endurance", ...
        "assets", "2024_fsec_endurance_track.png");
    if ~isfile(defaultImagePath)
        error("FSAE:Lap:TrackImageRequired", ...
            "默认耐久赛道原图不存在：%s。", defaultImagePath);
    end
end
end
