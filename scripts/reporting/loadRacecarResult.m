function [result, filePath] = loadRacecarResult(source, trackName)
%LOADRACECARRESULT 加载标准化圈速结果，或接收内存中的结果结构体。

arguments
    source = ""
    trackName (1, 1) string = "autocross"
end

if isstruct(source)
    result = source;
    filePath = "";
    validateInMemoryResult(result);
    return
end

projectRoot = racecarAnalysisProjectRoot();
addpath(fullfile(projectRoot, "scripts", "reporting"), "-begin");
filePath = string(source);
if strlength(filePath) == 0
    filePath = findLatestResult(projectRoot, trackName);
end
[result, filePath] = loadLapSimulationResult(filePath);
end

function filePath = findLatestResult(projectRoot, trackName)
validTracks = ["acceleration", "skidpad", "autocross", "endurance"];
trackName = lower(trackName);
assert(any(trackName == validTracks), "FSAE:Analysis:InvalidTrack", ...
    "Track must be acceleration, skidpad, autocross or endurance.");
resultRoot = fullfile(projectRoot, "results", "time_domain_closed_loop", ...
    trackName);
candidates = dir(fullfile(resultRoot, "**", "lap_simulation_result.mat"));
assert(~isempty(candidates), "FSAE:Analysis:NoResult", ...
    "No lap_simulation_result.mat was found under %s.", resultRoot);
[~, newest] = max([candidates.datenum]);
filePath = string(fullfile(candidates(newest).folder, candidates(newest).name));
end

function validateInMemoryResult(result)
assert(isfield(result, "Time") && ~isempty(result.Time), ...
    "FSAE:Analysis:MissingTime", "The result structure must contain Time.");
assert(isfield(result, "Vehicle") && isfield(result, "Track"), ...
    "FSAE:Analysis:MissingGroups", ...
    "The result structure must contain Vehicle and Track groups.");
end
