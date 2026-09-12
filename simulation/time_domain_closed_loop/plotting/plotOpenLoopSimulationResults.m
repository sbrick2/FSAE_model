function figures = plotOpenLoopSimulationResults(source, options)
%PLOTOPENLOOPSIMULATIONRESULTS 绘制已有开环时域仿真结果。
%   FIGURES = PLOTOPENLOOPSIMULATIONRESULTS(SOURCE) 在一个图窗的多个标签页中
%   绘制项目现有开环结果；
%   SOURCE 可为结果结构体、MAT 文件路径或空字符串。为空时自动读取
%   results/time_domain_closed_loop/<track>/open_loop 下最新 MAT 文件。
%
%   名称-值参数：Track 选择赛事目录；Visible 可选 on/off；SaveFigures 控制
%   FIG 保存；SaveSummary 控制摘要保存；OutputFolder 为空时使用
%   results/time_domain_closed_loop/<track>/plots/open_loop。
%   返回 FIGURES 主图窗句柄及兼容字段，并记录 SourceFile。该函数只读取结果，
%   不运行模型。

arguments
    source = ""
    options.Track (1, 1) string = "autocross"
    options.Visible (1, 1) string = "on"
    options.SaveFigures (1, 1) logical = false
    options.SaveSummary (1, 1) logical = false
    options.OutputFolder (1, 1) string = ""
end

[result, filePath] = loadOpenLoopResult(source, options.Track);
projectRoot = racecarAnalysisProjectRoot();
addpath(fullfile(projectRoot, "scripts", "simulation"), "-begin");
addpath(fullfile(projectRoot, "scripts", "reporting"), "-begin");
plotOptions = struct( ...
    "Visible", options.Visible, ...
    "SaveFigures", options.SaveFigures, ...
    "SaveSummary", options.SaveSummary, ...
    "Formats", "fig", ...
    "FilePrefix", "open_loop_analysis_");
if strlength(options.OutputFolder) > 0
    plotOptions.OutputDirectory = options.OutputFolder;
elseif options.SaveFigures || options.SaveSummary
    plotOptions.OutputDirectory = fullfile(projectRoot, ...
        "results", "time_domain_closed_loop", lower(options.Track), ...
        "plots", "open_loop");
end
figures = plotOpenLoopResults(result, plotOptions);
figures.SourceFile = filePath;
end

function [result, filePath] = loadOpenLoopResult(source, trackName)
if isstruct(source)
    result = source; filePath = ""; return
end
filePath = string(source);
if strlength(filePath) == 0
    trackName = lower(trackName);
    assert(any(trackName == ["acceleration", "skidpad", "autocross", ...
        "endurance"]), "FSAE:Analysis:InvalidTrack", ...
        "Track must be acceleration, skidpad, autocross or endurance.");
    root = fullfile(racecarAnalysisProjectRoot(), "results", ...
        "time_domain_closed_loop", trackName, "open_loop");
    candidates = dir(fullfile(root, "**", "*.mat"));
    assert(~isempty(candidates), "FSAE:Analysis:NoOpenLoopResult", ...
        "Provide a result structure/MAT file; no MAT file was found under %s.", root);
    [~, order] = sort([candidates.datenum], "descend");
    filePath = firstMatWithVariable(candidates(order), "result");
end
loaded = load(filePath, "result");
assert(isfield(loaded, "result") && isstruct(loaded.result), ...
    "FSAE:Analysis:OpenLoopVariable", ...
    "Open-loop MAT file must contain a result structure.");
result = loaded.result;
end

function filePath = firstMatWithVariable(candidates, variableName)
filePath = "";
for item = reshape(candidates, 1, [])
    candidate = string(fullfile(item.folder, item.name));
    variables = whos("-file", candidate);
    if any(string({variables.name}) == variableName)
        filePath = candidate;
        return
    end
end
assert(strlength(filePath) > 0, "FSAE:Analysis:OpenLoopVariable", ...
    "No saved open-loop MAT file contains variable '%s'.", variableName);
end
