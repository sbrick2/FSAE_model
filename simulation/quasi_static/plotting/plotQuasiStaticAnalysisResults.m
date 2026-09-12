function figures = plotQuasiStaticAnalysisResults(source, options)
%PLOTQuasiStaticANALYSISRESULTS 绘制已有准静态参数扫描（QuasiStatic）结果。
%   FIGURES = PLOTQuasiStaticANALYSISRESULTS() 自动读取最新参数扫描 MAT 文件；
%   SOURCE 也可直接传入 QuasiStatic 结果结构体数组，或传入包含 results 变量的 MAT 路径。
%   本函数只做结果分析，不重新计算 GGV 或圈速。
%
%   可选名称-值参数：
%     Visible      - 图窗可见性，"on"（默认）或 "off"
%     SaveFigures  - 是否保存 FIG，默认 false
%     OutputFolder - 保存目录；为空时使用
%                    results/quasi_static/parameter_sweep/plots
%
%   返回值 FIGURES 为绘图句柄结构体，并在 FIGURES.SourceFile 中记录来源文件。
%   示例：plotQuasiStaticAnalysisResults("scan.mat", SaveFigures=true);

arguments
    source = ""
    options.Visible (1, 1) string = "on"
    options.SaveFigures (1, 1) logical = false
    options.OutputFolder (1, 1) string = ""
end

[results, filePath] = loadQuasiStaticResults(source);
projectRoot = quasiStaticProjectRoot();
addpath(fullfile(projectRoot, "scripts", "reporting"), "-begin");
outputFolder = "";
if options.SaveFigures
    outputFolder = options.OutputFolder;
    if strlength(outputFolder) == 0
        outputFolder = fullfile(projectRoot, "results", "quasi_static", ...
            "parameter_sweep", "plots");
    end
end
figures = plotQuasiStaticResults(results, Visible = options.Visible, ...
    OutputFolder = outputFolder);
figures.SourceFile = filePath;
end

function [results, filePath] = loadQuasiStaticResults(source)
if isstruct(source)
    results = source; filePath = ""; return
end
filePath = string(source);
if strlength(filePath) == 0
    root = fullfile(quasiStaticProjectRoot(), "results", "quasi_static", ...
        "parameter_sweep");
    candidates = dir(fullfile(root, "**", "*.mat"));
    assert(~isempty(candidates), "FSAE:Analysis:NoQuasiStaticResult", ...
        "Provide an QuasiStatic result/MAT file; no MAT file was found under %s.", root);
    [~, order] = sort([candidates.datenum], "descend");
    filePath = firstCompatibleQuasiStaticFile(candidates(order));
end
loaded = load(filePath, "results");
assert(isfield(loaded, "results") && isstruct(loaded.results), ...
    "FSAE:Analysis:QuasiStaticVariable", ...
    "QuasiStatic MAT file must contain a results struct array.");
results = loaded.results;
hasCapabilitySweepFields = isfield(results, "PeakAx") && ...
    isfield(results, "PeakAy");
hasLapTimeSweepFields = isfield(results, "Parameter1Value") && ...
    isfield(results, "Parameter2Value") && isfield(results, "LapTime");
assert(hasCapabilitySweepFields || hasLapTimeSweepFields, ...
    "FSAE:Analysis:QuasiStaticContract", ...
    "The results variable is not a supported QuasiStatic sweep result.");
end

function filePath = firstCompatibleQuasiStaticFile(candidates)
filePath = "";
for item = reshape(candidates, 1, [])
    candidate = string(fullfile(item.folder, item.name));
    variables = whos("-file", candidate);
    if any(string({variables.name}) == "results")
        loaded = load(candidate, "results");
        if isstruct(loaded.results) && ...
                ((isfield(loaded.results, "PeakAx") && ...
                isfield(loaded.results, "PeakAy")) || ...
                (isfield(loaded.results, "Parameter1Value") && ...
                isfield(loaded.results, "Parameter2Value") && ...
                isfield(loaded.results, "LapTime")))
            filePath = candidate;
            return
        end
    end
end
assert(strlength(filePath) > 0, "FSAE:Analysis:QuasiStaticVariable", ...
    "No saved QuasiStatic MAT file contains a compatible capability-sweep result.");
end

function projectRoot = quasiStaticProjectRoot()
projectRoot = string(fileparts(fileparts(fileparts(fileparts( ...
    mfilename("fullpath"))))));
end
