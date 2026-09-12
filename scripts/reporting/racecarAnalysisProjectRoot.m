function projectRoot = racecarAnalysisProjectRoot()
%RACECARANALYSISPROJECTROOT 返回 FSAE 工程根目录。

projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
end
