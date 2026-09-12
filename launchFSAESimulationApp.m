function app = launchFSAESimulationApp()
%LAUNCHFSAESIMULATIONAPP 启动 FSAE 整车仿真图形界面。

projectRoot = string(fileparts(mfilename("fullpath")));
addpath(fullfile(projectRoot, "apps"), "-begin");
app = FSAESimulationApp(ProjectRoot = projectRoot);
setappdata(app.UIFigure, "FSAESimulationApp", app);
end