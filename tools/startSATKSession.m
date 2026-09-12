function startSATKSession()
%STARTSATKSESSION Start a persistent MATLAB session for agent model tools.

projectRoot = fileparts(fileparts(mfilename("fullpath")));
toolkitRoot = fullfile(getenv("USERPROFILE"), ".matlab", ...
    "agentic-toolkits", "simulink");

addpath(toolkitRoot);
satk_initialize;
cd(projectRoot);
fprintf("SATK session ready for project: %s\n", projectRoot);
end
