function project = initProject()
%INITPROJECT Open the FSAE project and report ProjectFoundation readiness.

projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
try
    project = currentProject;
    if ~strcmpi(string(project.RootFolder), projectRoot)
        close(project);
        project = openProject(projectRoot);
    end
catch
    project = openProject(projectRoot);
end

% Keep all Simulink cache and generated-code output under the project cache
% directory for this MATLAB session as well as in the project metadata.
generatedFolder = fullfile(project.RootFolder, "cache");
if ~isfolder(generatedFolder)
    mkdir(generatedFolder);
end
project.SimulinkCacheFolder = generatedFolder;
project.SimulinkCodeGenFolder = generatedFolder;
project.DependencyCacheFile = fullfile(generatedFolder, "dep_cache.graphml");
Simulink.fileGenControl( ...
    "set", ...
    "CacheFolder", char(generatedFolder), ...
    "CodeGenFolder", char(generatedFolder), ...
    "createDir", true);

dictionaryPath = fullfile(project.RootFolder, "data", "VehicleData.sldd");
assert(isfile(dictionaryPath), "FSAE:MissingDictionary", ...
    "VehicleData.sldd is missing. Run setupProject first.");

dictionary = Simulink.data.dictionary.open(dictionaryPath);
cleanup = onCleanup(@() close(dictionary));
designData = getSection(dictionary, "Design Data");
requiredBuses = ["TrackReferenceBus", "DriverCommandBus", ...
    "ActuatorCommandBus", "VehicleStateBus", "WheelStateBus", ...
    "PowertrainStateBus", "EnvironmentBus", "SensorBus", ...
    "ControllerDebugBus", "ScoringBus"];
for busName = requiredBuses
    getEntry(designData, busName);
end

fprintf("FSAE project initialized: %s\n", project.RootFolder);
fprintf("Resolved %d Bus definitions from VehicleData.sldd.\n", numel(requiredBuses));
fprintf("Unverified physical parameters remain placeholders until supporting data is imported.\n");
end
