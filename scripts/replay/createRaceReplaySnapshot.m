function snapshot = createRaceReplaySnapshot(scenario, parameters, cfg, options)
%CREATERACEREPLAYSNAPSHOT Capture effective geometry and parameters before a run.
%   Save this struct in result.Meta.Replay. TireMetadata can be supplied to
%   avoid any dictionary dependency (for imported or externally run results).
arguments
    scenario (1, 1) struct
    parameters (1, 1) struct
    cfg (1, 1) struct
    options.TireMetadata (1, 1) struct = struct
    options.DictionaryPath (1, 1) string = ""
end
root = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
dictionaryPath = options.DictionaryPath;
if dictionaryPath == ""
    dictionaryPath = fullfile(root, "data", "VehicleData.sldd");
end
geometry = struct;
for field = ["Wheelbase", "CGToFrontAxle", "TrackFront", "TrackRear"]
    geometry.(field) = raceReplayValue(parameters, "Vehicle." + field, NaN);
end
geometry.TireWidth = raceReplayValue(cfg, "Driver.TireSectionWidth", 0.2);
geometry.WheelLength = 2 * raceReplayValue(parameters, "Tire.UnloadedRadius", ...
    raceReplayValue(parameters, "Tire.EffectiveRadius", 0.2));
geometry.Approximate = false;
geometry.Source = "当次运行参数；车身外形为示意";
driver = raceReplayValue(cfg, "Driver", struct);
driver.SteeringWheelRatio = NaN;
if lower(string(raceReplayValue(driver, "Model", ""))) == "adaptive_autocross"
    driver.SteeringWheelRatio = 1;
end
tire = options.TireMetadata;
if isempty(fieldnames(tire))
    tire.Model = lower(string(raceReplayValue(cfg, "Vehicle.TireModel", "")));
    tire.RoadGripScale = raceReplayValue(scenario, "Environment.RoadGripScale", NaN);
    tire.RoadMuLimit = raceReplayValue(scenario, "Environment.RoadMuLimit", NaN);
    tire.ForceEpsilon = raceReplayValue(parameters, "Tire.ForceEpsilon", NaN);
    tire.ParameterSource = dictionaryPath;
    if tire.Model ~= "simple" && isfile(dictionaryPath)
        dictionary = Simulink.data.dictionary.open(dictionaryPath);
        cleanup = onCleanup(@() close(dictionary));
        section = getSection(dictionary, "Design Data");
        if tire.Model == "mf62"
            tire.Parameters = getValue(getEntry(section, "TireMFParameters"));
        elseif tire.Model == "ttc_map"
            tire.MapPoints = getValue(getEntry(section, "TireMapPoints"));
            tire.MapForces = getValue(getEntry(section, "TireMapForces"));
        end
        clear cleanup
    end
end
snapshot = struct("Version", "1.0", "CapturedAt", ...
    string(datetime("now", "Format", "yyyy-MM-dd'T'HH:mm:ss")), ...
    "Source", "运行开始前的有效参数", "WheelOrder", ["FL", "FR", "RL", "RR"], ...
    "Track", raceReplayValue(scenario, "Track", struct), ...
    "Geometry", geometry, "Driver", driver, "Tire", tire);
snapshot.Units = struct("Position", "m", "Angle", "rad", ...
    "Force", "N", "Torque", "N*m", "AngularSpeed", "rad/s", ...
    "Power", "W", "PedalRequest", "0..1");
end
