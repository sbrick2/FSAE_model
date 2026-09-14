function scenario = createPathTrackingEnduranceScenario(options)
%CREATEPathTrackingENDURANCESCENARIO Build the image-derived FSEC endurance scenario.
%   The one-lap geometry comes from the 2024 FSEC handbook schematic and is
%   normalized to 1 km. NumberOfLaps remains an explicit simulation input.

arguments
    options.NumberOfLaps (1, 1) double {mustBeInteger, mustBePositive} = 1
    options.SampleDistance (1, 1) double {mustBePositive} = 0.5
    options.TargetSpeed (1, 1) double {mustBeNonnegative} = 12.0
    options.LateralAccelerationLimit (1, 1) double {mustBePositive} = 4.0
    options.ReferenceAccelerationLimit (1, 1) double {mustBePositive} = 3.0
    options.ReferenceDecelerationLimit (1, 1) double {mustBePositive} = 4.0
    options.ImagePath (1, 1) string = ""
    options.UseSavedData (1, 1) logical = true
    options.DataFolder (1, 1) string = ""
end

projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
addpath(fullfile(projectRoot, "scenarios", "Autocross"));
autocross = createPathTrackingAutocrossScenario( ...
    SampleDistance = options.SampleDistance, ...
    TargetSpeed = options.TargetSpeed, ...
    LateralAccelerationLimit = options.LateralAccelerationLimit, ...
    ReferenceAccelerationLimit = options.ReferenceAccelerationLimit, ...
    ReferenceDecelerationLimit = options.ReferenceDecelerationLimit, ...
    ImagePath = options.ImagePath, UseSavedData = options.UseSavedData, ...
    DataFolder = options.DataFolder, TrackDataName = "endurance");

scenario = autocross;
scenario.ID = "PathTracking-ENDURANCE-FSEC2024-1KM-" + ...
    string(options.NumberOfLaps) + "LAPS";
scenario.Event = "Endurance";
scenario.NumberOfLaps = options.NumberOfLaps;
scenario.StopTime = autocross.StopTime * options.NumberOfLaps;
scenario.Track.EventUsage = ...
    "Endurance geometry extracted from the 2024 FSEC handbook schematic";
scenario.EventDescription = ...
    "Counterclockwise endurance centerline extracted from the 2024 FSEC " + ...
    "handbook schematic and normalized to 1,000 m; lap count is a " + ...
    "simulation input.";
end
