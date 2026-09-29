function data = prepareRaceReplayData(source, options)
%PREPARERACEREPLAYDATA Validate and prepare a saved lap for synchronized replay.
%   Track geometry is read from the run snapshot or matching recorded track
%   file. TrackFile and Snapshot may override it explicitly. The current
%   vehicle dictionary is never used implicitly.
arguments
    source
    options.TrackFile (1, 1) string = ""
    options.Track (1, 1) struct = struct
    options.Snapshot (1, 1) struct = struct
    options.ForceScale (1, 1) double = NaN
    options.ContactThreshold (1, 1) double {mustBeNonnegative} = 1
    options.ProjectRoot (1, 1) string = ""
end
if ischar(source) || (isstring(source) && isscalar(source))
    [result, sourceFile] = loadLapSimulationResult(string(source));
else
    assert(isstruct(source) && isscalar(source), ...
        "FSAE:Replay:Source", "Provide a result struct or MAT result path.");
    result = source;
    sourceFile = string(raceReplayValue(result, "Meta.LoadedFile", ...
        raceReplayValue(result, "Meta.ResultFile", "")));
end
time = raceReplayValue(result, "Time", []);
assert(isnumeric(time) && isvector(time) && ~isempty(time) && ...
    all(isfinite(time)) && all(diff(double(time(:))) > 0), ...
    "FSAE:Replay:Time", "Time must be a finite strictly increasing vector.");
time = double(time(:));
snapshot = raceReplayValue(result, "Meta.Replay", struct);
names = fieldnames(options.Snapshot);
for index = 1:numel(names)
    snapshot.(names{index}) = options.Snapshot.(names{index});
end
version = string(raceReplayValue(snapshot, "Version", "1.0"));
assert(isscalar(version) && version == "1.0", ...
    "FSAE:Replay:SnapshotVersion", "Unsupported replay snapshot version.");
order = string(raceReplayValue(snapshot, "WheelOrder", ["FL", "FR", "RL", "RR"]));
assert(isequal(reshape(order, 1, []), ["FL", "FR", "RL", "RR"]), ...
    "FSAE:Replay:WheelOrder", "Replay wheel order must be FL, FR, RL, RR.");
driver = raceReplayValue(snapshot, "Driver", ...
    raceReplayValue(result, "Config.Driver", struct));
mapping = {
    'X', 'Vehicle.X', 1; 'Y', 'Vehicle.Y', 1; 'Psi', 'Vehicle.Psi', 1;
    'Speed', 'Vehicle.Speed', 1; 'Ux', 'Vehicle.Ux', 1; 'Uy', 'Vehicle.Uy', 1;
    'Ax', 'Vehicle.Ax', 1; 'Ay', 'Vehicle.Ay', 1;
    'SteerAngle', 'Wheel.SteerAngle', 4;
    'NormalLoad', 'Wheel.NormalLoad', 4; 'CamberAngle', 'Wheel.CamberAngle', 4;
    'SlipRatio', 'Wheel.SlipRatio', 4; 'SlipAngle', 'Wheel.SlipAngle', 4;
    'Fx', 'Tire.FxWheel', 4; 'Fy', 'Tire.FyWheel', 4;
    'RecordedUtilization', 'Tire.MuUtilization', 4;
    'MotorTorque', 'Powertrain.MotorTorqueActual', 4;
    'MotorOmega', 'Powertrain.MotorSpeed', 4;
    'MotorRPM', 'Powertrain.MotorRPM', 4;
    'RecordedMotorPower', 'Powertrain.MotorMechanicalPower', 4;
    'BatteryPower', 'Battery.Power', 1;
    'SteeringRequest', 'Driver.SteeringWheelAngleRequest', 1;
    'AccelerationRequest', 'Driver.LongitudinalAccelerationRequest', 1;
    'AcceleratorPedalRequest', 'Driver.AcceleratorPedalRequest', 1;
    'BrakePedalRequest', 'Driver.BrakePedalRequest', 1;
    'DriveTorqueRequest', 'Driver.DriveTorqueRequest', 1;
    'BrakePressureRequest', 'Driver.BrakePressureRequest', 1;
    'LapIndex', 'Track.LapIndex', 1; 'PathS', 'Track.PathS', 1;
    'Progress', 'Track.Progress', 1;
    };
signals = struct;
missing = strings(0, 1);
for index = 1:size(mapping, 1)
    path = string(mapping{index, 2});
    value = raceReplayValue(result, path, []);
    if isempty(value)
        missing(end + 1, 1) = path; %#ok<AGROW>
        value = NaN(numel(time), mapping{index, 3});
    else
        assert((isnumeric(value) || islogical(value)) && ...
            isequal(size(value), [numel(time), mapping{index, 3}]), ...
            "FSAE:Replay:Dimension", "%s must be %d-by-%d.", ...
            path, numel(time), mapping{index, 3});
        value = double(value);
        value(~isfinite(value)) = NaN;
    end
    signals.(mapping{index, 1}) = value;
end
assert(any(all(isfinite([signals.X, signals.Y, signals.Psi]), 2)), ...
    "FSAE:Replay:Pose", "Replay requires valid Vehicle.X, Y and Psi.");
signals.Psi = unwrapFiniteSegments(signals.Psi);
speedDerived = hypot(signals.Ux, signals.Uy);
mask = ~isfinite(signals.Speed) & isfinite(speedDerived);
signals.Speed(mask) = speedDerived(mask);
mask = ~isfinite(signals.MotorOmega) & isfinite(signals.MotorRPM);
signals.MotorOmega(mask) = signals.MotorRPM(mask) * 2 * pi / 60;
validPair = isfinite(signals.MotorRPM) & isfinite(signals.MotorOmega);
rpmDifference = abs(signals.MotorRPM - signals.MotorOmega * 60 / (2 * pi));
assert(~any(rpmDifference(validPair) > ...
    1e-6 .* max(abs(signals.MotorRPM(validPair)), 1)), ...
    "FSAE:Replay:MotorSpeedMismatch", "MotorRPM and MotorSpeed disagree.");
track = raceReplayValue(snapshot, "Track", struct);
trackSource = "运行快照";
if ~isempty(fieldnames(options.Track))
    track = options.Track;
    trackSource = "指定赛道";
elseif strlength(options.TrackFile) > 0
    loaded = load(options.TrackFile);
    track = raceReplayValue(loaded, "track", ...
        raceReplayValue(loaded, "Track", struct));
    assert(~isempty(fieldnames(track)), "FSAE:Replay:TrackFile", ...
        "TrackFile must contain a track or Track struct.");
    trackSource = "指定赛道文件：" + options.TrackFile;
elseif isempty(fieldnames(track))
    [track, trackSource] = loadRaceReplayTrack(result, sourceFile, options.ProjectRoot);
end
track = prepareTrack(track);
if isempty(fieldnames(track))
    trackSource = "赛道几何未记录；仅显示车辆轨迹";
end
geometry = prepareGeometry(raceReplayValue(snapshot, "Geometry", struct));
forceScale = options.ForceScale;
if isnan(forceScale)
    forces = abs([signals.Fx(:); signals.Fy(:)]);
    forces = sort(forces(isfinite(forces) & forces > 0));
    reference = 1000;
    if ~isempty(forces)
        reference = max(forces(max(1, ceil(0.99 * numel(forces)))), 100);
    end
    forceScale = 1.5 / reference;
end
assert(isfinite(forceScale) && forceScale > 0, ...
    "FSAE:Replay:ForceScale", "ForceScale must be positive (m/N).");
tire = raceReplayValue(snapshot, "Tire", struct);
% Road changes are sampled as held inputs on the original result clock.
% Accept saved Environment channels or an explicitly supplied Nx4 snapshot.
for field = ["RoadGripScale", "RoadMuLimit"]
    recorded = raceReplayValue(result, "Environment." + field, []);
    assert(isempty(recorded) || (isnumeric(recorded) && ...
        (isequal(size(recorded), [numel(time), 1]) || ...
        isequal(size(recorded), [numel(time), 4]))), ...
        "FSAE:Replay:RoadDimension", "Recorded %s must align with Time (Nx1 or Nx4).", field);
    value = recorded;
    if isempty(value), value = raceReplayValue(tire, field, NaN); end
    assert(isnumeric(value) && (isscalar(value) || ...
        isequal(size(value), [1, 4]) || isequal(size(value), [4, 1]) || ...
        isequal(size(value), [numel(time), 1]) || ...
        isequal(size(value), [numel(time), 4])), ...
        "FSAE:Replay:RoadDimension", "%s must be scalar, four constants, Nx1 or Nx4.", field);
    if isscalar(value)
        value = repmat(value, numel(time), 4);
    elseif isequal(size(value), [1, 4]) || ...
            (isequal(size(value), [4, 1]) && isempty(recorded))
        value = repmat(reshape(value, 1, 4), numel(time), 1);
    elseif size(value, 2) == 1
        value = repmat(value, 1, 4);
    end
    signals.(field) = double(value);
end
if lower(string(raceReplayValue(tire, "Model", ""))) == "ttc_map"
    profile = raceReplayValue(tire, "Profile", struct);
    points = raceReplayValue(tire, "MapPoints", ...
        raceReplayValue(profile, "Map.Points", []));
    forces = raceReplayValue(tire, "MapForces", ...
        raceReplayValue(profile, "Map.Forces", []));
    if ~isempty(points) && ~isempty(forces)
        valid = points(:, 4) > 1;
        if any(valid)
            tire.MapPeakMu = max(0.05, max(hypot( ...
                forces(valid, 1), forces(valid, 2)) ./ points(valid, 4)));
        end
    end
end
lapStart = NaN(numel(time), 1);
crossings = find(diff(signals.LapIndex) > 0) + 1;
for index = 1:numel(crossings)
    last = numel(time);
    if index < numel(crossings)
        last = crossings(index + 1) - 1;
    end
    lapStart(crossings(index):last) = time(crossings(index));
end
if isfinite(signals.PathS(1)) && abs(signals.PathS(1)) <= 0.5
    last = numel(time);
    if ~isempty(crossings)
        last = crossings(1) - 1;
    end
    lapStart(1:last) = time(1);
end
signals.LapStartTime = lapStart;
data = struct("Time", time, "Signals", signals, "Track", track, ...
    "Geometry", geometry, "Driver", driver, "TireMetadata", tire, ...
    "ForceScale", forceScale, "ContactThreshold", options.ContactThreshold, ...
    "SourceFile", string(sourceFile), "MissingSignals", missing, ...
    "TrackSource", trackSource, "WheelNames", ["FL", "FR", "RL", "RR"]);
end

function values = unwrapFiniteSegments(values)
valid = isfinite(values);
starts = find(diff([false; valid]) == 1);
stops = find(diff([valid; false]) == -1);
for index = 1:numel(starts)
    rows = starts(index):stops(index);
    values(rows) = unwrap(values(rows));
end
end

function track = prepareTrack(track)
if isempty(fieldnames(track))
    return
end
x = raceReplayValue(track, "X", []);
y = raceReplayValue(track, "Y", []);
assert(isnumeric(x) && isnumeric(y) && isvector(x) && ...
    numel(x) >= 2 && numel(y) == numel(x) && ...
    all(isfinite(x)) && all(isfinite(y)), ...
    "FSAE:Replay:Track", "Track requires matching finite X/Y vectors.");
track.X = double(x(:));
track.Y = double(y(:));
count = numel(x);
heading = raceReplayValue(track, "Heading", []);
if isempty(heading)
    heading = atan2(gradient(track.Y), gradient(track.X));
end
track.Heading = expandTrackField(heading, count, "Heading");
track.LeftHalfWidth = expandTrackField( ...
    raceReplayValue(track, "LeftHalfWidth", NaN), count, "LeftHalfWidth");
track.RightHalfWidth = expandTrackField( ...
    raceReplayValue(track, "RightHalfWidth", NaN), count, "RightHalfWidth");
assert(~any(track.LeftHalfWidth < 0 | track.RightHalfWidth < 0), ...
    "FSAE:Replay:TrackWidth", "Track half-widths cannot be negative.");
closed = raceReplayValue(track, "IsClosed", false);
lengthValue = raceReplayValue(track, "Length", NaN);
assert(isscalar(closed) && isscalar(lengthValue), ...
    "FSAE:Replay:TrackDimension", "Track.IsClosed and Length must be scalar.");
track.IsClosed = logical(closed);
track.Length = double(lengthValue);
track.LeftX = track.X - sin(track.Heading) .* track.LeftHalfWidth;
track.LeftY = track.Y + cos(track.Heading) .* track.LeftHalfWidth;
track.RightX = track.X + sin(track.Heading) .* track.RightHalfWidth;
track.RightY = track.Y - cos(track.Heading) .* track.RightHalfWidth;
if track.IsClosed
    fields = ["X", "Y", "Heading", "LeftX", "LeftY", "RightX", "RightY"];
    for field = fields
        track.(field)(end + 1, 1) = track.(field)(1);
    end
end
end

function value = expandTrackField(value, count, name)
assert(isnumeric(value) && (isscalar(value) || numel(value) == count), ...
    "FSAE:Replay:TrackDimension", "Track.%s has invalid dimensions.", name);
value = double(value(:));
if isscalar(value)
    value = repmat(value, count, 1);
end
end

function geometry = prepareGeometry(geometry)
if isempty(fieldnames(geometry))
    geometry = struct("Wheelbase", 1.6, "CGToFrontAxle", 0.8, ...
        "TrackFront", 1.2, "TrackRear", 1.2, "TireWidth", 0.2, ...
        "WheelLength", 0.4, "Approximate", true, ...
        "Source", "示意几何（未记录当次车辆尺寸）");
end
required = ["Wheelbase", "CGToFrontAxle", "TrackFront", "TrackRear"];
for field = required
    value = raceReplayValue(geometry, field, NaN);
    assert(isnumeric(value) && isscalar(value) && isfinite(value) && value > 0, ...
        "FSAE:Replay:Geometry", "Geometry.%s must be positive.", field);
    geometry.(field) = double(value);
end
assert(geometry.CGToFrontAxle < geometry.Wheelbase, ...
    "FSAE:Replay:Geometry", "CGToFrontAxle must be less than Wheelbase.");
geometry.TireWidth = raceReplayValue(geometry, "TireWidth", 0.2);
geometry.WheelLength = raceReplayValue(geometry, "WheelLength", 0.4);
assert(all(isfinite([geometry.TireWidth, geometry.WheelLength])) && ...
    all([geometry.TireWidth, geometry.WheelLength] > 0), ...
    "FSAE:Replay:Geometry", "Wheel dimensions must be positive.");
geometry.Approximate = logical(raceReplayValue(geometry, "Approximate", false));
geometry.Source = string(raceReplayValue(geometry, "Source", "运行快照"));
end
