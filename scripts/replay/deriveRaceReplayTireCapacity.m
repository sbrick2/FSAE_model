function capacity = deriveRaceReplayTireCapacity(wheel, tire, metadata, threshold)
%DERIVERACEREPLAYTIRECAPACITY Reproduce tire-model reference utilization.
arguments
    wheel (1, 1) struct
    tire (1, 1) struct
    metadata (1, 1) struct
    threshold (1, 1) double {mustBeNonnegative} = 1
end
capacity = struct("X", NaN(1, 4), "Y", NaN(1, 4), ...
    "RoadLimit", NaN(1, 4), "RawUtilization", NaN(1, 4), ...
    "Utilization", NaN(1, 4), "Available", false(1, 4), ...
    "NoGrip", false(1, 4), ...
    "LowLoad", isfinite(wheel.NormalLoad) & wheel.NormalLoad <= threshold);
model = lower(string(raceReplayValue(metadata, "Model", "")));
grip = roadValue(metadata, "RoadGripScale", NaN);
limit = roadValue(metadata, "RoadMuLimit", NaN);
grip(grip < 0) = 0;
limit(limit < 0) = 0;
capacity.NoGrip = isfinite(grip) & ...
    (grip == 0 | (isfinite(limit) & limit == 0));
fz = max(wheel.NormalLoad, 0);
profile = raceReplayValue(metadata, "Profile", struct);
parameters = raceReplayValue(metadata, "Parameters", []);
if model == "mf62" && isnumeric(parameters) && numel(parameters) >= 29
    p = double(parameters(:));
    profile.Reference = struct("NormalLoadN", p(2), "PressurePa", p(3));
    profile.InputPressurePa = p(1);
    profile.Longitudinal = struct("Mu0", p(7), "MuLoad", p(8), ...
        "MuPressure", p(9), "MuCamber2", p(10), "MuAsymmetry", p(11), ...
        "VerticalShift", p(13));
    profile.Lateral = struct("Mu0", p(17), "MuLoad", p(18), ...
        "MuPressure", p(19), "MuCamber2", p(20), ...
        "VerticalShift", p(23), "CamberVerticalShift", p(24));
end
if model == "mf62" && all(isfield(profile, ...
        {'Reference', 'Longitudinal', 'Lateral', 'InputPressurePa'}))
    long = profile.Longitudinal;
    lat = profile.Lateral;
    loadDelta = (fz - profile.Reference.NormalLoadN) ./ ...
        max(profile.Reference.NormalLoadN, 1);
    pressureDelta = (profile.InputPressurePa - profile.Reference.PressurePa) / ...
        max(profile.Reference.PressurePa, 1);
    muX = max(long.Mu0 + long.MuLoad .* loadDelta + ...
        long.MuPressure .* pressureDelta + long.MuCamber2 .* ...
        wheel.CamberAngle.^2 + long.MuAsymmetry .* tanh(50 * wheel.SlipRatio), 0.05);
    muY = max(lat.Mu0 + lat.MuLoad .* loadDelta + ...
        lat.MuPressure .* pressureDelta + lat.MuCamber2 .* wheel.CamberAngle.^2, 0.05);
    capacity.X = grip .* fz .* max(abs(muX) + abs(long.VerticalShift), 0.05);
    capacity.Y = grip .* fz .* max(abs(muY) + abs(lat.VerticalShift + ...
        lat.CamberVerticalShift .* wheel.CamberAngle), 0.05);
    inputValid = isfinite(wheel.NormalLoad) & isfinite(wheel.CamberAngle) & ...
        isfinite(wheel.SlipRatio) & isfinite(grip);
    capacity.X(~inputValid) = NaN;
    capacity.Y(~inputValid) = NaN;
elseif model == "ttc_map"
    mu = raceReplayValue(metadata, "MapPeakMu", NaN);
    capacity.X = grip .* fz .* mu;
    capacity.Y = capacity.X;
elseif model == "simple"
    capacity.X = min(max(grip, 0), max(limit, 0)) .* fz;
    capacity.Y = capacity.X;
    bad = ~isfinite(grip) | isnan(limit);
    capacity.X(bad) = NaN;
    capacity.Y(bad) = NaN;
end
capacity.RoadLimit = limit .* fz;
capacity.RoadLimit(~isfinite(limit) | ~isfinite(wheel.NormalLoad)) = NaN;
capacity.X(~isfinite(wheel.NormalLoad)) = NaN;
capacity.Y(~isfinite(wheel.NormalLoad)) = NaN;
valid = isfinite(capacity.X) & isfinite(capacity.Y) & ...
    capacity.X > 0 & capacity.Y > 0 & ~capacity.LowLoad & ~capacity.NoGrip;
capacity.Available = valid;
raw = hypot(tire.Fx ./ max(capacity.X, 1), tire.Fy ./ max(capacity.Y, 1));
if model == "simple"
    epsilon = raceReplayValue(metadata, "ForceEpsilon", NaN);
    if isscalar(epsilon) && isfinite(epsilon) && epsilon > 0
        raw = hypot(tire.Fx, tire.Fy) ./ sqrt(capacity.X.^2 + epsilon.^2);
    else
        raw(:) = NaN;
    end
end
raw(~valid) = NaN;
roadValid = model ~= "simple" & isfinite(limit) & limit > 0 & isfinite(fz) & ...
    fz > threshold & isfinite(tire.Fx) & isfinite(tire.Fy);
roadUtilization = hypot(tire.Fx, tire.Fy) ./ max(capacity.RoadLimit, 1);
missing = roadValid & ~isfinite(raw);
raw(missing) = roadUtilization(missing);
validRoad = roadValid & isfinite(raw);
raw(validRoad) = max(raw(validRoad), roadUtilization(validRoad));
raw(capacity.LowLoad & isfinite(tire.Fx) & isfinite(tire.Fy)) = 0;
raw(capacity.NoGrip & isfinite(tire.Fx) & isfinite(tire.Fy)) = 0;
capacity.RawUtilization = raw;
capacity.Utilization = min(raw, 1);
capacity.Utilization(~isfinite(raw)) = NaN;
end

function value = roadValue(data, field, fallback)
value = raceReplayValue(data, field, fallback);
if ~isnumeric(value) || ~(isscalar(value) || numel(value) == 4)
    value = NaN(1, 4);
elseif isscalar(value)
    value = repmat(double(value), 1, 4);
else
    value = reshape(double(value), 1, 4);
end
end
