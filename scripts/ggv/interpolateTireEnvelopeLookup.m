function envelopes = interpolateTireEnvelopeLookup(lookup, normalLoad)
%INTERPOLATETIREENVELOPELOOKUP Interpolate four MF62 support envelopes.

arguments
    lookup (1, 1) struct
    normalLoad (4, 1) double
end

requiredFields = ["NormalLoadGrid", "Directions", "Support", ...
    "FxMax", "FyMax", "NormalLoadUpper", "InterpolationMethod"];
assert(all(isfield(lookup, requiredFields)), ...
    "FSAE:QuasiStatic:InvalidTireEnvelopeLookup", ...
    "Tire-envelope lookup is missing required fields.");
assert(all(isfinite(normalLoad)) && all(normalLoad >= 0.0), ...
    "FSAE:QuasiStatic:TireEnvelopeLookupLoad", ...
    "Lookup normal loads must be finite and nonnegative.");
rangeTolerance = max(1.0e-9, 1.0e-10 * lookup.NormalLoadUpper);
assert(all(normalLoad <= lookup.NormalLoadUpper + rangeTolerance), ...
    "FSAE:QuasiStatic:TireEnvelopeLookupRange", ...
    "Normal load exceeds the precomputed tire-envelope lookup range.");
normalLoad = min(normalLoad, lookup.NormalLoadUpper);

directions = lookup.Directions;
directionCount = size(directions, 1);
template = struct("Directions", directions, ...
    "Support", zeros(directionCount, 1), "FxMax", 0.0, "FyMax", 0.0);
envelopes = repmat(template, 4, 1);
normalLoadGrid = lookup.NormalLoadGrid(:);
lowerIndex = sum(normalLoadGrid <= normalLoad', 1);
lowerIndex = max(1, min(lowerIndex, numel(normalLoadGrid) - 1));
upperIndex = lowerIndex + 1;
lowerLoad = normalLoadGrid(lowerIndex);
upperLoad = normalLoadGrid(upperIndex);
weight = (normalLoad' - lowerLoad') ./ (upperLoad' - lowerLoad');
for wheel = 1:4
    lowerWeight = 1.0 - weight(wheel);
    upperWeight = weight(wheel);
    envelopes(wheel).Support = (lowerWeight .* ...
        lookup.Support(lowerIndex(wheel), :, wheel) + upperWeight .* ...
        lookup.Support(upperIndex(wheel), :, wheel))';
    envelopes(wheel).FxMax = lowerWeight .* ...
        lookup.FxMax(lowerIndex(wheel), wheel) + upperWeight .* ...
        lookup.FxMax(upperIndex(wheel), wheel);
    envelopes(wheel).FyMax = lowerWeight .* ...
        lookup.FyMax(lowerIndex(wheel), wheel) + upperWeight .* ...
        lookup.FyMax(upperIndex(wheel), wheel);
end
end
