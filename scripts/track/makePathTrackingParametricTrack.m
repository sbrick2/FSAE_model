function track = makePathTrackingParametricTrack(x, y, sampleDistance, laneWidth, name, isClosed)
%MAKEPathTrackingPARAMETRICTRACK Resample a parametric centerline with PathTracking metadata.

arguments
    x (:, 1) double
    y (:, 1) double
    sampleDistance (1, 1) double {mustBePositive}
    laneWidth (1, 1) double {mustBePositive}
    name (1, 1) string
    isClosed (1, 1) logical
end

if isClosed
    x = x(1:end-1);
    y = y(1:end-1);
end
arc = [0; cumsum(hypot(diff(x), diff(y)))];
if isClosed
    totalLength = arc(end) + hypot(x(1) - x(end), y(1) - y(end));
else
    totalLength = arc(end);
end
sampleCount = max(2, round(totalLength / sampleDistance));
sampleDistance = totalLength / (sampleCount - double(~isClosed));
if isClosed
    query = (0:sampleCount-1)' * sampleDistance;
    xClosed = [x; x(1)];
    yClosed = [y; y(1)];
    arcClosed = [arc; totalLength];
    sampleX = interp1(arcClosed, xClosed, query, "pchip");
    sampleY = interp1(arcClosed, yClosed, query, "pchip");
else
    query = (0:sampleCount-1)' * sampleDistance;
    sampleX = interp1(arc, x, query, "pchip");
    sampleY = interp1(arc, y, query, "pchip");
end

if isClosed
    dx = (circshift(sampleX, -1) - circshift(sampleX, 1)) / (2 * sampleDistance);
    dy = (circshift(sampleY, -1) - circshift(sampleY, 1)) / (2 * sampleDistance);
    ddx = (circshift(sampleX, -1) - 2 * sampleX + circshift(sampleX, 1)) / sampleDistance^2;
    ddy = (circshift(sampleY, -1) - 2 * sampleY + circshift(sampleY, 1)) / sampleDistance^2;
else
    dx = gradient(sampleX, sampleDistance);
    dy = gradient(sampleY, sampleDistance);
    ddx = gradient(dx, sampleDistance);
    ddy = gradient(dy, sampleDistance);
end

track = struct( ...
    "Name", name, ...
    "SourceImage", "generated parametrically", ...
    "SourcePixelLength", NaN, ...
    "StartPixelRowColumn", [NaN, NaN], ...
    "IsClosed", isClosed, ...
    "Length", totalLength, ...
    "SampleDistance", sampleDistance, ...
    "X", sampleX(:), ...
    "Y", sampleY(:), ...
    "Heading", unwrap(atan2(dy, dx)), ...
    "Curvature", (dx .* ddy - dy .* ddx) ./ max((dx.^2 + dy.^2).^(3/2), eps), ...
    "ReferenceSpeed", 8.0 * ones(sampleCount, 1), ...
    "LeftHalfWidth", laneWidth / 2 * ones(sampleCount, 1), ...
    "RightHalfWidth", laneWidth / 2 * ones(sampleCount, 1), ...
    "WidthSource", "scenario parameter", ...
    "CoordinateConvention", "global x forward; global y left; left turn positive");
end
