function track = extractTrackFromImage(imagePath, options)
%EXTRACTTRACKFROMIMAGE Extract and resample a closed centerline from a map image.
%   The supplied image is expected to contain one dark/grey centerline on a
%   light background.  The image has no metric scale, so TotalLength is an
%   explicit normalization rather than a measured vehicle-track dimension.
%   RegionOfInterest, grey-level thresholds and optional blue track areas
%   make the extraction reproducible for annotated competition diagrams.

arguments
    imagePath (1, 1) string
    options.SampleDistance (1, 1) double {mustBePositive} = 0.5
    options.TotalLength (1, 1) double {mustBePositive} = 1000.0
    options.LaneWidth (1, 1) double {mustBePositive} = 3.0
    options.SmoothingDistance (1, 1) double {mustBeNonnegative} = 10.0
    options.StartPoint (1, 2) double = [NaN, NaN]
    options.TravelDirection (1, 1) string {mustBeMember(options.TravelDirection, ["up", "down"])} = "down"
    options.RegionOfInterest (1, 4) double = [NaN, NaN, NaN, NaN]
    options.GrayIntensityRange (1, 2) double = [0, 230]
    options.GrayChromaTolerance (1, 1) double {mustBeNonnegative} = 40
    options.IncludeBlueTrackAreas (1, 1) logical = false
    options.BlueTrackRegions (:, 4) double = zeros(0, 4)
    options.UseRedStartBridge (1, 1) logical = true
    options.SupplementalPathWaypoints (:, 2) double = zeros(0, 2)
    options.SupplementalPathRadius (1, 1) double {mustBeInteger, mustBeNonnegative} = 0
    options.MaskClosingRadius (1, 1) double {mustBeInteger, mustBeNonnegative} = 2
    options.MinimumComponentArea (1, 1) double {mustBeInteger, mustBePositive} = 100
    options.PruneSkeletonSpurs (1, 1) logical = false
    options.MinimumTraceCoverage (1, 1) double {mustBeGreaterThanOrEqual(options.MinimumTraceCoverage, 0), mustBeLessThanOrEqual(options.MinimumTraceCoverage, 1)} = 0.75
end

assert(isfile(imagePath), "FSAE:TrackImageMissing", ...
    "Track image does not exist: %s", imagePath);

image = imread(imagePath);
if ismatrix(image)
    image = repmat(image, 1, 1, 3);
end
image = im2double(image(:, :, 1:3));
assert(options.GrayIntensityRange(1) >= 0 && ...
    options.GrayIntensityRange(2) <= 255 && ...
    options.GrayIntensityRange(1) < options.GrayIntensityRange(2), ...
    "FSAE:TrackImageGrayRange", ...
    "GrayIntensityRange must be an increasing two-element range in [0, 255].");

channelRange = max(image, [], 3) - min(image, [], 3);
meanIntensity = mean(image, 3);
grayMask = channelRange <= options.GrayChromaTolerance / 255 & ...
    meanIntensity >= options.GrayIntensityRange(1) / 255 & ...
    meanIntensity <= options.GrayIntensityRange(2) / 255;

if options.IncludeBlueTrackAreas
    blueMask = image(:, :, 3) >= 110 / 255 & ...
        image(:, :, 3) - image(:, :, 1) >= 35 / 255 & ...
        image(:, :, 3) - image(:, :, 2) >= 10 / 255;
    if ~isempty(options.BlueTrackRegions)
        blueMask = blueMask & makeRegionUnionMask( ...
            size(grayMask), options.BlueTrackRegions);
    end
    grayMask = grayMask | blueMask;
end

if ~isempty(options.SupplementalPathWaypoints)
    grayMask = grayMask | makePolylineMask(size(grayMask), ...
        options.SupplementalPathWaypoints, ...
        options.SupplementalPathRadius);
end

roiMask = makeRegionMask(size(grayMask), options.RegionOfInterest);
grayMask = grayMask & roiMask;

redRows = zeros(0, 1);
redColumns = zeros(0, 1);
if options.UseRedStartBridge
    % A simple source image may use one red start marker that masks a short
    % section of the centreline. Annotated diagrams with red flags or a red
    % frame must disable this heuristic and provide StartPoint explicitly.
    redMask = image(:, :, 1) > 150 / 255 & ...
        image(:, :, 2) < 150 / 255 & image(:, :, 3) < 150 / 255;
    [redRows, redColumns] = find(redMask);
    if ~isempty(redRows)
        centerRow = round(mean(redRows));
        centerColumn = round(mean(redColumns));
        rowRange = max(1, centerRow - 5):min(size(grayMask, 1), centerRow + 5);
        columnRange = max(1, centerColumn - 6):min(size(grayMask, 2), centerColumn + 6);
        grayMask(rowRange, columnRange) = true;
    end
end

if exist("strel", "file") == 2 && options.MaskClosingRadius > 0
    grayMask = imclose(grayMask, ...
        strel("disk", options.MaskClosingRadius, 0));
end
grayMask = bwareaopen(grayMask, options.MinimumComponentArea);
grayMask = keepLargestComponent(grayMask);
if exist("bwskel", "file") == 2
    centerlineMask = bwskel(grayMask, MinBranchLength=20);
else
    centerlineMask = bwmorph(grayMask, "thin", Inf);
end
if options.PruneSkeletonSpurs
    centerlineMask = bwmorph(centerlineMask, "spur", Inf);
end
centerlineMask = keepLargestComponent(centerlineMask);

[rows, columns] = find(centerlineMask);
assert(~isempty(rows), "FSAE:TrackImageEmpty", ...
    "No centerline pixels were extracted from %s.", imagePath);

if any(isfinite(options.StartPoint))
    distances = hypot(columns - options.StartPoint(1), rows - options.StartPoint(2));
else
    assert(~isempty(redRows), "FSAE:TrackStartMissing", ...
        "A red start marker or explicit StartPoint is required.");
    distances = hypot(columns - mean(redColumns), rows - mean(redRows));
end
[~, startIndex] = min(distances);
startPixel = [rows(startIndex), columns(startIndex)];
startDistancePixels = hypot(startPixel(2) - options.StartPoint(1), ...
    startPixel(1) - options.StartPoint(2));
if all(isfinite(options.StartPoint))
    assert(startDistancePixels <= 30, "FSAE:TrackStartTooFar", ...
        "The closest centerline pixel is %.1f px from StartPoint.", ...
        startDistancePixels);
end

neighbors = @(point) findNeighbors(centerlineMask, point);
startNeighbors = neighbors(startPixel);
assert(numel(startNeighbors) >= 2, "FSAE:TrackStartDisconnected", ...
    "The extracted centerline is not connected at the start point.");
if options.TravelDirection == "down"
    [~, nextIndex] = max(startNeighbors(:, 1));
else
    [~, nextIndex] = min(startNeighbors(:, 1));
end
nextPixel = startNeighbors(nextIndex, :);

pathPixels = zeros(nnz(centerlineMask) + 2, 2);
pathPixels(1, :) = startPixel;
pathCount = 1;
currentPixel = startPixel;
visitedMask = false(size(centerlineMask));
visitedMask(startPixel(1), startPixel(2)) = true;
completedLoop = false;
while pathCount < size(pathPixels, 1)
    pathCount = pathCount + 1;
    pathPixels(pathCount, :) = nextPixel;
    previousPixel = currentPixel;
    currentPixel = nextPixel;
    visitedMask(currentPixel(1), currentPixel(2)) = true;
    candidates = neighbors(currentPixel);
    candidates = candidates(~all(candidates == previousPixel, 2), :);
    isStart = all(candidates == startPixel, 2);
    if pathCount > 100 && any(isStart)
        completedLoop = true;
        break
    end
    visited = visitedMask(sub2ind(size(visitedMask), ...
        candidates(:, 1), candidates(:, 2)));
    candidates = candidates(~visited, :);
    if isempty(candidates)
        break
    end
    incomingDirection = currentPixel - previousPixel;
    candidateDirection = candidates - currentPixel;
    continuationScore = candidateDirection * incomingDirection';
    [~, nextIndex] = max(continuationScore);
    nextPixel = candidates(nextIndex, :);
end
pathPixels = pathPixels(1:pathCount, :);
assert(completedLoop, "FSAE:TrackTraceOpen", ...
    "Centerline trace did not return to the start pixel.");
traceCoverage = pathCount / max(nnz(centerlineMask), 1);
assert(traceCoverage >= options.MinimumTraceCoverage, ...
    "FSAE:TrackTraceCoverage", ...
    "Centerline trace covered only %.1f%%%% of the retained skeleton.", ...
    100 * traceCoverage);

pixelXY = [pathPixels(:, 2), -pathPixels(:, 1)];
closedPixelXY = [pixelXY; pixelXY(1, :)];
pixelSteps = hypot(diff(closedPixelXY(:, 1)), diff(closedPixelXY(:, 2)));
pixelLength = sum(pixelSteps);
assert(pixelLength > 100, "FSAE:TrackLengthInvalid", ...
    "Extracted pixel centerline is unexpectedly short.");

sampleCount = max(20, round(options.TotalLength / options.SampleDistance));
sampleDistance = options.TotalLength / sampleCount;
pixelArc = [0; cumsum(pixelSteps(1:end-1))];
pixelArcClosed = [pixelArc; pixelLength];
pixelXYClosed = [pixelXY; pixelXY(1, :)];
queryArc = (0:sampleCount-1)' * pixelLength / sampleCount;
sampleX = interp1(pixelArcClosed, pixelXYClosed(:, 1), queryArc, "pchip");
sampleY = interp1(pixelArcClosed, pixelXYClosed(:, 2), queryArc, "pchip");

% Normalize the image centerline to the requested one-lap length and put
% the image start marker at global (0, 0).
pixelToMeter = options.TotalLength / pixelLength;
sampleX = (sampleX - sampleX(1)) * pixelToMeter;
sampleY = (sampleY - sampleY(1)) * pixelToMeter;
[sampleX, sampleY] = smoothClosedCenterline(sampleX, sampleY, ...
    sampleDistance, options.TotalLength, options.SmoothingDistance);

track = makeTrackFromSamples(sampleX, sampleY, sampleDistance, ...
    options.LaneWidth, imagePath, pixelLength, startPixel);
track.SourceRegionOfInterest = options.RegionOfInterest;
track.SourcePixelToMeter = pixelToMeter;
track.SourceTraceCoverage = traceCoverage;
track.SourceStartDistancePixels = startDistancePixels;
track.ExtractionSettings = struct( ...
    "GrayIntensityRange", options.GrayIntensityRange, ...
    "GrayChromaTolerance", options.GrayChromaTolerance, ...
    "IncludeBlueTrackAreas", options.IncludeBlueTrackAreas, ...
    "BlueTrackRegions", options.BlueTrackRegions, ...
    "UseRedStartBridge", options.UseRedStartBridge, ...
    "SupplementalPathWaypoints", options.SupplementalPathWaypoints, ...
    "SupplementalPathRadius", options.SupplementalPathRadius, ...
    "MaskClosingRadius", options.MaskClosingRadius, ...
    "PruneSkeletonSpurs", options.PruneSkeletonSpurs, ...
    "SmoothingDistance", options.SmoothingDistance, ...
    "TravelDirection", options.TravelDirection);
end

function mask = makeRegionMask(imageSize, regionOfInterest)
mask = true(imageSize);
if all(isnan(regionOfInterest))
    return
end
assert(all(isfinite(regionOfInterest)) && ...
    all(regionOfInterest(3:4) > 0), "FSAE:TrackImageROI", ...
    "RegionOfInterest must be [x, y, width, height] or all NaN.");

xFirst = max(1, round(regionOfInterest(1)));
yFirst = max(1, round(regionOfInterest(2)));
xLast = min(imageSize(2), round(sum(regionOfInterest([1, 3]))));
yLast = min(imageSize(1), round(sum(regionOfInterest([2, 4]))));
assert(xFirst <= xLast && yFirst <= yLast, "FSAE:TrackImageROIOutside", ...
    "RegionOfInterest does not overlap the supplied image.");
mask(:) = false;
mask(yFirst:yLast, xFirst:xLast) = true;
end

function mask = makeRegionUnionMask(imageSize, regions)
mask = false(imageSize);
for regionIndex = 1:size(regions, 1)
    mask = mask | makeRegionMask(imageSize, regions(regionIndex, :));
end
end

function mask = makePolylineMask(imageSize, waypoints, radius)
assert(size(waypoints, 1) >= 2 && all(isfinite(waypoints), "all"), ...
    "FSAE:TrackImageSupplementalPath", ...
    "SupplementalPathWaypoints must contain at least two finite [x, y] rows.");

segmentLength = hypot(diff(waypoints(:, 1)), diff(waypoints(:, 2)));
assert(all(segmentLength > 0), "FSAE:TrackImageSupplementalPath", ...
    "Consecutive SupplementalPathWaypoints must be distinct.");
pathArc = [0; cumsum(segmentLength)];
queryArc = (0:0.25:pathArc(end))';
if queryArc(end) < pathArc(end)
    queryArc(end + 1, 1) = pathArc(end);
end
columns = interp1(pathArc, waypoints(:, 1), queryArc, "pchip");
rows = interp1(pathArc, waypoints(:, 2), queryArc, "pchip");
columns = min(max(round(columns), 1), imageSize(2));
rows = min(max(round(rows), 1), imageSize(1));

mask = false(imageSize);
mask(sub2ind(imageSize, rows, columns)) = true;
if radius > 0
    mask = imdilate(mask, strel("disk", radius, 0));
end
end

function outputMask = keepLargestComponent(inputMask)
components = bwconncomp(inputMask, 8);
assert(components.NumObjects > 0, "FSAE:TrackImageEmpty", ...
    "No connected centerline component remains after image filtering.");
componentSizes = cellfun(@numel, components.PixelIdxList);
[~, largestIndex] = max(componentSizes);
outputMask = false(size(inputMask));
outputMask(components.PixelIdxList{largestIndex}) = true;
end

function [sampleX, sampleY] = smoothClosedCenterline( ...
    sampleX, sampleY, sampleDistance, totalLength, smoothingDistance)
if smoothingDistance <= 0
    return
end

sampleCount = numel(sampleX);
windowLength = max(5, 2 * floor(smoothingDistance / sampleDistance / 2) + 1);
halfWindow = floor(windowLength / 2);
kernelIndex = (-halfWindow:halfWindow)';
sigma = max(windowLength / 6, 1);
kernel = exp(-0.5 * (kernelIndex / sigma).^2);
kernel = kernel / sum(kernel);
wrappedIndex = mod((-halfWindow:sampleCount + halfWindow - 1)', ...
    sampleCount) + 1;
smoothedX = conv(sampleX(wrappedIndex), kernel, "same");
smoothedY = conv(sampleY(wrappedIndex), kernel, "same");
smoothedX = smoothedX(halfWindow + (1:sampleCount));
smoothedY = smoothedY(halfWindow + (1:sampleCount));

closedX = [smoothedX; smoothedX(1)];
closedY = [smoothedY; smoothedY(1)];
arc = [0; cumsum(hypot(diff(closedX), diff(closedY)))];
query = (0:sampleCount-1)' * arc(end) / sampleCount;
sampleX = interp1(arc, closedX, query, "pchip");
sampleY = interp1(arc, closedY, query, "pchip");
scale = totalLength / sum(hypot( ...
    circshift(sampleX, -1) - sampleX, ...
    circshift(sampleY, -1) - sampleY));
sampleX = (sampleX - sampleX(1)) * scale;
sampleY = (sampleY - sampleY(1)) * scale;
end

function points = findNeighbors(mask, point)
row = point(1);
column = point(2);
candidateRows = row + (-1:1);
candidateColumns = column + (-1:1);
points = zeros(8, 2);
pointCount = 0;
for rowIndex = candidateRows
    for columnIndex = candidateColumns
        if rowIndex == row && columnIndex == column
            continue
        end
        if rowIndex >= 1 && rowIndex <= size(mask, 1) && ...
                columnIndex >= 1 && columnIndex <= size(mask, 2) && ...
                mask(rowIndex, columnIndex)
            pointCount = pointCount + 1;
            points(pointCount, :) = [rowIndex, columnIndex];
        end
    end
end
points = points(1:pointCount, :);
end

function track = makeTrackFromSamples(sampleX, sampleY, sampleDistance, laneWidth, sourceImage, pixelLength, startPixel)
sampleX = sampleX(:);
sampleY = sampleY(:);
sampleCount = numel(sampleX);
dx = (circshift(sampleX, -1) - circshift(sampleX, 1)) / (2 * sampleDistance);
dy = (circshift(sampleY, -1) - circshift(sampleY, 1)) / (2 * sampleDistance);
ddx = (circshift(sampleX, -1) - 2 * sampleX + circshift(sampleX, 1)) / sampleDistance^2;
ddy = (circshift(sampleY, -1) - 2 * sampleY + circshift(sampleY, 1)) / sampleDistance^2;
heading = unwrap(atan2(dy, dx));
curvature = (dx .* ddy - dy .* ddx) ./ max((dx.^2 + dy.^2).^(3/2), eps);

track = struct( ...
    "Name", "AutocrossTrackMap", ...
    "SourceImage", string(sourceImage), ...
    "SourcePixelLength", pixelLength, ...
    "StartPixelRowColumn", startPixel, ...
    "IsClosed", true, ...
    "Length", sampleCount * sampleDistance, ...
    "SampleDistance", sampleDistance, ...
    "X", sampleX, ...
    "Y", sampleY, ...
    "Heading", heading, ...
    "Curvature", curvature, ...
    "ReferenceSpeed", 12.0 * ones(sampleCount, 1), ...
    "LeftHalfWidth", laneWidth / 2 * ones(sampleCount, 1), ...
    "RightHalfWidth", laneWidth / 2 * ones(sampleCount, 1), ...
    "WidthSource", "temporary 3 m lane placeholder; width is not encoded in track_map.png", ...
    "CoordinateConvention", "global x forward at start; global y left; image y is inverted");
end
