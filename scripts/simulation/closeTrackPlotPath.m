function [plotX, plotY] = closeTrackPlotPath(track, x, y)
%CLOSETRACKPLOTPATH Append the first point when plotting a closed track.
%   Closed PathTracking tracks store one lap as unique periodic samples. This helper
%   adds only the display segment from the last sample back to the first;
%   it does not alter the simulation geometry or its sample count.

arguments
    track (1, 1) struct
    x (:, 1) double
    y (:, 1) double
end

assert(numel(x) == numel(y), "FSAE:TrackPlotPathSize", ...
    "Track plot X/Y vectors must have the same length.");

plotX = x;
plotY = y;
isClosed = isfield(track, "IsClosed") && isscalar(track.IsClosed) && ...
    logical(track.IsClosed);
if isClosed && ~isempty(plotX) && ...
        (plotX(end) ~= plotX(1) || plotY(end) ~= plotY(1))
    plotX(end + 1, 1) = plotX(1);
    plotY(end + 1, 1) = plotY(1);
end
end
