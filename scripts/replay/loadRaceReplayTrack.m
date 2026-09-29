function [track, source] = loadRaceReplayTrack(result, sourceFile, projectRoot)
%LOADRACEREPLAYTRACK Find recorded or event-matched track data without rebuilding.
arguments
    result (1, 1) struct
    sourceFile (1, 1) string = ""
    projectRoot (1, 1) string = ""
end
track = struct;
source = "赛道几何未记录；仅显示车辆轨迹";
resultFolder = string(fileparts(sourceFile));
if projectRoot == ""
    projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
    % A relocated result can belong to another checkout of the project.
    ancestorFolder = resultFolder;
    while ancestorFolder ~= ""
        if isfolder(fullfile(ancestorFolder, "trackdata"))
            projectRoot = ancestorFolder;
            break
        end
        next = string(fileparts(ancestorFolder));
        if next == ancestorFolder, break; end
        ancestorFolder = next;
    end
end
roots = unique([projectRoot; resultFolder], "stable");
roots(roots == "") = [];
name = lower(readText(result, "Config.Track.Name"));
if name == "", name = lower(readText(result, "Meta.Event")); end
candidates = strings(0, 1);
for field = ["Meta.TrackDataFile", "Meta.TrackFile", "Config.Track.DataFile", "Config.Track.File"]
    file = readText(result, field);
    if file ~= "", candidates = [candidates; resolveFile(file, roots)]; end %#ok<AGROW>
end
if ~isempty(regexp(name, '^[a-z0-9_]+$', 'once'))
    folder = readText(result, "Config.Track.DataFolder");
    if folder ~= ""
        candidates = [candidates; resolveFile(fullfile(folder, name + ".mat"), roots)];
    end
    for root = roots.'
        candidates(end + 1, 1) = fullfile(root, "trackdata", name + ".mat"); %#ok<AGROW>
    end
end
for file = unique(candidates, "stable").'
    if ~isfile(file), continue; end
    try
        loaded = load(file);
        candidate = raceReplayValue(loaded, "track", raceReplayValue(loaded, "Track", struct));
        actualName = lower(readText(loaded, "metadata.TrackName"));
        if name ~= "" && actualName ~= "" && name ~= actualName, continue; end
        if ~validTrack(candidate), continue; end
        track = candidate;
        source = "自动加载赛道文件：" + file;
        return
    catch
        % Missing/corrupt automatic candidates must not prevent trajectory replay.
    end
end
end

function files = resolveFile(file, roots)
if startsWith(file, ["/", "\"]) || ~isempty(regexp(file, '^[A-Za-z]:[\\/]', 'once'))
    files = file;
else
    files = strings(numel(roots), 1);
    for index = 1:numel(roots), files(index) = fullfile(roots(index), file); end
end
end

function text = readText(value, field)
value = raceReplayValue(value, field, "");
text = "";
if ischar(value) || (isstring(value) && isscalar(value))
    text = strtrim(string(value));
end
end

function valid = validTrack(track)
valid = isstruct(track) && isscalar(track) && isfield(track, "X") && isfield(track, "Y");
if ~valid, return; end
x = track.X;
y = track.Y;
valid = isnumeric(x) && isnumeric(y) && isvector(x) && isvector(y) && ...
    numel(x) >= 2 && numel(x) == numel(y) && all(isfinite(x)) && all(isfinite(y));
if ~valid, return; end
for field = ["Heading", "LeftHalfWidth", "RightHalfWidth"]
    value = raceReplayValue(track, field, NaN);
    if field == "Heading" && isempty(value), continue; end
    valid = isnumeric(value) && (isscalar(value) || numel(value) == numel(x));
    if contains(field, "HalfWidth"), valid = valid && ~any(value(:) < 0); end
    if ~valid, return; end
end
closed = raceReplayValue(track, "IsClosed", false);
lengthValue = raceReplayValue(track, "Length", NaN);
valid = (islogical(closed) || isnumeric(closed)) && isscalar(closed) && ...
    isreal(closed) && isfinite(closed) && isnumeric(lengthValue) && ...
    isscalar(lengthValue) && isreal(lengthValue);
end
