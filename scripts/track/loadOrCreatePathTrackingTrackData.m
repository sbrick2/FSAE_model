function [track, dataInfo] = loadOrCreatePathTrackingTrackData( ...
        trackName, parameterSignature, buildFunction, options)
%LOADORCREATEPATHTRACKINGTRACKDATA Load or create a named track MAT file.
%   Each track is stored as <project>/trackdata/<trackName>.mat. Geometry
%   parameters are recorded in metadata; a mismatch regenerates and
%   replaces that named file.

arguments
    trackName (1, 1) string
    parameterSignature (1, 1) string
    buildFunction (1, 1) function_handle
    options.UseSavedData (1, 1) logical = true
    options.DataFolder (1, 1) string = ""
end

assert(strlength(trackName) > 0, "FSAE:TrackDataName", ...
    "Track data name must not be empty.");
nameToken = regexprep(lower(trackName), "[^0-9a-z]+", "_");
assert(nameToken == lower(trackName), "FSAE:TrackDataName", ...
    "Track data name must contain only lowercase letters, digits, or underscores.");

projectRoot = string(fileparts(fileparts(fileparts( ...
    mfilename("fullpath")))));
dataFolder = options.DataFolder;
if strlength(dataFolder) == 0
    dataFolder = fullfile(projectRoot, "trackdata");
end
dataFile = fullfile(dataFolder, nameToken + ".mat");

dataVersion = "path_tracking_track_data_v1";
expectedMetadata = struct( ...
    "Version", dataVersion, ...
    "TrackName", trackName, ...
    "ParameterSignature", parameterSignature);

loaded = false;
if options.UseSavedData && isfile(dataFile)
    try
        saved = load(dataFile, "track", "metadata");
        if isValidTrackData(saved, expectedMetadata)
            track = saved.track;
            loaded = true;
        end
    catch
        % An incomplete or obsolete data file is regenerated below.
    end
end

if ~loaded
    track = buildFunction();
    assert(isstruct(track) && isscalar(track), ...
        "FSAE:TrackDataBuilderResult", ...
        "Track data builder must return one scalar structure.");
    if options.UseSavedData
        if ~isfolder(dataFolder)
            mkdir(dataFolder);
        end
        metadata = expectedMetadata;
        temporaryFile = string(tempname(dataFolder)) + ".mat";
        temporaryCleanup = onCleanup(@() deleteIfPresent(temporaryFile));
        save(temporaryFile, "track", "metadata", "-v7");
        movefile(temporaryFile, dataFile, "f");
        clear temporaryCleanup
    end
end

dataInfo = struct( ...
    "Enabled", logical(options.UseSavedData), ...
    "Loaded", logical(loaded), ...
    "File", dataFile, ...
    "Version", dataVersion, ...
    "ParameterSignature", parameterSignature);
end

function valid = isValidTrackData(saved, expected)
valid = isfield(saved, "track") && ...
    isstruct(saved.track) && isscalar(saved.track) && ...
    isfield(saved, "metadata") && ...
    isstruct(saved.metadata) && isscalar(saved.metadata);
if ~valid
    return
end
actual = saved.metadata;
required = ["Version", "TrackName", "ParameterSignature"];
valid = all(isfield(actual, cellstr(required))) && ...
    string(actual.Version) == expected.Version && ...
    string(actual.TrackName) == expected.TrackName && ...
    string(actual.ParameterSignature) == expected.ParameterSignature;
end

function deleteIfPresent(filePath)
if isfile(filePath)
    delete(filePath);
end
end
