function [track, dataInfo] = loadOrCreatePathTrackingImageTrack( ...
        trackName, imagePath, options)
%LOADORCREATEPATHTRACKINGIMAGETRACK Persist image-derived track geometry.

arguments
    trackName (1, 1) string
    imagePath (1, 1) string
    options.SampleDistance (1, 1) double {mustBePositive} = 0.5
    options.ExtractionProfile (1, 1) string {mustBeMember( ...
        options.ExtractionProfile, ["fsec_endurance", "generic"])} = ...
        "generic"
    options.UseSavedData (1, 1) logical = true
    options.DataFolder (1, 1) string = ""
end

assert(isfile(imagePath), "FSAE:TrackImageMissing", ...
    "Track image does not exist: %s", imagePath);
sourceHash = calculateFileSHA256(imagePath);
parameterSignature = sprintf( ...
    "profile=%s;imageSHA256=%s;sample=%.17g;length=1000;laneWidth=3", ...
    options.ExtractionProfile, sourceHash, options.SampleDistance);
[track, dataInfo] = loadOrCreatePathTrackingTrackData( ...
    trackName, parameterSignature, ...
    @() buildImageTrack(imagePath, options.SampleDistance, ...
        options.ExtractionProfile), ...
    UseSavedData = options.UseSavedData, DataFolder = options.DataFolder);

% Identical image content may be loaded through a different path.
track.SourceImage = imagePath;
track.SourceImageSHA256 = sourceHash;
dataInfo.SourceImageSHA256 = sourceHash;
end

function track = buildImageTrack(imagePath, sampleDistance, profile)
switch profile
    case "fsec_endurance"
        track = extractFsecEnduranceTrack(imagePath, ...
            SampleDistance = sampleDistance, ...
            TotalLength = 1000.0, LaneWidth = 3.0);
    case "generic"
        track = extractTrackFromImage(imagePath, ...
            SampleDistance = sampleDistance, ...
            TotalLength = 1000.0, LaneWidth = 3.0);
end
track = orientPathTrackingClosedTrack(track, "counterclockwise");
end

function digest = calculateFileSHA256(filePath)
fileID = fopen(filePath, "rb");
assert(fileID >= 0, "FSAE:TrackImageUnreadable", ...
    "Unable to read track image: %s", filePath);
cleanup = onCleanup(@() fclose(fileID));
messageDigest = java.security.MessageDigest.getInstance("SHA-256");
while ~feof(fileID)
    bytes = fread(fileID, 1024 * 1024, "*uint8");
    if ~isempty(bytes)
        messageDigest.update(bytes);
    end
end
rawDigest = typecast(messageDigest.digest(), "uint8");
digest = upper(string(reshape(dec2hex(rawDigest, 2).', 1, [])));
clear cleanup
end
