function app = launchFSAERaceReplay(source, options)
%LAUNCHFSAERACEREPLAY Open a saved lap result or normalized result struct.
%   app = launchFSAERaceReplay(resultFile)
%   app = launchFSAERaceReplay(resultFile, TrackFile="trackdata/autocross.mat")
%   app = launchFSAERaceReplay(result, Snapshot=snapshot)
arguments
    source = []
    options.Visible (1, 1) string = "on"
    options.TrackFile (1, 1) string = ""
    options.Track (1, 1) struct = struct
    options.Snapshot (1, 1) struct = struct
    options.FrameRate (1, 1) double = 20
    options.WindowSize (1, 2) double = [1400, 850]
    options.Parent = []
    options.ProjectRoot (1, 1) string = ""
end
root = string(fileparts(mfilename("fullpath")));
addpath(fullfile(root, "apps"), fullfile(root, "scripts", "replay"), ...
    fullfile(root, "scripts", "reporting"), "-begin");
if isempty(source)
    [file, folder] = uigetfile('*.mat', '选择圈速结果', ...
        char(fullfile(root, "results", "time_domain_closed_loop")));
    if isequal(file, 0), app = []; return; end
    source = string(fullfile(folder, file));
end
args = namedargs2cell(options);
app = FSAERaceReplayApp(source, args{:});
end
