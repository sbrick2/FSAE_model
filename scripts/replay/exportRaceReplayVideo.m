function report = exportRaceReplayVideo(data, filePath, options)
%EXPORTRACEREPLAYVIDEO Render fixed-time dashboard frames to an MP4.
%   Uses a dedicated graphics figure and temporary output. Cancellation leaves an
%   existing destination untouched. Does not require Image Processing Toolbox.
arguments
    data (1, 1) struct
    filePath (1, 1) string
    options.FrameRate (1, 1) double {mustBePositive, mustBeFinite} = 30
    options.Resolution (1, 2) double {mustBePositive, mustBeInteger} = [1920, 1080]
    options.TimeRange (1, 2) double = [NaN, NaN]
    options.Speed (1, 1) double {mustBePositive, mustBeFinite} = 1
    options.View (1, 1) string {mustBeMember(options.View, ...
        ["north", "heading", "overview"])} = "north"
    options.FollowSpan (1, 1) double {mustBePositive} = 22
    options.ShowValues (1, 1) logical = true
    options.ShowStats (1, 1) logical = true
    options.Renderer (1, 1) string {mustBeMember(options.Renderer, ["graphics", "ui"])} = "graphics"
    options.Overwrite (1, 1) logical = false
    options.ProgressFcn (1, 1) function_handle = @(~) []
    options.CancelFcn (1, 1) function_handle = @() false
end
assert(all(mod(options.Resolution, 2) == 0), ...
    "FSAE:Replay:VideoResolution", "MPEG-4 width and height must be even.");
range = options.TimeRange;
if all(isnan(range))
    range = [data.Time(1), data.Time(end)];
end
assert(all(isfinite(range)) && range(1) <= range(2) && ...
    range(1) >= data.Time(1) && range(2) <= data.Time(end), ...
    "FSAE:Replay:VideoRange", "TimeRange must lie within the result time span.");
[folder, ~, extension] = fileparts(filePath);
assert(strcmpi(extension, ".mp4"), "FSAE:Replay:VideoExtension", ...
    "Choose an .mp4 output file.");
if folder == ""
    folder = string(pwd);
    filePath = fullfile(folder, filePath);
end
assert(isfolder(folder), "FSAE:Replay:VideoFolder", "Output folder does not exist.");
assert(options.Overwrite || ~isfile(filePath), ...
    "FSAE:Replay:VideoExists", "Output exists; explicitly enable Overwrite.");
temporaryFile = string(tempname(folder)) + ".mp4";
resources = RaceReplayVideoResources;
resources.TemporaryFile = temporaryFile;
cleanup = onCleanup(@() resources.release());
duration = (range(2) - range(1)) / options.Speed;
count = max(1, ceil(duration * options.FrameRate - 1e-10));
report = struct("File", filePath, "FrameCount", 0, ...
    "FrameRate", options.FrameRate, "Resolution", options.Resolution, ...
    "Duration", 0, "TimeRange", range, "Speed", options.Speed, "Cancelled", false, ...
    "Renderer", options.Renderer, "ElapsedSeconds", 0, ...
    "Timing", struct("Setup", 0, "Sample", 0, "Render", 0, ...
        "Capture", 0, "Resize", 0, "Encode", 0, "Finalize", 0));
exportClock = tic;
if options.CancelFcn()
    report.Cancelled = true;
    return
end
if options.Renderer == "graphics"
    % Keep labels and axes well formed at small output sizes, then resize
    % the captured image to the exact selected dimensions below.
    canvasResolution = max(options.Resolution, [1280, 720]);
    figureHandle = figure(Name = "FSAE 视频导出", Visible = "off", ...
        WindowStyle = "normal", NumberTitle = "off", MenuBar = "none", ToolBar = "none", ...
        Position = [30, 30, canvasResolution], Resize = "off");
    resources.Figure = figureHandle;
    view = createRaceReplayView(figureHandle, data, Renderer = "graphics");
else
    figureHandle = uifigure(Name = "FSAE 视频导出", Visible = "off", ...
        Position = [30, 30, options.Resolution], Resize = "off");
    resources.Figure = figureHandle;
    gridHandle = uigridlayout(figureHandle, [1, 1], Padding = [0, 0, 0, 0]);
    view = createRaceReplayView(gridHandle, data);
end
figureHandle.CloseRequestFcn = @(~, ~) setappdata(figureHandle, "CancelExport", true);
setappdata(figureHandle, "CancelExport", false);
writer = VideoWriter(temporaryFile, "MPEG-4");
resources.Writer = writer;
writer.FrameRate = options.FrameRate;
writer.Quality = 95;
open(writer);
resources.WriterOpen = true;
drawnow;
report.Timing.Setup = toc(exportClock);
for index = 1:count
    if options.CancelFcn() || getappdata(figureHandle, "CancelExport")
        report.Cancelled = true;
        report.ElapsedSeconds = toc(exportClock);
        return
    end
    time = min(range(2), range(1) + (index - 1) * options.Speed / options.FrameRate);
    if index == count
        time = range(2);
    end
    stageClock = tic;
    frame = sampleRaceReplayFrame(data, time);
    report.Timing.Sample = report.Timing.Sample + toc(stageClock);
    stageClock = tic;
    renderRaceReplayFrame(view, data, frame, View = options.View, ...
        FollowSpan = options.FollowSpan, ShowValues = options.ShowValues, ...
        ShowStats = options.ShowStats);
    report.Timing.Render = report.Timing.Render + toc(stageClock);
    stageClock = tic;
    % getframe synchronizes and draws the figure itself. Another drawnow in
    % this loop repeats that work and does not change the sampling clock.
    captured = getframe(figureHandle);
    report.Timing.Capture = report.Timing.Capture + toc(stageClock);
    stageClock = tic;
    rgb = resizeRGB(captured.cdata, options.Resolution);
    report.Timing.Resize = report.Timing.Resize + toc(stageClock);
    if options.CancelFcn() || getappdata(figureHandle, "CancelExport")
        report.Cancelled = true;
        report.ElapsedSeconds = toc(exportClock);
        return
    end
    stageClock = tic;
    writeVideo(writer, rgb);
    report.Timing.Encode = report.Timing.Encode + toc(stageClock);
    report.FrameCount = index;
    report.Duration = index / options.FrameRate;
    report.ElapsedSeconds = toc(exportClock);
    options.ProgressFcn(struct("Fraction", index / count, ...
        "FrameCount", index, "TotalFrames", count, "Time", time));
end
if options.CancelFcn() || getappdata(figureHandle, "CancelExport")
    report.Cancelled = true;
    report.ElapsedSeconds = toc(exportClock);
    return
end
stageClock = tic;
close(writer);
resources.WriterOpen = false;
[success, message] = movefile(temporaryFile, filePath, "f");
assert(success, "FSAE:Replay:VideoPublish", "%s", message);
clear cleanup
report.Timing.Finalize = toc(stageClock);
report.ElapsedSeconds = toc(exportClock);
end

function rgb = resizeRGB(rgb, resolution)
height = resolution(2);
width = resolution(1);
[sourceHeight, sourceWidth, ~] = size(rgb);
if sourceHeight == height && sourceWidth == width
    return
end
% Two separable linear interpolations keep output dimensions independent of
% Windows display scaling without introducing another toolbox dependency.
values = reshape(double(rgb), sourceHeight, sourceWidth * 3);
values = interp1((1:sourceHeight).', values, linspace(1, sourceHeight, height).');
values = permute(reshape(values, height, sourceWidth, 3), [2, 1, 3]);
values = reshape(values, sourceWidth, height * 3);
values = interp1((1:sourceWidth).', values, linspace(1, sourceWidth, width).');
rgb = uint8(permute(reshape(values, width, height, 3), [2, 1, 3]));
end
