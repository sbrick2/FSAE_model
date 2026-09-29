function receipt = verifyRaceReplayArtifacts(resultFiles, snapshot, folder)
%VERIFYRACEREPLAYARTIFACTS Real-result geometry, rendering and video checks.
arguments
    resultFiles (1, :) string
    snapshot (1, 1) struct
    folder (1, 1) string
end
receipt = struct("Environment", string(version));
receipt.ResultChecks = repmat(struct("File", "", "Samples", 0, ...
    "ComparedFrames", 0, "MaxUtilizationError", NaN), 1, numel(resultFiles));
for index = 1:numel(resultFiles)
    data = prepareRaceReplayData(resultFiles(index), Snapshot = snapshot);
    samples = unique(round(linspace(1, numel(data.Time), 21)));
    differences = [];
    for sample = samples
        frame = sampleRaceReplayFrame(data, data.Time(sample));
        valid = frame.Tire.Capacity.Available & ...
            isfinite(frame.Tire.RecordedUtilization);
        differences = [differences, abs(frame.Tire.Capacity.Utilization(valid) - ...
            frame.Tire.RecordedUtilization(valid))]; %#ok<AGROW>
        validPower = isfinite(frame.Motor.Torque) & isfinite(frame.Motor.Omega);
        assert(isequaln(frame.Motor.Power(validPower), ...
            frame.Motor.Torque(validPower) .* frame.Motor.Omega(validPower)));
    end
    assert(~isempty(differences) && max(differences) < 1e-8, ...
        "FSAE:Replay:RealUtilization", "Reference utilization differs from recorded values.");
    receipt.ResultChecks(index) = struct("File", resultFiles(index), ...
        "Samples", numel(data.Time), "ComparedFrames", numel(samples), ...
        "MaxUtilizationError", max(differences));
end
sizes = [1366, 768; 1920, 1080];
receipt.Rendering = repmat(struct("Resolution", [0, 0], "Frames", 0, ...
    "ElapsedSeconds", 0, "UpdatesPerSecond", 0, "Preview", ""), 1, size(sizes, 1));
for index = 1:size(sizes, 1)
    app = FSAERaceReplayApp(resultFiles(end), Snapshot = snapshot, ...
        WindowSize = sizes(index, :), Visible = "off");
    cleanup = onCleanup(@() deleteIfValid(app));
    app.seek(10);
    app.setView("heading");
    drawnow;
    imageFile = fullfile(folder, sprintf('replay_%dx%d.png', sizes(index, :)));
    exportapp(app.UIFigure, imageFile);
    count = 25;
    clock = tic;
    for frameIndex = 1:count
        frame = sampleRaceReplayFrame(app.Data, 10 + frameIndex / 20);
        renderRaceReplayFrame(app.View, app.Data, frame, View = "heading", FollowSpan = 18);
        drawnow;
    end
    seconds = toc(clock);
    receipt.Rendering(index) = struct("Resolution", sizes(index, :), ...
        "Frames", count, "ElapsedSeconds", seconds, ...
        "UpdatesPerSecond", count / seconds, "Preview", string(imageFile));
    if index == size(sizes, 1)
        videoFile = fullfile(folder, "replay_example_1080p.mp4");
        receipt.Video = app.exportVideo(videoFile, TimeRange = [10, 13], ...
            FrameRate = 30, Resolution = [1920, 1080], ShowProgress = false, Overwrite = true);
        video = VideoReader(videoFile);
        assert(video.Width == 1920 && video.Height == 1080 && video.FrameRate == 30);
        assert(abs(video.Duration - 3) <= 1 / 30 + 1e-8);
        assert(receipt.Video.FrameCount == 90);
        imwrite(readFrame(video), fullfile(folder, "replay_video_first_frame.png"));
    end
    clear cleanup
end
end

function deleteIfValid(app)
if isvalid(app), delete(app); end
end
