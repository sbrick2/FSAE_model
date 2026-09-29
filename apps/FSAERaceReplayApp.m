classdef FSAERaceReplayApp < handle
    %FSAERACEREPLAYAPP Synchronized race replay, wheel forces and motor telemetry.
    properties (SetAccess = private)
        UIFigure
        View
        Data
        LastFrame
        CurrentTime (1, 1) double = 0
        State (1, 1) string = "Paused"
        PlaybackSpeed (1, 1) double = 1
        PlayButton
        SpeedDropDown
        ViewDropDown
        SpanField
        ValuesCheckBox
        StatsCheckBox
        TimeSlider
        TimeLabel
        StatusLabel
        ExportButton
        PlaybackTimer
    end
    properties (Access = private)
        Source
        SnapshotOverride = struct
        TrackOverride = struct
        TrackFile (1, 1) string = ""
        RootGrid
        ContentHost
        ControlGrid
        TimelineGrid
        AnchorTime (1, 1) double = 0
        PlaybackClock
        Rendering (1, 1) logical = false
        ExportCancelRequested (1, 1) logical = false
        PendingClose (1, 1) logical = false
        ProgressDialog
        ExportDialog
        WindowSize (1, 2) double = [1400, 850]
        Parent = []
        OwnsFigure (1, 1) logical = true
        ParentDestroyedListener = []
        ProjectRoot (1, 1) string = ""
    end
    methods
        function app = FSAERaceReplayApp(source, options)
            arguments
                source
                options.Visible (1, 1) string {mustBeMember(options.Visible, ["on", "off"])} = "on"
                options.TrackFile (1, 1) string = ""
                options.Track (1, 1) struct = struct
                options.Snapshot (1, 1) struct = struct
                options.FrameRate (1, 1) double {mustBePositive, mustBeFinite} = 20
                options.WindowSize (1, 2) double {mustBePositive, mustBeFinite} = [1400, 850]
                options.Parent = []
                options.ProjectRoot (1, 1) string = ""
            end
            app.Source = source;
            app.TrackFile = options.TrackFile;
            app.TrackOverride = options.Track;
            app.SnapshotOverride = options.Snapshot;
            app.WindowSize = options.WindowSize;
            app.Parent = options.Parent;
            app.OwnsFigure = isempty(options.Parent);
            app.ProjectRoot = options.ProjectRoot;
            app.Data = prepareRaceReplayData(source, TrackFile = app.TrackFile, ...
                Track = app.TrackOverride, Snapshot = app.SnapshotOverride, ProjectRoot = app.ProjectRoot);
            try
                app.createComponents();
                app.PlaybackTimer = timer(Name = "FSAERaceReplay", ...
                    ExecutionMode = "fixedRate", BusyMode = "drop", ...
                    Period = max(0.02, 1 / options.FrameRate), ...
                    TimerFcn = @(~, ~) app.onTick(), ...
                    ErrorFcn = @(~, ~) app.pausePlayback());
                app.updateFrame(app.Data.Time(1));
                app.initializeSceneLayout();
                if app.OwnsFigure
                    app.UIFigure.Visible = options.Visible;
                else
                    app.ParentDestroyedListener = addlistener(app.UIFigure, "ObjectBeingDestroyed", ...
                        @(~, ~) delete(app));
                end
            catch exception
                delete(app);
                rethrow(exception)
            end
        end

        function delete(app)
            app.State = "Closing";
            if ~isempty(app.ParentDestroyedListener)
                delete(app.ParentDestroyedListener);
                app.ParentDestroyedListener = [];
            end
            if ~isempty(app.PlaybackTimer) && isvalid(app.PlaybackTimer)
                stop(app.PlaybackTimer);
                delete(app.PlaybackTimer);
            end
            if ~isempty(app.ExportDialog) && isgraphics(app.ExportDialog)
                delete(app.ExportDialog);
            end
            if ~isempty(app.ProgressDialog) && isvalid(app.ProgressDialog)
                delete(app.ProgressDialog);
            end
            if app.OwnsFigure && ~isempty(app.UIFigure) && isgraphics(app.UIFigure)
                app.UIFigure.CloseRequestFcn = [];
                delete(app.UIFigure);
            elseif ~isempty(app.RootGrid) && isgraphics(app.RootGrid) && app.RootGrid.BeingDeleted == "off"
                delete(app.RootGrid);
            end
        end

        function loadResult(app, source)
            arguments
                app
                source
            end
            app.assertInteractive();
            prepared = prepareRaceReplayData(source, ProjectRoot = app.ProjectRoot);
            app.Source = source;
            app.TrackFile = "";
            app.TrackOverride = struct;
            app.SnapshotOverride = struct;
            app.replaceData(prepared);
        end

        function seek(app, time)
            arguments
                app
                time (1, 1) double {mustBeFinite}
            end
            app.assertInteractive();
            app.pausePlayback();
            app.State = "Seeking";
            app.updateFrame(time);
            app.State = "Paused";
            app.updateStatus();
            drawnow limitrate
        end

        function step(app, direction)
            arguments
                app
                direction (1, 1) double {mustBeMember(direction, [-1, 1])}
            end
            app.assertInteractive();
            if direction > 0
                index = find(app.Data.Time > app.CurrentTime + 1e-10, 1);
                if isempty(index), index = numel(app.Data.Time); end
            else
                index = find(app.Data.Time < app.CurrentTime - 1e-10, 1, "last");
                if isempty(index), index = 1; end
            end
            app.seek(app.Data.Time(index));
        end

        function play(app)
            app.assertInteractive();
            if numel(app.Data.Time) < 2 || app.State == "Playing"
                return
            end
            if app.CurrentTime >= app.Data.Time(end)
                app.updateFrame(app.Data.Time(1));
            end
            app.AnchorTime = app.CurrentTime;
            app.PlaybackClock = tic;
            app.State = "Playing";
            app.PlayButton.Text = "暂停";
            start(app.PlaybackTimer);
            app.updateStatus();
        end

        function pausePlayback(app)
            if app.State == "Playing"
                time = app.AnchorTime + toc(app.PlaybackClock) * app.PlaybackSpeed;
                stop(app.PlaybackTimer);
                app.State = "Paused";
                app.updateFrame(time);
            elseif ~isempty(app.PlaybackTimer) && isvalid(app.PlaybackTimer)
                stop(app.PlaybackTimer);
            end
            if ~isempty(app.PlayButton) && isvalid(app.PlayButton)
                app.PlayButton.Text = "播放";
            end
        end

        function setPlaybackSpeed(app, speed)
            arguments
                app
                speed (1, 1) double {mustBeMember(speed, [0.25, 0.5, 1, 2, 4])}
            end
            app.assertInteractive();
            wasPlaying = app.State == "Playing";
            app.pausePlayback();
            app.PlaybackSpeed = speed;
            app.SpeedDropDown.Value = speed;
            if wasPlaying && app.CurrentTime < app.Data.Time(end), app.play(); end
        end

        function setView(app, value)
            arguments
                app
                value (1, 1) string {mustBeMember(value, ["north", "heading", "overview"])}
            end
            app.assertInteractive();
            app.ViewDropDown.Value = char(value);
            app.updateFrame(app.CurrentTime);
            drawnow limitrate
        end

        function report = exportVideo(app, filePath, options)
            arguments
                app
                filePath (1, 1) string
                options.FrameRate (1, 1) double {mustBePositive} = 30
                options.Resolution (1, 2) double = [1920, 1080]
                options.TimeRange (1, 2) double = [NaN, NaN]
                options.Speed (1, 1) double {mustBePositive} = 1
                options.Overwrite (1, 1) logical = false
                options.ShowProgress (1, 1) logical = true
                options.Renderer (1, 1) string {mustBeMember(options.Renderer, ["graphics", "ui"])} = "graphics"
                options.CancelFcn (1, 1) function_handle = @() false
                options.ProgressFcn (1, 1) function_handle = @(~) []
            end
            app.assertInteractive();
            wasPlaying = app.State == "Playing";
            app.pausePlayback();
            previousTime = app.CurrentTime;
            app.State = "Exporting";
            app.ExportCancelRequested = false;
            cleanup = onCleanup(@() app.restoreAfterExport(previousTime, wasPlaying));
            app.enableControls(false);
            if options.ShowProgress
                app.ProgressDialog = uiprogressdlg(app.UIFigure, ...
                    Title = "导出 MP4", Message = "准备视频帧…", Cancelable = "on");
            end
            report = exportRaceReplayVideo(app.Data, filePath, ...
                FrameRate = options.FrameRate, Resolution = options.Resolution, ...
                TimeRange = options.TimeRange, Speed = options.Speed, ...
                View = string(app.ViewDropDown.Value), FollowSpan = app.SpanField.Value, ...
                ShowValues = app.ValuesCheckBox.Value, ShowStats = app.StatsCheckBox.Value, ...
                Renderer = options.Renderer, ...
                Overwrite = options.Overwrite, ...
                CancelFcn = @() app.isExportCancelled(options.CancelFcn), ...
                ProgressFcn = @(progress) app.onExportProgress(progress, options.ProgressFcn));
            clear cleanup
        end
    end

    methods (Access = private)
        function createComponents(app)
            if app.OwnsFigure
                app.UIFigure = uifigure(Name = "FSAE 赛事回放", Visible = "off", ...
                    Position = [80, 60, app.WindowSize], ...
                    CloseRequestFcn = @(~, ~) app.requestClose());
                app.Parent = app.UIFigure;
            else
                app.UIFigure = ancestor(app.Parent, "figure");
            end
            app.RootGrid = uigridlayout(app.Parent, [4, 1], ...
                RowHeight = {42, '1x', 38, 38}, Padding = [8, 8, 8, 8], RowSpacing = 5);
            app.ControlGrid = uigridlayout(app.RootGrid, [1, 15], ...
                ColumnWidth = {65, 55, 62, 62, 38, 70, 115, 64, 67, 65, 72, 83, 100, 100, '1x'}, ...
                Padding = [0, 0, 0, 0], ColumnSpacing = 5);
            app.ControlGrid.Layout.Row = 1;
            app.PlayButton = uibutton(app.ControlGrid, Text = "播放", ...
                ButtonPushedFcn = @(~, ~) app.togglePlayback());
            uibutton(app.ControlGrid, Text = "归零", ...
                ButtonPushedFcn = @(~, ~) app.seek(app.Data.Time(1)));
            uibutton(app.ControlGrid, Text = "上一帧", ...
                ButtonPushedFcn = @(~, ~) app.step(-1));
            uibutton(app.ControlGrid, Text = "下一帧", ...
                ButtonPushedFcn = @(~, ~) app.step(1));
            uilabel(app.ControlGrid, Text = "倍速");
            app.SpeedDropDown = uidropdown(app.ControlGrid, ...
                Items = {'0.25×', '0.5×', '1×', '2×', '4×'}, ...
                ItemsData = [0.25, 0.5, 1, 2, 4], Value = 1, ...
                ValueChangedFcn = @(src, ~) app.setPlaybackSpeed(src.Value));
            app.ViewDropDown = uidropdown(app.ControlGrid, ...
                Items = {'跟车 · 北向', '跟车 · 车头朝上', '全景'}, ...
                ItemsData = {'north', 'heading', 'overview'}, Value = 'north', ...
                ValueChangedFcn = @(src, ~) app.setView(string(src.Value)));
            uilabel(app.ControlGrid, Text = "视野 (m)");
            app.SpanField = uieditfield(app.ControlGrid, "numeric", ...
                Limits = [6, 200], Value = 18, ...
                ValueChangedFcn = @(~, ~) app.updateFrame(app.CurrentTime));
            app.ValuesCheckBox = uicheckbox(app.ControlGrid, Text = "轮旁值", Value = true, ...
                ValueChangedFcn = @(~, ~) app.updateFrame(app.CurrentTime));
            app.StatsCheckBox = uicheckbox(app.ControlGrid, Text = "数据表", Value = true, ...
                ValueChangedFcn = @(~, ~) app.updateFrame(app.CurrentTime));
            uibutton(app.ControlGrid, Text = "加载赛道", ...
                ButtonPushedFcn = @(~, ~) app.browseTrack());
            uibutton(app.ControlGrid, Text = "加载参数快照", ...
                ButtonPushedFcn = @(~, ~) app.browseSnapshot());
            app.ExportButton = uibutton(app.ControlGrid, Text = "导出 MP4", ...
                ButtonPushedFcn = @(~, ~) app.openExportDialog());
            app.ContentHost = uigridlayout(app.RootGrid, [1, 1], Padding = [0, 0, 0, 0]);
            app.ContentHost.Layout.Row = 2;
            app.View = createRaceReplayView(app.ContentHost, app.Data);
            app.TimelineGrid = uigridlayout(app.RootGrid, [1, 2], ...
                ColumnWidth = {'1x', 180}, Padding = [8, 2, 8, 0]);
            app.TimelineGrid.Layout.Row = 3;
            upper = max(app.Data.Time(end), app.Data.Time(1) + 1e-6);
            app.TimeSlider = uislider(app.TimelineGrid, ...
                Limits = [app.Data.Time(1), upper], Value = app.Data.Time(1), ...
                MajorTicks = [], MinorTicks = [], ...
                ValueChangedFcn = @(~, event) app.seek(event.Value));
            app.TimeLabel = uilabel(app.TimelineGrid, HorizontalAlignment = "right");
            app.StatusLabel = uilabel(app.RootGrid, FontSize = 10, WordWrap = "on");
            app.StatusLabel.Layout.Row = 4;
            setappdata(app.Parent, "FSAERaceReplayApp", app);
        end

        function updateFrame(app, time)
            if app.Rendering || app.State == "Closing", return; end
            app.Rendering = true;
            cleanup = onCleanup(@() app.finishRendering());
            app.LastFrame = sampleRaceReplayFrame(app.Data, time);
            app.CurrentTime = app.LastFrame.Time;
            renderRaceReplayFrame(app.View, app.Data, app.LastFrame, ...
                View = string(app.ViewDropDown.Value), FollowSpan = app.SpanField.Value, ...
                ShowValues = app.ValuesCheckBox.Value, ShowStats = app.StatsCheckBox.Value);
            app.TimeSlider.Value = app.CurrentTime;
            app.TimeLabel.Text = sprintf('%.2f / %.2f s', app.CurrentTime, app.Data.Time(end));
            app.updateStatus();
            clear cleanup
        end

        function finishRendering(app)
            if isvalid(app), app.Rendering = false; end
        end

        function updateStatus(app)
            label = "已暂停";
            if app.State == "Playing", label = "播放中"; end
            if app.State == "Exporting", label = "导出中"; end
            app.StatusLabel.Text = char(label + " | " + app.Data.TrackSource + ...
                " | " + app.Data.Geometry.Source + " | " + app.LastFrame.Pedals.Source);
        end

        function onTick(app)
            if app.State ~= "Playing" || app.Rendering, return; end
            time = app.AnchorTime + toc(app.PlaybackClock) * app.PlaybackSpeed;
            app.updateFrame(time);
            if time >= app.Data.Time(end)
                stop(app.PlaybackTimer);
                app.State = "Paused";
                app.PlayButton.Text = "播放";
                app.updateStatus();
            end
            drawnow limitrate
        end

        function togglePlayback(app)
            if app.State == "Playing", app.pausePlayback(); else, app.play(); end
        end

        function assertInteractive(app)
            assert(~any(app.State == ["Exporting", "Closing"]), ...
                "FSAE:Replay:Busy", "Replay is exporting or closing.");
        end

        function replaceData(app, prepared)
            app.pausePlayback();
            app.Data = prepared;
            delete(app.View.Grid);
            app.View = createRaceReplayView(app.ContentHost, app.Data);
            app.TimeSlider.Limits = [app.Data.Time(1), ...
                max(app.Data.Time(end), app.Data.Time(1) + 1e-6)];
            app.State = "Paused";
            app.updateFrame(app.Data.Time(1));
            app.initializeSceneLayout();
        end

        function initializeSceneLayout(app)
            % Flush the complete controller and its ancestor grids once;
            % hidden containers do not invoke SizeChangedFcn themselves.
            drawnow;
            pause(0.05);
            drawnow;
            panel = app.View.ScenePanel;
            panel.SizeChangedFcn(panel, []);
        end

        function browseTrack(app)
            app.pausePlayback();
            [file, folder] = uigetfile('*.mat', '选择与此结果对应的赛道');
            if isequal(file, 0), return; end
            try
                path = string(fullfile(folder, file));
                prepared = prepareRaceReplayData(app.Source, TrackFile = path, ...
                    Snapshot = app.SnapshotOverride, ProjectRoot = app.ProjectRoot);
                app.TrackFile = path;
                app.TrackOverride = struct;
                app.replaceData(prepared);
            catch exception
                uialert(app.UIFigure, exception.message, "赛道加载失败");
            end
        end

        function browseSnapshot(app)
            app.pausePlayback();
            [file, folder] = uigetfile('*.mat', '选择历史运行参数快照或带快照的结果');
            if isequal(file, 0), return; end
            try
                loaded = load(fullfile(folder, file));
                snapshot = raceReplayValue(loaded, "snapshot", ...
                    raceReplayValue(loaded, "result.Meta.Replay", struct));
                assert(~isempty(fieldnames(snapshot)), "FSAE:Replay:Snapshot", ...
                    "MAT 文件需包含 snapshot 或 result.Meta.Replay。");
                prepared = prepareRaceReplayData(app.Source, TrackFile = app.TrackFile, ...
                    Track = app.TrackOverride, Snapshot = snapshot, ProjectRoot = app.ProjectRoot);
                app.SnapshotOverride = snapshot;
                app.replaceData(prepared);
            catch exception
                uialert(app.UIFigure, exception.message, "快照加载失败");
            end
        end

        function enableControls(app, enabled)
            value = "off";
            if enabled, value = "on"; end
            for type = ["uibutton", "uidropdown", "uieditfield", "uicheckbox", "uislider"]
                controls = findall(app.RootGrid, Type = type);
                for control = reshape(controls, 1, [])
                    control.Enable = value;
                end
            end
        end

        function cancelled = isExportCancelled(app, external)
            cancelled = ~isvalid(app);
            if cancelled, return; end
            cancelled = app.ExportCancelRequested || external();
            if ~isempty(app.ProgressDialog) && isvalid(app.ProgressDialog)
                cancelled = cancelled || app.ProgressDialog.CancelRequested;
            end
        end

        function onExportProgress(app, progress, external)
            if ~isvalid(app), return; end
            if ~isempty(app.ProgressDialog) && isvalid(app.ProgressDialog)
                app.ProgressDialog.Value = progress.Fraction;
                app.ProgressDialog.Message = sprintf('帧 %d / %d · t = %.2f s', ...
                    progress.FrameCount, progress.TotalFrames, progress.Time);
            end
            external(progress);
        end

        function restoreAfterExport(app, time, wasPlaying)
            if ~isvalid(app), return; end
            if ~isempty(app.ProgressDialog) && isvalid(app.ProgressDialog)
                delete(app.ProgressDialog);
            end
            app.ProgressDialog = [];
            if app.PendingClose
                delete(app);
                return
            end
            app.State = "Paused";
            app.enableControls(true);
            app.updateFrame(time);
            if wasPlaying, app.play(); end
        end

        function requestClose(app)
            if app.State == "Exporting"
                app.ExportCancelRequested = true;
                app.PendingClose = true;
            else
                delete(app);
            end
        end

        function openExportDialog(app)
            app.pausePlayback();
            app.ExportDialog = uifigure(Name = "MP4 导出设置", ...
                Position = [250, 180, 470, 340], WindowStyle = "modal");
            gridHandle = uigridlayout(app.ExportDialog, [6, 2], ...
                RowHeight = {36, 36, 36, 36, '1x', 36}, ColumnWidth = {150, '1x'});
            uilabel(gridHandle, Text = "起始仿真时间 (s)");
            timeLimits = [app.Data.Time(1), max(app.Data.Time(end), app.Data.Time(1) + 1e-6)];
            startField = uieditfield(gridHandle, "numeric", ...
                Value = app.Data.Time(1), Limits = timeLimits);
            uilabel(gridHandle, Text = "结束仿真时间 (s)");
            endField = uieditfield(gridHandle, "numeric", ...
                Value = app.Data.Time(end), Limits = timeLimits);
            uilabel(gridHandle, Text = "帧率 (fps)");
            rateField = uieditfield(gridHandle, "numeric", Value = 30, ...
                Limits = [1, 60], RoundFractionalValues = "on");
            uilabel(gridHandle, Text = "输出分辨率");
            resolution = uidropdown(gridHandle, ...
                Items = {'1920×1080', '1280×720'}, Value = '1920×1080');
            hint = uilabel(gridHandle, Text = sprintf( ...
                '使用当前视角及 %.2g× 倍速。导出期间暂停回放，可取消。', app.PlaybackSpeed), ...
                WordWrap = "on");
            hint.Layout.Column = [1, 2];
            uibutton(gridHandle, Text = "取消", ...
                ButtonPushedFcn = @(~, ~) delete(app.ExportDialog));
            uibutton(gridHandle, Text = "选择文件并导出", ...
                ButtonPushedFcn = @(~, ~) exportSelected());

            function exportSelected()
                range = [startField.Value, endField.Value];
                rate = rateField.Value;
                outputResolution = [1920, 1080];
                if strcmp(resolution.Value, '1280×720'), outputResolution = [1280, 720]; end
                if range(2) < range(1)
                    uialert(app.ExportDialog, "结束时间不能早于起始时间。", "时间范围无效");
                    return
                end
                [file, folder] = uiputfile('*.mp4', '保存赛事回放视频', 'race_replay.mp4');
                if isequal(file, 0), return; end
                delete(app.ExportDialog);
                try
                    report = app.exportVideo(fullfile(folder, file), FrameRate = rate, ...
                        Resolution = outputResolution, TimeRange = range, Speed = app.PlaybackSpeed, ...
                        Overwrite = true);
                    if isvalid(app) && ~report.Cancelled
                        app.StatusLabel.Text = char("视频已保存：" + report.File);
                    end
                catch exception
                    if isvalid(app), uialert(app.UIFigure, exception.message, "视频导出失败"); end
                end
            end
        end
    end
end
