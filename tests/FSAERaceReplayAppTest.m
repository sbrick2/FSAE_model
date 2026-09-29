classdef FSAERaceReplayAppTest < matlab.uitest.TestCase
    properties (Access = private)
        App
        Folder
        CancelRequested = false
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "apps")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "scripts", "replay")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "scripts", "reporting")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, "tests", "helpers")));
        end
    end
    methods (TestMethodSetup)
        function launchApp(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            testCase.Folder = string(fixture.Folder);
            testCase.App = FSAERaceReplayApp(createRaceReplayTestResult(), Visible = "off");
            testCase.CancelRequested = false;
            testCase.addTeardown(@() FSAERaceReplayAppTest.deleteIfValid(testCase.App));
            drawnow;
        end
    end
    methods (Test)
        function createsEightForceComponentsAndTwelveMotorBars(testCase)
            testCase.verifyNumElements(testCase.App.View.FxArrows.UData, 4);
            testCase.verifyNumElements(testCase.App.View.FyArrows.UData, 4);
            testCase.verifySize(testCase.App.View.MotorBars, [4, 3]);
            testCase.verifyEqual(testCase.App.View.MotorBars(1, 1).YData(3), 100 * 60 / (2 * pi), AbsTol = 1e-10);
            testCase.verifyEqual(testCase.App.View.MotorBars(4, 2).YData(3), -20);
            testCase.verifyEqual(testCase.App.View.MotorBars(4, 3).YData(3), -8);
            testCase.verifyEqual(testCase.App.View.MotorAxes(2, 2).YLim, testCase.App.View.MotorAxes(4, 2).YLim);
        end
        function seekAndStepsUseSampleClock(testCase)
            testCase.App.seek(0.5);
            testCase.verifyEqual(testCase.App.CurrentTime, 0.5);
            testCase.verifyEqual(testCase.App.LastFrame.Vehicle.X, 2.5);
            testCase.App.step(1);
            testCase.verifyEqual(testCase.App.CurrentTime, 1);
            testCase.App.step(-1);
            testCase.verifyEqual(testCase.App.CurrentTime, 0);
        end
        function viewCallbackKeepsClockAndForceDirections(testCase)
            testCase.App.seek(0.5);
            testCase.App.UIFigure.Visible = "on";
            drawnow;
            testCase.choose(testCase.App.ViewDropDown, "跟车 · 车头朝上");
            testCase.verifyEqual(testCase.App.CurrentTime, 0.5);
            testCase.verifyEqual(testCase.App.View.FxArrows.VData, ...
                testCase.App.LastFrame.Tire.Fx * testCase.App.Data.ForceScale, AbsTol = 1e-10);
            testCase.choose(testCase.App.ViewDropDown, "全景");
            bounds = testCase.App.View.MapBounds;
            testCase.verifyLessThanOrEqual(testCase.App.View.MainAxes.XLim(1), bounds(1, 1) + 1e-10);
            testCase.verifyGreaterThanOrEqual(testCase.App.View.MainAxes.XLim(2), bounds(2, 1) - 1e-10);
            testCase.verifyLessThanOrEqual(testCase.App.View.MainAxes.YLim(1), bounds(1, 2) + 1e-10);
            testCase.verifyGreaterThanOrEqual(testCase.App.View.MainAxes.YLim(2), bounds(2, 2) - 1e-10);
        end
        function playbackSpeedAndPauseRemainConsistent(testCase)
            testCase.App.setPlaybackSpeed(2);
            testCase.App.play();
            pause(0.15);
            testCase.App.pausePlayback();
            testCase.verifyEqual(testCase.App.State, "Paused");
            testCase.verifyGreaterThanOrEqual(testCase.App.CurrentTime, 0.25);
            testCase.verifyLessThanOrEqual(testCase.App.CurrentTime, 3);
            testCase.verifyEqual(string(testCase.App.PlaybackTimer.Running), "off");
            testCase.App.setPlaybackSpeed(0.5);
            testCase.verifyEqual(testCase.App.SpeedDropDown.Value, 0.5);
        end
        function playbackStopsAtEndAndRestarts(testCase)
            testCase.App.seek(2.99);
            testCase.App.play();
            FSAERaceReplayAppTest.waitUntilStopped(testCase.App);
            testCase.verifyEqual(testCase.App.CurrentTime, 3);
            testCase.verifyEqual(testCase.App.State, "Paused");
            testCase.App.play();
            testCase.verifyLessThan(testCase.App.CurrentTime, 3);
            testCase.App.pausePlayback();
        end
        function newResultRebuildsRanges(testCase)
            testCase.App.seek(3);
            result = createRaceReplayTestResult();
            result.Time = result.Time + 10;
            testCase.App.loadResult(result);
            testCase.verifyEqual(testCase.App.CurrentTime, 10);
            testCase.verifyEqual(testCase.App.TimeSlider.Limits, [10, 13]);
            testCase.verifyEqual(testCase.App.State, "Paused");
        end
        function closeDeletesPlaybackTimer(testCase)
            replayTimer = testCase.App.PlaybackTimer;
            figureHandle = testCase.App.UIFigure;
            testCase.App.play();
            close(figureHandle);
            testCase.verifyFalse(isvalid(replayTimer));
            testCase.verifyFalse(isgraphics(figureHandle));
        end
        function exportHasFixedFramesDimensionsAndRestoresTime(testCase)
            testCase.App.seek(1);
            report = testCase.App.exportVideo(fullfile(testCase.Folder, "normal.mp4"), ...
                Resolution = [1280, 720], TimeRange = [0, 1], ...
                FrameRate = 4, Speed = 2, ShowProgress = false, ...
                ProgressFcn = @(p) testCase.verifyExportState(p));
            video = VideoReader(report.File);
            testCase.verifyEqual(report.FrameCount, 2);
            testCase.verifyEqual([video.Width, video.Height, video.FrameRate], [1280, 720, 4]);
            testCase.verifyEqual(video.Duration, 0.5, AbsTol = 1e-6);
            testCase.verifyEqual(report.Renderer, "graphics");
            testCase.verifyGreaterThan(report.ElapsedSeconds, 0);
            firstImage = readFrame(video);
            lastImage = readFrame(video);
            testCase.verifyFalse(isequal(firstImage, lastImage), ...
                "Successive output frames must capture the changed simulation state.");
            testCase.verifyEqual(testCase.App.CurrentTime, 1);
            testCase.verifyEqual(testCase.App.State, "Paused");
            testCase.verifyEqual(string(testCase.App.PlayButton.Enable), "on");
        end
        function graphicsRendererMatchesUIValuesAcrossViews(testCase)
            result = createRaceReplayTestResult();
            result.Vehicle.Psi = [0; 0.4; 0.8];
            result.Vehicle.Ax = [0; 1; -2] * 9.80665;
            result.Vehicle.Ay = [0; -1; 2] * 9.80665;
            result.Wheel.SteerAngle = repmat([0.2, 0.2, 0, 0], 3, 1);
            testCase.App.loadResult(result);
            figureHandle = figure(Visible = "off", WindowStyle = "normal", ...
                Position = [30, 30, 1280, 720], MenuBar = "none", ToolBar = "none");
            testCase.addTeardown(@() delete(figureHandle));
            graphicsView = createRaceReplayView(figureHandle, testCase.App.Data, Renderer = "graphics");
            for camera = ["north", "heading", "overview"]
                testCase.App.setView(camera);
                for time = [0.5, 2]
                    testCase.App.seek(time);
                    renderRaceReplayFrame(graphicsView, testCase.App.Data, testCase.App.LastFrame, ...
                        View = camera, FollowSpan = testCase.App.SpanField.Value);
                    ui = testCase.App.View;
                    testCase.verifyEqual(graphicsView.FxArrows.UData, ui.FxArrows.UData);
                    testCase.verifyEqual(graphicsView.FxArrows.VData, ui.FxArrows.VData);
                    testCase.verifyEqual(graphicsView.FyArrows.UData, ui.FyArrows.UData);
                    testCase.verifyEqual(graphicsView.FyArrows.VData, ui.FyArrows.VData);
                    testCase.verifyEqual(graphicsView.Body.XData, ui.Body.XData);
                    testCase.verifyEqual(graphicsView.Body.YData, ui.Body.YData);
                    testCase.verifyEqual(string({graphicsView.StatsValues.String}), string({ui.StatsValues.String}));
                    testCase.verifyEqual(string({graphicsView.StatsUnits.String}), string({ui.StatsUnits.String}));
                    testCase.verifyEqual(graphicsView.GGPoint.XData, ui.GGPoint.XData);
                    testCase.verifyEqual(graphicsView.GGPoint.YData, ui.GGPoint.YData);
                    testCase.verifyEqual(graphicsView.GGTrail.XData, ui.GGTrail.XData);
                    testCase.verifyEqual(graphicsView.GGTrail.YData, ui.GGTrail.YData);
                    testCase.verifyEqual(graphicsView.MainAxes.DataAspectRatio, ui.MainAxes.DataAspectRatio);
                    mainBox = graphicsView.MainAxes.InnerPosition;
                    for inset = [graphicsView.GGAxes, graphicsView.StatsAxes, ...
                            graphicsView.MapAxes, graphicsView.PedalAxes]
                        bounds = inset.InnerPosition;
                        testCase.verifyGreaterThanOrEqual(bounds(1:2), mainBox(1:2));
                        testCase.verifyLessThanOrEqual(bounds(1:2) + bounds(3:4), mainBox(1:2) + mainBox(3:4));
                    end
                    testCase.verifyEqual(graphicsView.PedalBars.YData, ui.PedalBars.YData);
                    for wheel = 1:4
                        testCase.verifyEqual(graphicsView.FrictionPoint(wheel).XData, ui.FrictionPoint(wheel).XData);
                        testCase.verifyEqual(graphicsView.FrictionPoint(wheel).YData, ui.FrictionPoint(wheel).YData);
                        testCase.verifyEqual(graphicsView.FrictionText(wheel).String, ui.FrictionText(wheel).String);
                        testCase.verifyEqual(graphicsView.RoadCircle(wheel).XData, ui.RoadCircle(wheel).XData);
                        for quantity = 1:3
                            testCase.verifyEqual(graphicsView.MotorBars(wheel, quantity).YData, ui.MotorBars(wheel, quantity).YData);
                            testCase.verifyEqual(graphicsView.MotorText(wheel, quantity).String, ui.MotorText(wheel, quantity).String);
                        end
                    end
                end
            end
        end
        function uiRendererRetainsTimingAndDimensions(testCase)
            report = testCase.App.exportVideo(fullfile(testCase.Folder, "ui-renderer.mp4"), ...
                Resolution = [1280, 720], TimeRange = [0, 1], FrameRate = 1, ...
                Renderer = "ui", ShowProgress = false);
            video = VideoReader(report.File);
            testCase.verifyEqual(report.Renderer, "ui");
            testCase.verifyEqual(report.FrameCount, 1);
            testCase.verifyEqual([video.Width, video.Height, video.FrameRate], [1280, 720, 1]);
            testCase.verifyEqual(video.Duration, 1, AbsTol = 1e-6);
        end
        function cancelledExportLeavesExistingDestinationUntouched(testCase)
            file = fullfile(testCase.Folder, "cancelled.mp4");
            FSAERaceReplayAppTest.writeSentinel(file);
            report = testCase.App.exportVideo(file, Resolution = [1280, 720], ...
                TimeRange = [0, 1], FrameRate = 4, Overwrite = true, ShowProgress = false, ...
                ProgressFcn = @(~) testCase.requestCancel(), ...
                CancelFcn = @() testCase.CancelRequested);
            testCase.verifyTrue(report.Cancelled);
            testCase.verifyEqual(report.FrameCount, 1);
            testCase.verifyEqual(fileread(file), 'existing destination');
            testCase.verifyNumElements(dir(fullfile(testCase.Folder, "*.mp4")), 1);
            testCase.verifyEqual(testCase.App.State, "Paused");
        end
        function encodingCallbackFailureCleansAndRestores(testCase)
            testCase.App.seek(1);
            file = fullfile(testCase.Folder, "failure.mp4");
            testCase.verifyError(@() testCase.App.exportVideo(file, ...
                Resolution = [1280, 720], TimeRange = [0, 1], FrameRate = 4, ...
                ShowProgress = false, ProgressFcn = @FSAERaceReplayAppTest.failExport), ...
                "FSAE:Replay:InjectedFailure");
            testCase.verifyFalse(isfile(file));
            testCase.verifyEmpty(dir(fullfile(testCase.Folder, "*.mp4")));
            testCase.verifyEqual(testCase.App.CurrentTime, 1);
            testCase.verifyEqual(testCase.App.State, "Paused");
            testCase.verifyEqual(string(testCase.App.PlayButton.Enable), "on");
        end
        function invalidExportRestoresPlayingState(testCase)
            testCase.App.play();
            testCase.verifyError(@() testCase.App.exportVideo( ...
                fullfile(testCase.Folder, "invalid.mp4"), TimeRange = [-1, 1], ...
                ShowProgress = false), "FSAE:Replay:VideoRange");
            testCase.verifyEqual(testCase.App.State, "Playing");
            testCase.verifyEqual(string(testCase.App.PlaybackTimer.Running), "on");
            testCase.App.pausePlayback();
        end
        function closeDuringExportCancelsAndReleasesTimer(testCase)
            replayTimer = testCase.App.PlaybackTimer;
            file = fullfile(testCase.Folder, "closed.mp4");
            report = testCase.App.exportVideo(file, Resolution = [1280, 720], ...
                TimeRange = [0, 1], FrameRate = 4, ShowProgress = false, ...
                ProgressFcn = @(~) close(testCase.App.UIFigure));
            testCase.verifyTrue(report.Cancelled);
            testCase.verifyFalse(isvalid(replayTimer));
            testCase.verifyFalse(isfile(file));
        end
        function interpolatedPowerPeakFitsFixedBarRange(testCase)
            result = createRaceReplayTestResult();
            result.Powertrain.MotorTorqueActual = repmat([0; 100; 0], 1, 4);
            result.Powertrain.MotorSpeed = repmat([100; 0; 100], 1, 4);
            result.Powertrain.MotorRPM = result.Powertrain.MotorSpeed * 60 / (2 * pi);
            result.Powertrain.MotorMechanicalPower = zeros(3, 4);
            testCase.App.loadResult(result);
            testCase.App.seek(0.5);
            testCase.verifyEqual(testCase.App.View.MotorBars(1, 3).YData(3), 2.5);
            testCase.verifyGreaterThan(testCase.App.View.MotorAxes(1, 3).YLim(2), 2.5);
        end
        function cancellationAtLastFrameDoesNotPublish(testCase)
            file = fullfile(testCase.Folder, "last-frame.mp4");
            report = testCase.App.exportVideo(file, Resolution = [1280, 720], ...
                TimeRange = [0, 1], FrameRate = 1, ShowProgress = false, ...
                ProgressFcn = @(~) testCase.requestCancel(), ...
                CancelFcn = @() testCase.CancelRequested);
            testCase.verifyTrue(report.Cancelled);
            testCase.verifyEqual(report.FrameCount, 1);
            testCase.verifyFalse(isfile(file));
            testCase.verifyEmpty(dir(fullfile(testCase.Folder, "*.mp4")));
        end
        function finiteRoadBoundaryFitsFrictionPlot(testCase)
            result = createRaceReplayTestResult();
            result.Meta.Replay.Tire.RoadMuLimit = 2;
            testCase.App.loadResult(result);
            testCase.verifyGreaterThanOrEqual(testCase.App.View.FrictionAxes(1).XLim(2), ...
                max(testCase.App.View.RoadCircle(1).XData));
            testCase.verifyGreaterThanOrEqual(testCase.App.View.FrictionAxes(1).YLim(2), ...
                max(testCase.App.View.RoadCircle(1).YData));
        end
        function frictionUsesPhysicalForceAndAnisotropicBoundary(testCase)
            frame = testCase.App.LastFrame;
            frame.Tire.Capacity.X(1) = 2000;
            frame.Tire.Capacity.Y(1) = 1000;
            frame.Tire.Capacity.RoadLimit(1) = 3000;
            renderRaceReplayFrame(testCase.App.View, testCase.App.Data, frame);
            view = testCase.App.View;

            testCase.verifyEqual(view.FrictionPoint(1).XData, 0.4, AbsTol = 1e-12);
            testCase.verifyEqual(view.FrictionPoint(1).YData, 0.3, AbsTol = 1e-12);
            testCase.verifyEqual(max(view.FrictionCircle(1).XData), 1, AbsTol = 1e-12);
            testCase.verifyEqual(max(view.FrictionCircle(1).YData), 2, AbsTol = 1e-12);
            testCase.verifyEqual(max(view.RoadCircle(1).XData), 3, AbsTol = 1e-12);
            testCase.verifyEqual(max(view.RoadCircle(1).YData), 3, AbsTol = 1e-12);
            testCase.verifyEqual(string(view.FrictionAxes(1).XLabel.String), "Fy (kN)");
            testCase.verifyEqual(string(view.FrictionAxes(1).YLabel.String), "Fx (kN)");
            for wheel = 2:4
                testCase.verifyEqual(view.FrictionAxes(wheel).XLim, view.FrictionAxes(1).XLim);
                testCase.verifyEqual(view.FrictionAxes(wheel).YLim, view.FrictionAxes(1).YLim);
            end
        end
        function liveGGUsesPlaybackClockAndSupportsMissingAcceleration(testCase)
            result = createRaceReplayTestResult();
            result.Vehicle.Ax = [0; 1; -2] * 9.80665;
            result.Vehicle.Ay = [0; -1; 2] * 9.80665;
            testCase.App.loadResult(result);
            testCase.App.seek(2);
            view = testCase.App.View;
            testCase.verifyEqual(view.GGPoint.XData, 0.5, AbsTol = 1e-12);
            testCase.verifyEqual(view.GGPoint.YData, -0.5, AbsTol = 1e-12);
            testCase.verifyEqual(view.GGTrail.XData, [0, -1, 0.5], AbsTol = 1e-12);
            testCase.verifyEqual(view.GGTrail.YData, [0, 1, -0.5], AbsTol = 1e-12);
            testCase.verifyEqual(string(view.StatsValues(6).String), "+0.50");
            testCase.verifyEqual(string(view.StatsValues(7).String), "-0.50");
            testCase.App.seek(0.5);
            testCase.verifyEqual(view.GGTrail.XData, [0, -0.5], AbsTol = 1e-12);
            testCase.verifyEqual(view.GGTrail.YData, [0, 0.5], AbsTol = 1e-12);
            frame = testCase.App.LastFrame;
            frame.Vehicle.Ax = NaN;
            frame.Vehicle.Ay = NaN;
            renderRaceReplayFrame(view, testCase.App.Data, frame, ShowStats = false);
            testCase.verifyTrue(isnan(view.GGPoint.XData));
            testCase.verifyTrue(isnan(view.GGPoint.YData));
            testCase.verifyEqual(string(view.StatsValues(6).String), "不可用");
            testCase.verifyEqual(string(view.StatsAxes.Visible), "off");
            testCase.verifyEqual(string(view.GGAxes.Visible), "on");
            testCase.verifyTrue(all(string({view.StatsAxes.Children.Visible}) == "off"));
            testCase.App.seek(0.5);
            testCase.verifyEqual(string(view.StatsAxes.Visible), "on");
        end
        function cornerInsetsStayInsideSceneAfterResize(testCase)
            result = createRaceReplayTestResult();
            angle = linspace(0, 2 * pi, 65).';
            result.Meta.Replay.Track = struct("X", 10 * cos(angle), ...
                "Y", 10 * sin(angle), "Heading", angle + pi / 2, ...
                "LeftHalfWidth", 1, "RightHalfWidth", 1, "IsClosed", true);
            testCase.App.loadResult(result);
            panel = testCase.App.View.ScenePanel;
            testCase.verifyEqual(testCase.App.View.MainAxes.InnerPosition(3), ...
                panel.Position(3) - 8, AbsTol = 1e-10);
            testCase.verifyEqual(testCase.App.View.MapAxes.Parent, panel);
            testCase.verifyEqual(testCase.App.View.PedalAxes.Parent, panel);
            testCase.App.UIFigure.Visible = "on";
            testCase.App.setView("overview");
            for dimensions = [1366, 768; 1920, 1080].'
                testCase.App.UIFigure.Position(3:4) = dimensions.';
                drawnow;
                pause(0.2);
                drawnow;
                testCase.verifyEqual(testCase.App.CurrentTime, 0);
                for inset = [testCase.App.View.GGAxes, testCase.App.View.StatsAxes, ...
                        testCase.App.View.MapAxes, testCase.App.View.PedalAxes]
                    bounds = inset.InnerPosition;
                    scene = testCase.App.View.MainAxes.InnerPosition;
                    testCase.verifyGreaterThanOrEqual(bounds(1:2), scene(1:2));
                    testCase.verifyLessThanOrEqual(bounds(1:2) + bounds(3:4), scene(1:2) + scene(3:4));
                end
                map = testCase.App.View.MapAxes.InnerPosition;
                pedal = testCase.App.View.PedalAxes.InnerPosition;
                testCase.verifyEqual(map(2), pedal(2));
                testCase.verifyLessThan(map(1) + map(3), pedal(1));
                testCase.verifyLessThan(map(2) + map(4), testCase.App.View.GGAxes.InnerPosition(2));
                testCase.verifyLessThan(pedal(2) + pedal(4), testCase.App.View.StatsAxes.InnerPosition(2));
                testCase.verifyEqual(testCase.App.View.GGAxes.DataAspectRatio, [1, 1, 1]);
                for ax = [testCase.App.View.MainAxes, testCase.App.View.MapAxes]
                    plotBox = tightPosition(ax, Units = "pixels");
                    testCase.verifyEqual(plotBox(3) / diff(ax.XLim), ...
                        plotBox(4) / diff(ax.YLim), RelTol = 0.03);
                    bounds = testCase.App.View.MapBounds;
                    testCase.verifyLessThanOrEqual(ax.XLim(1), bounds(1, 1) + 1e-10);
                    testCase.verifyGreaterThanOrEqual(ax.XLim(2), bounds(2, 1) - 1e-10);
                    testCase.verifyLessThanOrEqual(ax.YLim(1), bounds(1, 2) + 1e-10);
                    testCase.verifyGreaterThanOrEqual(ax.YLim(2), bounds(2, 2) - 1e-10);
                end
            end
        end
        function embeddedDeletionKeepsParentWindow(testCase)
            parent = uifigure(Visible = "off", Position = [30, 30, 1400, 850]);
            testCase.addTeardown(@() FSAERaceReplayAppTest.deleteIfValid(parent));
            host = uipanel(parent, Position = [0, 0, 1400, 850]);
            replay = FSAERaceReplayApp(createRaceReplayTestResult(), Parent = host);
            testCase.addTeardown(@() FSAERaceReplayAppTest.deleteIfValid(replay));
            testCase.verifyEqual(replay.UIFigure, parent);
            testCase.verifyEqual(string(parent.Visible), "off");
            replayTimer = replay.PlaybackTimer;
            delete(replay);
            testCase.verifyTrue(isgraphics(parent));
            testCase.verifyFalse(isvalid(replayTimer));
            testCase.verifyEmpty(host.Children);
        end
        function embeddedExportPreservesOtherControls(testCase)
            parent = uifigure(Visible = "off", Position = [30, 30, 1400, 850]);
            testCase.addTeardown(@() FSAERaceReplayAppTest.deleteIfValid(parent));
            other = uibutton(parent, Enable = "off", Position = [0, 0, 100, 30]);
            host = uipanel(parent, Position = [0, 35, 1400, 815]);
            replay = FSAERaceReplayApp(createRaceReplayTestResult(), Parent = host);
            testCase.addTeardown(@() FSAERaceReplayAppTest.deleteIfValid(replay));
            report = replay.exportVideo(fullfile(testCase.Folder, "embedded.mp4"), ...
                Resolution = [1280, 720], TimeRange = [0, 1], FrameRate = 1, ShowProgress = false);
            testCase.verifyFalse(report.Cancelled);
            testCase.verifyEqual(string(other.Enable), "off");
            testCase.verifyEqual(replay.State, "Paused");
        end
        function closingParentDuringExportCancelsAndCleans(testCase)
            parent = uifigure(Visible = "off", Position = [30, 30, 1400, 850]);
            testCase.addTeardown(@() FSAERaceReplayAppTest.deleteIfValid(parent));
            host = uipanel(parent, Position = [0, 0, 1400, 850]);
            replay = FSAERaceReplayApp(createRaceReplayTestResult(), Parent = host);
            testCase.addTeardown(@() FSAERaceReplayAppTest.deleteIfValid(replay));
            replayTimer = replay.PlaybackTimer;
            file = fullfile(testCase.Folder, "parent-close.mp4");
            report = replay.exportVideo(file, Resolution = [1280, 720], ...
                TimeRange = [0, 1], FrameRate = 4, ShowProgress = false, ...
                ProgressFcn = @(~) close(parent));
            testCase.verifyTrue(report.Cancelled);
            testCase.verifyFalse(isfile(file));
            testCase.verifyFalse(isvalid(replayTimer));
            testCase.verifyFalse(isvalid(replay));
        end
    end
    methods (Access = private)
        function requestCancel(testCase)
            testCase.CancelRequested = true;
        end
        function verifyExportState(testCase, progress)
            testCase.verifyEqual(testCase.App.State, "Exporting");
            testCase.verifyEqual(string(testCase.App.PlayButton.Enable), "off");
            testCase.verifyEqual(string(testCase.App.TimeSlider.Enable), "off");
            testCase.verifyError(@() testCase.App.seek(progress.Time), "FSAE:Replay:Busy");
        end
    end
    methods (Static, Access = private)
        function deleteIfValid(app)
            if isvalid(app), delete(app); end
        end
        function waitUntilStopped(app)
            clock = tic;
            while app.State == "Playing" && toc(clock) < 2
                pause(0.02);
                drawnow;
            end
        end
        function writeSentinel(file)
            handle = fopen(file, 'w');
            cleanup = onCleanup(@() fclose(handle));
            fwrite(handle, 'existing destination');
        end
        function failExport(~)
            error("FSAE:Replay:InjectedFailure", "Intentional callback failure.");
        end
    end
end
