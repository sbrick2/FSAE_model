classdef FSAESimulationAppTest < matlab.uitest.TestCase
    %FSAESIMULATIONAPPTEST 图形界面快速回归测试。

    properties (Access = private)
        App
    end

    methods (TestMethodSetup)
        function launchApp(testCase)
            projectRoot = string(fileparts(fileparts(mfilename("fullpath"))));
            addpath(fullfile(projectRoot, "apps"), "-begin");
            addpath(fullfile(projectRoot, "scripts", "initialization"), ...
                "-begin");
            addpath(fullfile(projectRoot, "scripts", "simulation"), "-begin");
            testCase.App = FSAESimulationApp( ...
                ProjectRoot = projectRoot, Visible = "off");
            testCase.addTeardown(@() delete(testCase.App));
            drawnow;
        end
    end

    methods (Test)
        function testThreeMainModules(testCase)
            titles = string({testCase.App.ModuleTabGroup.Children.Title});

            testCase.verifyNumElements(titles, 3);
            testCase.verifyTrue(all(ismember( ...
                ["准静态", "时域闭环", "车辆参数"], titles)));
            testCase.verifyEqual( ...
                testCase.App.ModuleTabGroup.SelectedTab, ...
                testCase.App.QuasiStaticModuleTab);
        end

        function testVehicleParameterModuleLoadsCatalog(testCase)
            catalog = testCase.App.listVehicleParameterCatalog();
            expectedGroups = ["Vehicle", "Tire", "Aero", ...
                "Powertrain", "Battery", "Brake", "Simulation"];
            hasChineseName = ~cellfun("isempty", regexp( ...
                cellstr(catalog.Name), "[\x{4e00}-\x{9fff}]", "once"));

            testCase.verifyGreaterThanOrEqual(height(catalog), 120);
            testCase.verifyTrue(all(ismember( ...
                expectedGroups, unique(catalog.Group))));
            testCase.verifyTrue(all(hasChineseName));
            testCase.verifyFalse(any(startsWith( ...
                catalog.Name, "待翻译参数")));
            testCase.verifyEqual( ...
                height(testCase.App.VehicleParameterTable.Data), ...
                height(catalog));
            testCase.verifyEqual( ...
                testCase.App.VehicleParameterSaveButton.Enable, ...
                matlab.lang.OnOffSwitchState.off);
        end

        function testVehicleParameterGroupFilter(testCase)
            testCase.chooseComponent( testCase.App.ModuleTabGroup, "车辆参数");
            testCase.chooseComponent( ...
                testCase.App.VehicleParameterGroupDropDown, ...
                "整车 | Vehicle");
            drawnow;
            data = testCase.App.VehicleParameterTable.Data;

            testCase.verifyNotEmpty(data);
            testCase.verifyTrue(all(startsWith(data.Path, "Vehicle.")));
        end

        function testVehicleParameterEditTracksPendingChange(testCase)
            testCase.chooseComponent( testCase.App.ModuleTabGroup, "车辆参数");
            data = testCase.App.VehicleParameterTable.Data;
            row = find(data.Path == "Vehicle.Mass", 1);
            originalValue = data.Value(row);

            testCase.typeComponent( testCase.App.VehicleParameterTable, ...
                [row, 4], string(originalValue + 1.0));
            drawnow;
            changes = testCase.App.buildVehicleParameterChanges();

            testCase.verifyEqual(changes.Path, "Vehicle.Mass");
            testCase.verifyEqual(changes.Value, originalValue + 1.0, ...
                AbsTol = 1e-12);
            testCase.verifyEqual( ...
                testCase.App.VehicleParameterSaveButton.Enable, ...
                matlab.lang.OnOffSwitchState.on);
            testCase.verifyTrue(contains(string( ...
                testCase.App.VehicleParameterStatusLabel.Text), ...
                "尚未写入"));

            testCase.pressComponent( testCase.App.VehicleParameterRevertButton);
            testCase.verifyEmpty( ...
                testCase.App.buildVehicleParameterChanges());
        end

        function testVehicleParameterWriterRoundTrip(testCase)
            projectRoot = string(fileparts(fileparts( ...
                mfilename("fullpath"))));
            temporaryFolder = string(tempname);
            mkdir(temporaryFolder);
            testCase.addTeardown(@() rmdir(temporaryFolder, "s"));
            dictionaryPath = fullfile(temporaryFolder, "VehicleData.sldd");
            copyfile(fullfile(projectRoot, "data", "VehicleData.sldd"), ...
                dictionaryPath);
            catalog = listVehicleParameters(projectRoot, ...
                DictionaryPath = dictionaryPath);
            originalValue = catalog.Value( ...
                catalog.Path == "Vehicle.Mass");
            changes = table("Vehicle.Mass", originalValue + 0.5, ...
                'VariableNames', {'Path', 'Value'});

            receipt = updateVehicleParameters(changes, ...
                ProjectRoot = projectRoot, ...
                DictionaryPath = dictionaryPath, ...
                CreateBackup = false);
            updatedCatalog = listVehicleParameters(projectRoot, ...
                DictionaryPath = dictionaryPath);
            updatedValue = updatedCatalog.Value( ...
                updatedCatalog.Path == "Vehicle.Mass");

            testCase.verifyEqual(receipt.ChangedCount, 1, AbsTol = 0);
            testCase.verifyEqual(updatedValue, originalValue + 0.5, ...
                AbsTol = 1e-12);
            testCase.verifyEqual(receipt.BackupPath, "");
        end

        function testVehicleParameterWriterRejectsUpperBound(testCase)
            projectRoot = string(fileparts(fileparts( ...
                mfilename("fullpath"))));
            temporaryFolder = string(tempname);
            mkdir(temporaryFolder);
            testCase.addTeardown(@() rmdir(temporaryFolder, "s"));
            dictionaryPath = fullfile(temporaryFolder, "VehicleData.sldd");
            copyfile(fullfile(projectRoot, "data", "VehicleData.sldd"), ...
                dictionaryPath);
            changes = table("Battery.InitialSOC", 1.1, ...
                'VariableNames', {'Path', 'Value'});

            testCase.verifyError(@() updateVehicleParameters(changes, ...
                ProjectRoot = projectRoot, ...
                DictionaryPath = dictionaryPath, ...
                CreateBackup = false), ...
                "FSAE:VehicleParameters:AboveUpperBound");
        end

        function testDefaultQuasiStaticConfiguration(testCase)
            cfg = testCase.App.buildQuasiStaticConfig();

            testCase.verifyEqual(cfg.EventName, "autocross");
            testCase.verifyEqual(cfg.PassCount, 4, AbsTol = 0);
            testCase.verifyTrue(isinf(cfg.MaximumSpeed));
            testCase.verifyEqual(cfg.FigureVisible, "off");
            testCase.verifyTrue(cfg.SaveResults);
        end

        function testDefaultConfiguration(testCase)
            cfg = testCase.App.buildRunConfig();

            testCase.verifyEqual(cfg.Track.Name, "autocross");
            testCase.verifyEqual(cfg.Vehicle.DynamicsModel, "7DOF");
            testCase.verifyEqual(cfg.Driver.Model, "adaptive_autocross");
            testCase.verifyEqual(cfg.Simulation.SolverProfile, "standard");
            testCase.verifyEqual(cfg.Track.SampleDistance, 0.5, ...
                AbsTol = 1e-12);
        end

        function testAppCreatesResultInventory(testCase)
            testCase.verifyTrue(isvalid(testCase.App.UIFigure));
            testCase.verifyClass(testCase.App.ResultsTable.Data, "table");
            testCase.verifyEqual(width(testCase.App.ResultsTable.Data), 6);
        end

        function testTablesAutoSizeAndFillAvailableWidth(testCase)
            testCase.verifyEqual( ...
                testCase.App.VehicleParameterTable.ColumnWidth, ...
                {'auto', 'auto', 'auto', 'auto', 'auto', 'auto', ...
                'auto', 'auto', 'auto', 'auto', '1x'});
            testCase.verifyEqual( ...
                testCase.App.QuasiSummaryTable.ColumnWidth, ...
                {'auto', '1x'});
            testCase.verifyEqual( ...
                testCase.App.QuasiResultsTable.ColumnWidth, ...
                {'auto', 'auto', '1x'});
            testCase.verifyEqual( ...
                testCase.App.SummaryTable.ColumnWidth, {'auto', '1x'});
            testCase.verifyEqual(testCase.App.ResultsTable.ColumnWidth, ...
                {'auto', 'auto', 'auto', 'auto', 'auto', '1x'});
        end

        function testTenDofKeepsAdaptiveDriver(testCase)
            testCase.chooseComponent( testCase.App.ModuleTabGroup, "时域闭环");
            testCase.chooseComponent( testCase.App.DynamicsDropDown, "10DOF");
            drawnow;

            testCase.verifyEqual( ...
                string(testCase.App.DriverDropDown.Value), "adaptive_autocross");
            cfg = testCase.App.buildRunConfig();
            testCase.verifyEqual(cfg.Vehicle.DynamicsModel, "10DOF");
            testCase.verifyEqual(cfg.Driver.Model, "adaptive_autocross");
        end

        function testAdaptiveDriverKeepsTenDof(testCase)
            testCase.chooseComponent( testCase.App.ModuleTabGroup, "时域闭环");
            testCase.chooseComponent( testCase.App.DynamicsDropDown, "10DOF");
            testCase.chooseComponent( testCase.App.DriverDropDown, ...
                "reference_speed");
            testCase.chooseComponent( testCase.App.DriverDropDown, ...
                "adaptive_autocross");
            drawnow;

            testCase.verifyEqual( ...
                string(testCase.App.DynamicsDropDown.Value), "10DOF");
        end

        function testLoadsExistingResult(testCase)
            files = testCase.App.listResultFiles();
            testCase.assumeNotEmpty(files, ...
                "项目中没有可用于加载测试的圈速结果。");

            testCase.chooseComponent( testCase.App.ModuleTabGroup, "时域闭环");
            testCase.chooseComponent( testCase.App.TabGroup, "历史结果");
            testCase.chooseComponent( testCase.App.ResultsTable, [1, 1]);
            testCase.pressComponent( testCase.App.LoadButton);
            drawnow;

            testCase.verifyTrue(startsWith(string( ...
                testCase.App.StatusLabel.Text), "已加载（"));
            testCase.verifyGreaterThan(height( ...
                testCase.App.SummaryTable.Data), 2);
            testCase.verifyNotEmpty(testCase.App.TrackAxes.Children);
        end

        function testRendersTimeAnalysisInsideApp(testCase)
            files = testCase.App.listResultFiles();
            testCase.assumeNotEmpty(files, ...
                "项目中没有可用于界面分析测试的圈速结果。");

            testCase.chooseComponent( testCase.App.ModuleTabGroup, "时域闭环");
            testCase.chooseComponent( testCase.App.TabGroup, "历史结果");
            testCase.chooseComponent( testCase.App.ResultsTable, [1, 1]);
            testCase.pressComponent( testCase.App.LoadButton);
            testCase.chooseComponent( testCase.App.TabGroup, "分析绘图");
            testCase.chooseComponent( testCase.App.AnalysisPlotTypeDropDown, "折线图");
            testCase.chooseComponent( testCase.App.AnalysisXAxisDropDown, ...
                "赛项累计距离 | Track.EventDistance (m)");
            testCase.chooseComponent( testCase.App.AnalysisDropDown, ...
                "车速 | Vehicle.Speed (m/s)");
            testCase.pressComponent( testCase.App.OpenAnalysisButton);
            drawnow;

            testCase.verifyNumElements( ...
                testCase.App.AnalysisOutputTabGroup.Children, 1);
            testCase.verifyNotEmpty(testCase.App.AnalysisAxes.Children);
            testCase.verifyTrue(startsWith(string( ...
                testCase.App.AnalysisStatusLabel.Text), "已在界面绘制"));
            testCase.verifyGreaterThan(height( ...
                testCase.App.AnalysisSummaryTable.Data), 2);
        end

        function testTimeAnalysisFormatControlsXAxis(testCase)
            testCase.chooseComponent( testCase.App.ModuleTabGroup, "时域闭环");
            testCase.chooseComponent( testCase.App.TabGroup, "分析绘图");
            testCase.chooseComponent( testCase.App.AnalysisPlotTypeDropDown, "赛道图");
            drawnow;

            testCase.verifyEqual(testCase.App.AnalysisXAxisDropDown.Enable, ...
                matlab.lang.OnOffSwitchState.off);

            testCase.chooseComponent( testCase.App.AnalysisPlotTypeDropDown, "折线图");
            drawnow;

            testCase.verifyEqual(testCase.App.AnalysisXAxisDropDown.Enable, ...
                matlab.lang.OnOffSwitchState.on);
        end

        function testTimeAnalysisBottomNoteRemoved(testCase)
            testCase.chooseComponent(testCase.App.ModuleTabGroup, "时域闭环");
            testCase.chooseComponent(testCase.App.TabGroup, "分析绘图");
            drawnow;

            analysisGrid = testCase.App.AnalysisOutputTabGroup.Parent;
            textAreas = findall(testCase.App.AnalysisTab, ...
                "Type", "uitextarea");

            testCase.verifyEmpty(textAreas);
            testCase.verifyNumElements(analysisGrid.RowHeight, 5);
            testCase.verifyEqual(analysisGrid.RowHeight{4}, '1x');
            testCase.verifyEqual( ...
                testCase.App.AnalysisOutputTabGroup.Layout.Row, 4);
            testCase.verifyEqual( ...
                testCase.App.AnalysisStatusLabel.Layout.Row, 5);
        end

        function testFourWheelTrackMapCreatesFourTabs(testCase)
            files = testCase.App.listResultFiles();
            testCase.assumeNotEmpty(files, ...
                "项目中没有可用于四轮赛道图测试的圈速结果。");

            testCase.chooseComponent( testCase.App.ModuleTabGroup, "时域闭环");
            testCase.App.loadResult(files(1));
            testCase.chooseComponent( testCase.App.TabGroup, "分析绘图");
            testCase.chooseComponent( testCase.App.AnalysisPlotTypeDropDown, "赛道图");
            testCase.chooseComponent( testCase.App.AnalysisDropDown, ...
                "四轮法向载荷 | Wheel.NormalLoad (N)");
            testCase.pressComponent( testCase.App.OpenAnalysisButton);
            drawnow;

            titles = string({ ...
                testCase.App.AnalysisOutputTabGroup.Children.Title});
            testCase.verifyNumElements(titles, 4);
            testCase.verifyTrue(all(ismember(["FL", "FR", "RL", "RR"], ...
                titles)));
            testCase.verifyNumElements(testCase.App.AnalysisAxes, 4);
        end

        function testSuspensionTravelTrackMapCreatesFourTabs(testCase)
            files = testCase.App.listResultFiles();
            testCase.assumeNotEmpty(files, ...
                "项目中没有可用于悬架行程赛道图测试的圈速结果。");
            [result, ~] = loadLapSimulationResult(files(1));
            result.Wheel.SuspensionDeflection = ...
                zeros(size(result.Wheel.NormalLoad));

            testCase.chooseComponent( testCase.App.ModuleTabGroup, "时域闭环");
            testCase.chooseComponent( testCase.App.TabGroup, "分析绘图");
            [axesHandles, plotInfo] = renderRacecarAnalysisTabs( ...
                testCase.App.AnalysisOutputTabGroup, result, ...
                "time_domain", "track", "Distance", ...
                "Wheel.SuspensionDeflection");
            drawnow;

            testCase.verifyNumElements(axesHandles, 4);
            testCase.verifyEqual(string(plotInfo.ChannelNames), ...
                ["FL"; "FR"; "RL"; "RR"]);
            testCase.verifyEqual(plotInfo.Unit, "m");
        end

        function testExpandedTimeAnalysisCatalog(testCase)
            files = testCase.App.listResultFiles();
            testCase.assumeNotEmpty(files, ...
                "项目中没有可用于扩展时域参数目录测试的结果。");
            [result, ~] = loadLapSimulationResult(files(1));

            [axisCatalog, parameterCatalog] = ...
                listRacecarAnalysisCatalog(result, "time_domain");
            requiredParameters = ["Track.LateralError", ...
                "Track.VehicleEnvelopeViolation", ...
                "Derived.PlanarAcceleration", ...
                "Derived.TireForceMagnitude", ...
                "Derived.CumulativeDischargeEnergy", ...
                "Derived.SensorYawRateError"];
            trackOnlyParameters = ["Time", "Distance", "Track.PathS", ...
                "Track.Curvature", "Vehicle.X", "Vehicle.Y", ...
                "Sensor.PositionX", "Sensor.PositionY", ...
                "Derived.TurnRadius", "Derived.DistanceRemaining"];

            testCase.verifyGreaterThanOrEqual(height(axisCatalog), 8);
            testCase.verifyGreaterThanOrEqual(height(parameterCatalog), 120);
            testCase.verifyTrue(all(ismember( ...
                requiredParameters, parameterCatalog.Path)));
            testCase.verifyFalse(any(ismember( ...
                trackOnlyParameters, parameterCatalog.Path)));
            testCase.verifyTrue(all(ismember( ...
                ["Time", "Distance", "Track.PathS", "Vehicle.X"], ...
                axisCatalog.Path)));
        end

        function testTimeDerivedAccelerationMatchesSource(testCase)
            files = testCase.App.listResultFiles();
            testCase.assumeNotEmpty(files, ...
                "项目中没有可用于派生加速度测试的时域结果。");
            [result, ~] = loadLapSimulationResult(files(1));

            derived = deriveRacecarAnalysisSignals(result, "time_domain");
            row = derived(derived.Path == "Derived.PlanarAcceleration", :);
            expected = hypot(result.Vehicle.Ax, result.Vehicle.Ay);

            testCase.verifyNumElements(row.Path, 1);
            testCase.verifyEqual(row.Data{1}, expected, AbsTol = 1e-12);
        end

        function testDerivedFourWheelTrackMapCreatesFourTabs(testCase)
            files = testCase.App.listResultFiles();
            testCase.assumeNotEmpty(files, ...
                "项目中没有可用于四轮派生参数测试的时域结果。");
            [result, ~] = loadLapSimulationResult(files(1));

            [axesHandles, plotInfo] = renderRacecarAnalysisTabs( ...
                testCase.App.AnalysisOutputTabGroup, result, ...
                "time_domain", "track", "Distance", ...
                "Derived.TireForceMagnitude");
            drawnow;

            testCase.verifyNumElements(axesHandles, 4);
            testCase.verifyEqual(string(plotInfo.ChannelNames), ...
                ["FL"; "FR"; "RL"; "RR"]);
            testCase.verifyEqual(plotInfo.Unit, "N");
        end

        function testLoadsExistingQuasiStaticResult(testCase)
            files = testCase.App.listQuasiStaticResultFiles();
            testCase.assumeNotEmpty(files, ...
                "项目中没有可用于加载测试的准静态圈速结果。");

            testCase.chooseComponent( testCase.App.ModuleTabGroup, "准静态");
            testCase.chooseComponent( testCase.App.QuasiTabGroup, "历史结果");
            testCase.chooseComponent( testCase.App.QuasiResultsTable, [1, 1]);
            testCase.pressComponent( testCase.App.QuasiLoadButton);
            drawnow;

            testCase.verifyTrue(startsWith(string( ...
                testCase.App.QuasiStatusLabel.Text), "已加载（"));
            testCase.verifyGreaterThan(height( ...
                testCase.App.QuasiSummaryTable.Data), 2);
            testCase.verifyNotEmpty(testCase.App.QuasiTrackAxes.Children);
        end

        function testRendersQuasiStaticAnalysisInsideApp(testCase)
            files = testCase.App.listQuasiStaticResultFiles();
            testCase.assumeNotEmpty(files, ...
                "项目中没有可用于界面分析测试的准静态结果。");

            testCase.chooseComponent( testCase.App.ModuleTabGroup, "准静态");
            testCase.chooseComponent( testCase.App.QuasiTabGroup, "历史结果");
            testCase.chooseComponent( testCase.App.QuasiResultsTable, [1, 1]);
            testCase.pressComponent( testCase.App.QuasiLoadButton);
            testCase.chooseComponent( testCase.App.QuasiTabGroup, "工具与分析");
            testCase.chooseComponent( testCase.App.QuasiAnalysisPlotTypeDropDown, ...
                "折线图");
            testCase.chooseComponent( testCase.App.QuasiAnalysisXAxisDropDown, ...
                "仿真时间 | SpeedProfile.Time (s)");
            testCase.chooseComponent( testCase.App.QuasiAnalysisDropDown, ...
                "车速 | SpeedProfile.Speed (m/s)");
            testCase.pressComponent( testCase.App.QuasiAnalysisButton);
            drawnow;

            testCase.verifyNumElements( ...
                testCase.App.QuasiAnalysisTabGroup.Children, 1);
            testCase.verifyNotEmpty(testCase.App.QuasiAnalysisAxes.Children);
            testCase.verifyTrue(startsWith(string( ...
                testCase.App.QuasiAnalysisStatusLabel.Text), "已在界面绘制"));
            testCase.verifyGreaterThan(height( ...
                testCase.App.QuasiAnalysisSummaryTable.Data), 2);
        end

        function testExpandedQuasiStaticAnalysisCatalog(testCase)
            files = testCase.App.listQuasiStaticResultFiles();
            testCase.assumeNotEmpty(files, ...
                "项目中没有可用于扩展准静态参数目录测试的结果。");
            loaded = load(files(1), "quasiStaticResult");

            [axisCatalog, parameterCatalog] = ...
                listRacecarAnalysisCatalog(loaded.quasiStaticResult, ...
                "quasi_static");
            requiredParameters = ["SpeedProfile.Speed", ...
                "SpeedProfile.LateralAcceleration", ...
                "Derived.LongitudinalAcceleration", ...
                "Derived.PlanarAccelerationG", ...
                "Derived.CurvatureSpeedUtilization", ...
                "Derived.GGVCombinedUtilization"];
            trackOnlyParameters = ["SpeedProfile.Time", ...
                "SpeedProfile.ProgressS", "SpeedProfile.Curvature", ...
                "Track.X", "Track.Y", "Track.Heading", ...
                "Track.Curvature", "Track.ReferenceSpeed", ...
                "Track.LeftHalfWidth", "Track.RightHalfWidth", ...
                "Derived.NormalizedProgress", "Derived.TurnRadius", ...
                "Derived.TurnDirection", "Derived.TrackWidth", ...
                "Derived.DistanceRemaining"];

            testCase.verifyGreaterThanOrEqual(height(axisCatalog), 5);
            testCase.verifyGreaterThanOrEqual(height(parameterCatalog), 25);
            testCase.verifyTrue(all(ismember( ...
                requiredParameters, parameterCatalog.Path)));
            testCase.verifyFalse(any(ismember( ...
                trackOnlyParameters, parameterCatalog.Path)));
            testCase.verifyTrue(all(ismember( ...
                ["SpeedProfile.Time", "SpeedProfile.ProgressS", ...
                "Track.X", "Track.Y"], axisCatalog.Path)));
        end

        function testQuasiAnalysisFormatControlsXAxis(testCase)
            testCase.chooseComponent( testCase.App.ModuleTabGroup, "准静态");
            testCase.chooseComponent( testCase.App.QuasiTabGroup, "工具与分析");
            testCase.chooseComponent( testCase.App.QuasiAnalysisPlotTypeDropDown, ...
                "赛道图");
            drawnow;

            testCase.verifyEqual( ...
                testCase.App.QuasiAnalysisXAxisDropDown.Enable, ...
                matlab.lang.OnOffSwitchState.off);

            testCase.chooseComponent( testCase.App.QuasiAnalysisPlotTypeDropDown, ...
                "折线图");
            drawnow;

            testCase.verifyEqual( ...
                testCase.App.QuasiAnalysisXAxisDropDown.Enable, ...
                matlab.lang.OnOffSwitchState.on);
        end

        function testAnalysisAxesFillResultTab(testCase)
            files = testCase.App.listQuasiStaticResultFiles();
            testCase.assumeNotEmpty(files, ...
                "项目中没有可用于分析布局测试的准静态结果。");

            testCase.chooseComponent(testCase.App.ModuleTabGroup, "准静态");
            testCase.App.loadQuasiStaticResult(files(1));
            testCase.chooseComponent(testCase.App.QuasiTabGroup, "工具与分析");
            testCase.pressComponent(testCase.App.QuasiAnalysisButton);
            drawnow;

            gridLayout = testCase.App.QuasiAnalysisAxes.Parent;
            testCase.verifyClass(gridLayout, "matlab.ui.container.GridLayout");
            testCase.verifyEqual(gridLayout.RowHeight, {'1x'});
            testCase.verifyEqual(gridLayout.ColumnWidth, {'1x'});
            testCase.verifyEqual(gridLayout.Padding, [8, 8, 8, 8]);
            testCase.verifyEqual(testCase.App.QuasiAnalysisAxes.Layout.Row, 1);
            testCase.verifyEqual(testCase.App.QuasiAnalysisAxes.Layout.Column, 1);
        end

        function testRendersQuasiStaticTrackMap(testCase)
            files = testCase.App.listQuasiStaticResultFiles();
            testCase.assumeNotEmpty(files, ...
                "项目中没有可用于准静态赛道图测试的结果。");

            testCase.chooseComponent( testCase.App.ModuleTabGroup, "准静态");
            testCase.App.loadQuasiStaticResult(files(1));
            testCase.chooseComponent( testCase.App.QuasiTabGroup, "工具与分析");
            testCase.chooseComponent( testCase.App.QuasiAnalysisPlotTypeDropDown, ...
                "赛道图");
            testCase.chooseComponent( testCase.App.QuasiAnalysisDropDown, ...
                "横向加速度 | SpeedProfile.LateralAcceleration (m/s^2)");
            testCase.pressComponent( testCase.App.QuasiAnalysisButton);
            drawnow;

            testCase.verifyNumElements( ...
                testCase.App.QuasiAnalysisTabGroup.Children, 1);
            testCase.verifyNumElements(testCase.App.QuasiAnalysisAxes, 1);
            testCase.verifyNotEmpty(testCase.App.QuasiAnalysisAxes.Children);
        end

        function testQuasiStaticGGVActionsHaveDistinctEntrances(testCase)
            browseButton = findall(testCase.App.UIFigure, ...
                "Type", "uibutton", "Text", "打开文件夹");
            generateButton = findall(testCase.App.UIFigure, ...
                "Type", "uibutton", "Text", "生成 GGV");
            latestButton = findall(testCase.App.UIFigure, ...
                "Type", "uibutton", "Text", "打开最新 GGV 图窗");
            configPanel = findall(testCase.App.UIFigure, ...
                "Type", "uipanel", "Title", "准静态配置");

            testCase.verifyNumElements(browseButton, 1);
            testCase.verifyNumElements(generateButton, 1);
            testCase.verifyNumElements(latestButton, 1);
            testCase.verifyNotEmpty(browseButton.ButtonPushedFcn);
            testCase.verifyNotEmpty(generateButton.ButtonPushedFcn);
            testCase.verifyNotEmpty(latestButton.ButtonPushedFcn);
            testCase.verifyEqual(ancestor(latestButton, "uipanel"), ...
                configPanel);

            testCase.verifyNumElements(findall(testCase.App.UIFigure, ...
                "Type", "uibutton", "Text", "GGV 配置"), 0);
            testCase.verifyNumElements(findall(testCase.App.UIFigure, ...
                "Type", "uibutton", "Text", "参数扫描配置"), 0);
            testCase.verifyNumElements(findall(testCase.App.UIFigure, ...
                "Type", "uibutton", "Text", "运行 GGV（默认配置）"), 0);
            testCase.verifyNumElements(findall(testCase.App.UIFigure, ...
                "Type", "uibutton", "Text", "绘制最新参数扫描分析"), 0);
        end

        function testDefaultQuasiStaticSweepConfiguration(testCase)
            cfg = testCase.App.buildQuasiStaticSweepConfig();

            testCase.verifyEqual(cfg.ScanMode, "double");
            testCase.verifyEqual(cfg.EventName, "autocross");
            testCase.verifyEqual(cfg.Parameter1Path, "Vehicle.Mass");
            testCase.verifyEqual(cfg.Parameter2Path, ...
                "Powertrain.GearRatio");
            testCase.verifyEqual(cfg.CaseCount, 25, AbsTol = 0);
            testCase.verifyEqual(string( ...
                testCase.App.QuasiSweepParameter1PointCountLabel.Text), ...
                "点数：5");
            testCase.verifyEqual(string( ...
                testCase.App.QuasiSweepParameter2PointCountLabel.Text), ...
                "点数：5");
            testCase.verifyEqual( ...
                testCase.App.QuasiSweepParameter1UnitLabel.Layout.Row, ...
                testCase.App.QuasiSweepParameter1PointCountLabel.Layout.Row);
            testCase.verifyEqual( ...
                testCase.App.QuasiSweepParameter2UnitLabel.Layout.Row, ...
                testCase.App.QuasiSweepParameter2PointCountLabel.Layout.Row);
            testCase.verifyTrue(any(contains(string( ...
                testCase.App.QuasiSweepParameter1DropDown.Items), ...
                "Vehicle.Mass")));
        end

        function testSweepCatalogIncludesAllSupportedScalars(testCase)
            catalog = testCase.App.listQuasiStaticSweepParameterCatalog();
            requiredPaths = [ ...
                "Vehicle.Mass"; "Powertrain.GearRatio"; ...
                "Vehicle.Wheelbase"; "Vehicle.TrackFront"; ...
                "Vehicle.TrackRear"; "Vehicle.CGHeight"; ...
                "Vehicle.CGToFrontAxle"; ...
                "Vehicle.RollStiffnessDistributionFront"; ...
                "Tire.EffectiveRadius"; "Tire.LowSpeedEpsilon"; ...
                "Aero.CdA"; "Aero.ClAFront"; "Aero.ClARear"; ...
                "Aero.YawCorrection"; "Powertrain.GearEfficiency"; ...
                "Powertrain.MotorTorqueLimit"; ...
                "Powertrain.MotorSpeedLimit"; ...
                "Powertrain.InverterEfficiency"; ...
                "Battery.PowerLimitDrive"; ...
                "Battery.PowerLimitRegen"];
            allowedPaths = [requiredPaths; "Brake.MaxTotalForce"];

            testCase.verifyTrue(all(ismember(requiredPaths, catalog.Path)));
            testCase.verifyTrue(all(ismember(catalog.Path, allowedPaths)));
            testCase.verifyGreaterThanOrEqual(height(catalog), 20);
            testCase.verifyEqual(numel(unique(catalog.Path)), height(catalog));
            testCase.verifyEqual(numel( ...
                testCase.App.QuasiSweepParameter1DropDown.Items), ...
                height(catalog));
            testCase.verifyTrue(all(contains(catalog.Label, " | ")));
            testCase.verifyTrue(all(isfinite(catalog.Start)));
            testCase.verifyTrue(all(catalog.Stop > catalog.Start));
            testCase.verifyTrue(all(catalog.Step > 0));
            testCase.verifyFalse(any(catalog.Path == "Aero.YawAngleGrid"));
        end

        function testQuasiStaticSweepTrackSelection(testCase)
            testCase.App.QuasiSweepEventDropDown.Value = 'skidpad';
            drawnow;
            cfg = testCase.App.buildQuasiStaticSweepConfig();

            testCase.verifyEqual(cfg.EventName, "skidpad");
            testCase.verifyTrue(any(contains(string( ...
                testCase.App.QuasiSweepEventDropDown.Items), ...
                "八字绕环 | skidpad")));
        end

        function testSingleSweepDisablesSecondParameter(testCase)
            testCase.chooseComponent( testCase.App.QuasiTabGroup, "参数扫描");
            testCase.chooseComponent( testCase.App.QuasiSweepModeDropDown, ...
                "单参数");
            drawnow;
            cfg = testCase.App.buildQuasiStaticSweepConfig();

            testCase.verifyEqual(cfg.ScanMode, "single");
            testCase.verifyEqual(cfg.Parameter2Path, "none");
            testCase.verifyEqual(cfg.CaseCount, 5, AbsTol = 0);
            testCase.verifyEqual( ...
                testCase.App.QuasiSweepParameter2DropDown.Enable, ...
                matlab.lang.OnOffSwitchState.off);
            testCase.verifyEqual(string( ...
                testCase.App.QuasiSweepParameter2PointCountLabel.Text), ...
                "点数：未启用");
        end

        function testSweepRejectsDescendingRange(testCase)
            testCase.App.QuasiSweepParameter1StartField.Value = 360;
            testCase.App.QuasiSweepParameter1StopField.Value = 280;

            testCase.verifyError( ...
                @() testCase.App.buildQuasiStaticSweepConfig(), ...
                "FSAE:QuasiStatic:InvalidScanRange");
        end

        function testSweepProgressUpdatesGaugeAndLabel(testCase)
            reset = struct( ...
                "Reset", true, "Complete", false, ...
                "Completed", 0, "Total", 5, "Fraction", 0.0, ...
                "Parameter1", "", "Value1", NaN, ...
                "Parameter2", "none", "Value2", NaN, ...
                "LapTime", NaN, "SuccessCount", 0, "FailureCount", 0);
            progress = struct( ...
                "Reset", false, "Complete", false, ...
                "Completed", 2, "Total", 5, "Fraction", 0.4, ...
                "Parameter1", "Vehicle.Mass", "Value1", 300, ...
                "Parameter2", "none", "Value2", NaN, ...
                "LapTime", 61.5, "SuccessCount", 0, "FailureCount", 0);

            testCase.App.reportQuasiStaticSweepProgress(reset);
            testCase.App.reportQuasiStaticSweepProgress(progress);

            testCase.verifyEqual( ...
                testCase.App.QuasiSweepProgressGauge.Value, 40, ...
                AbsTol = 1e-12);
            testCase.verifyTrue(contains(string( ...
                testCase.App.QuasiSweepProgressLabel.Text), "2 / 5"));
            testCase.verifyTrue(contains(string( ...
                testCase.App.QuasiSweepProgressLabel.Text), ...
                "Vehicle.Mass=300"));
        end

        function testDisplaysCompletedSweepInGui(testCase)
            results = repmat(struct( ...
                "Success", true, "LapTime", NaN, ...
                "Parameter1Value", NaN, "Parameter2Value", NaN), 4, 1);
            parameter1Values = num2cell([280; 300; 280; 300]);
            parameter2Values = num2cell([11; 11; 13; 13]);
            lapTimes = num2cell([62.8; 63.1; 62.5; 62.9]);
            [results.Parameter1Value] = parameter1Values{:};
            [results.Parameter2Value] = parameter2Values{:};
            [results.LapTime] = lapTimes{:};
            scanConfig = struct( ...
                "Mode", "double", "Event", "autocross", ...
                "Parameter1", "Vehicle.Mass", ...
                "Parameter2", "Powertrain.GearRatio");
            originalItemsData = ...
                testCase.App.QuasiAnalysisDropDown.ItemsData;

            testCase.App.displayQuasiStaticSweepResults(results, scanConfig);
            drawnow;

            testCase.verifyEqual( ...
                testCase.App.QuasiAnalysisDropDown.ItemsData, ...
                originalItemsData);
            testCase.verifyTrue(ismember( ...
                testCase.App.QuasiAnalysisDropDown.Value, ...
                testCase.App.QuasiAnalysisDropDown.ItemsData));
            testCase.verifyEqual( ...
                testCase.App.QuasiTabGroup.SelectedTab, ...
                testCase.App.QuasiSweepTab);
            testCase.verifyNotEmpty( ...
                testCase.App.QuasiSweepResultAxes.Children);
            testCase.verifyTrue(contains(string( ...
                testCase.App.QuasiSweepProgressLabel.Text), ...
                "结果图已生成"));
        end

        function testCloseRequestClosesWindow(testCase)
            figureHandle = testCase.App.UIFigure;
            closeCallback = figureHandle.CloseRequestFcn;
            closeCallback(figureHandle, []);
            drawnow;

            testCase.verifyFalse(isvalid(figureHandle));
        end

        function testQuasiStaticGGVFileSelectionFeedsRunConfig(testCase)
            projectRoot = string(fileparts(fileparts( ...
                mfilename("fullpath"))));
            ggvFiles = dir(fullfile(projectRoot, "results", ...
                "quasi_static", "ggv", "*.mat"));
            testCase.assumeNotEmpty(ggvFiles, ...
                "项目中没有可用于 GGV 文件选择测试的 MAT 结果。");
            ggvPath = string(fullfile(ggvFiles(1).folder, ...
                ggvFiles(1).name));

            testCase.App.QuasiGGVFileField.Value = char(ggvPath);
            cfg = testCase.App.buildQuasiStaticConfig();

            testCase.verifyEqual(cfg.GGVFile, ggvPath);
            testCase.verifyEqual(testCase.App.QuasiGGVFileField.Value, ...
                char(ggvPath));
        end
    end

    methods (Access = private)
        function chooseComponent(testCase, component, selection)
            if isprop(component, "SelectedTab")
                titles = string({component.Children.Title});
                index = find(titles == string(selection), 1);
                assert(~isempty(index), "FSAE:Test:UnknownTab", ...
                    "找不到标签页：%s。", string(selection));
                component.SelectedTab = component.Children(index);
            elseif isa(component, "matlab.ui.control.Table")
                event = struct("Indices", selection);
                testCase.invokeComponentCallback( ...
                    component.CellSelectionCallback, component, event);
            else
                items = string(component.Items);
                index = find(items == string(selection), 1);
                assert(~isempty(index), "FSAE:Test:UnknownItem", ...
                    "找不到控件选项：%s。", string(selection));
                previousValue = component.Value;
                if isempty(component.ItemsData)
                    component.Value = char(items(index));
                elseif iscell(component.ItemsData)
                    component.Value = component.ItemsData{index};
                else
                    component.Value = component.ItemsData(index);
                end
                event = struct("Value", component.Value, ...
                    "PreviousValue", previousValue);
                testCase.invokeComponentCallback( ...
                    component.ValueChangedFcn, component, event);
            end
            drawnow;
        end

        function pressComponent(testCase, component)
            testCase.invokeComponentCallback( ...
                component.ButtonPushedFcn, component, struct());
            drawnow;
        end

        function typeComponent(testCase, component, indices, newData)
            event = struct("Indices", indices, "NewData", double(newData));
            testCase.invokeComponentCallback( ...
                component.CellEditCallback, component, event);
            drawnow;
        end

        function invokeComponentCallback(~, callback, source, event)
            if isempty(callback)
                return
            end
            if isa(callback, "function_handle")
                callback(source, event);
            else
                feval(callback{1}, source, event, callback{2:end});
            end
        end
    end
end
