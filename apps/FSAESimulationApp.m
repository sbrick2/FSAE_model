classdef FSAESimulationApp < handle
    %FSAESIMULATIONAPP FSAE 整车仿真与结果分析图形界面。

    properties (SetAccess = private)
        UIFigure
        ModuleTabGroup
        QuasiStaticModuleTab
        TimeDomainModuleTab
        VehicleParameterModuleTab
        VehicleParameterGroupDropDown
        VehicleParameterSearchField
        VehicleParameterCountLabel
        VehicleParameterTable
        VehicleParameterDetailsArea
        VehicleParameterStatusLabel
        VehicleParameterRefreshButton
        VehicleParameterRevertButton
        VehicleParameterSaveButton
        QuasiEventDropDown
        QuasiGGVFileField
        QuasiGGVBrowseButton
        QuasiGGVGenerateButton
        QuasiOpenLatestGGVButton
        QuasiPassCountField
        QuasiMaximumSpeedField
        QuasiSaveCheckBox
        QuasiRunButton
        QuasiRefreshButton
        QuasiLoadButton
        QuasiStatusLabel
        QuasiSummaryTable
        QuasiResultsTable
        QuasiTrackAxes
        QuasiAnalysisPlotTypeDropDown
        QuasiAnalysisXAxisDropDown
        QuasiAnalysisDropDown
        QuasiAnalysisButton
        QuasiAnalysisTabGroup
        QuasiAnalysisAxes
        QuasiAnalysisSummaryTable
        QuasiAnalysisStatusLabel
        QuasiSweepRunButton
        QuasiToolsTab
        QuasiSweepTab
        QuasiSweepEventDropDown
        QuasiSweepModeDropDown
        QuasiSweepParameter1DropDown
        QuasiSweepParameter1StartField
        QuasiSweepParameter1StopField
        QuasiSweepParameter1StepField
        QuasiSweepParameter1UnitLabel
        QuasiSweepParameter1PointCountLabel
        QuasiSweepParameter2DropDown
        QuasiSweepParameter2StartField
        QuasiSweepParameter2StopField
        QuasiSweepParameter2StepField
        QuasiSweepParameter2UnitLabel
        QuasiSweepParameter2PointCountLabel
        QuasiSweepCaseCountLabel
        QuasiSweepProgressGauge
        QuasiSweepProgressLabel
        QuasiSweepResultAxes
        QuasiLogTextArea
        QuasiTabGroup
        QuasiOverviewTab
        EventDropDown
        LapsField
        SampleDistanceField
        DynamicsDropDown
        DriverDropDown
        TireDropDown
        SolverDropDown
        StopTimeField
        RunNameField
        SaveFigureCheckBox
        RunButton
        RefreshButton
        LoadButton
        BrowseButton
        OpenFolderButton
        OpenAnalysisButton
        DetailedAnalysisButton
        AnalysisPlotTypeDropDown
        AnalysisXAxisDropDown
        AnalysisDropDown
        AnalysisOutputTabGroup
        AnalysisAxes
        AnalysisSummaryTable
        AnalysisStatusLabel
        StatusLabel
        SummaryTable
        ResultsTable
        TrackAxes
        LogTextArea
        TabGroup
        OverviewTab
        TrackTab
        ResultsTab
        AnalysisTab
    end

    properties (Access = private)
        ProjectRoot (1, 1) string
        ResultFiles (:, 1) string = strings(0, 1)
        SelectedResultIndex (1, 1) double = NaN
        CurrentResult = struct
        CurrentResultFile (1, 1) string = ""
        QuasiResultFiles (:, 1) string = strings(0, 1)
        QuasiSelectedResultIndex (1, 1) double = NaN
        CurrentQuasiResult = struct
        CurrentQuasiResultFile (1, 1) string = ""
        QuasiSweepCatalog = table()
        VehicleParameterCatalog = table()
        OriginalVehicleParameterCatalog = table()
        IsRunning (1, 1) logical = false
    end

    methods
        function app = FSAESimulationApp(options)
            arguments
                options.ProjectRoot (1, 1) string = ""
                options.Visible (1, 1) string {mustBeMember( ...
                    options.Visible, ["on", "off"])} = "on"
            end

            app.ProjectRoot = app.resolveProjectRoot(options.ProjectRoot);
            app.addProjectPaths();
            app.createComponents();
            app.refreshQuasiStaticResults();
            app.refreshResults();
            app.synchronizeConfiguration("startup");
            app.UIFigure.Visible = options.Visible;
        end

        function delete(app)
            %DELETE 关闭窗口；即使类被清除导致窗口句柄已失效，也不报错。
            try
                figureHandle = app.UIFigure;
                if ~isempty(figureHandle) && isvalid(figureHandle)
                    figureHandle.CloseRequestFcn = [];
                    delete(figureHandle);
                end
            catch
                % 析构阶段不应因失效的 UIFigure 阻止对象清理。
            end
        end

        function cfg = buildRunConfig(app)
            %BUILDRUNCONFIG 将界面选择转换为完整圈速配置。
            cfg = createLapSimulationConfig();
            cfg.Track.Name = lower(string(app.EventDropDown.Value));
            cfg.Track.NumberOfLaps = round(app.LapsField.Value);
            cfg.Track.SampleDistance = app.SampleDistanceField.Value;
            cfg.Vehicle.DynamicsModel = string(app.DynamicsDropDown.Value);
            cfg.Vehicle.TireModel = lower(string(app.TireDropDown.Value));
            cfg.Driver.Model = lower(string(app.DriverDropDown.Value));
            cfg.Simulation.SolverProfile = lower( ...
                string(app.SolverDropDown.Value));
            cfg.Simulation.StopTime = app.StopTimeField.Value;
            cfg.Visualization.Enabled = false;
            cfg.Output.RunName = string(app.RunNameField.Value);
            cfg.Output.SaveFinalTrackView = ...
                logical(app.SaveFigureCheckBox.Value);

        end

        function cfg = buildQuasiStaticConfig(app)
            %BUILDQUASISTATICCONFIG 将准静态界面选择转换为脚本配置。
            cfg = struct( ...
                "EventName", lower(string(app.QuasiEventDropDown.Value)), ...
                "GGVFile", string(app.QuasiGGVFileField.Value), ...
                "SampleDistance", NaN, ...
                "InitialSpeed", 0.0, ...
                "FinalSpeed", NaN, ...
                "PassCount", round(app.QuasiPassCountField.Value), ...
                "MaximumSpeed", app.QuasiMaximumSpeedField.Value, ...
                "FigureVisible", "off", ...
                "SaveResults", logical(app.QuasiSaveCheckBox.Value), ...
                "OutputFolder", "");
        end

        function cfg = buildQuasiStaticSweepConfig(app)
            %BUILDQUASISTATICSWEEPCONFIG 将参数扫描控件转换为脚本配置。
            mode = lower(string(app.QuasiSweepModeDropDown.Value));
            parameter1Path = string( ...
                app.QuasiSweepParameter1DropDown.Value);
            parameter1Values = createSweepValues( ...
                app.QuasiSweepParameter1StartField.Value, ...
                app.QuasiSweepParameter1StopField.Value, ...
                app.QuasiSweepParameter1StepField.Value);

            parameter2Path = "none";
            parameter2Unit = "1";
            parameter2Values = NaN;
            if mode == "double"
                parameter2Path = string( ...
                    app.QuasiSweepParameter2DropDown.Value);
                assert(parameter1Path ~= parameter2Path, ...
                    "FSAE:App:DuplicateSweepParameter", ...
                    "双参数扫描必须选择两个不同的参数。");
                parameter2Unit = string(erase( ...
                    app.QuasiSweepParameter2UnitLabel.Text, "单位："));
                parameter2Values = createSweepValues( ...
                    app.QuasiSweepParameter2StartField.Value, ...
                    app.QuasiSweepParameter2StopField.Value, ...
                    app.QuasiSweepParameter2StepField.Value);
            end

            cfg = struct( ...
                "ScanMode", mode, ...
                "EventName", lower(string(app.QuasiSweepEventDropDown.Value)), ...
                "Parameter1Path", parameter1Path, ...
                "Parameter1Unit", string( ...
                    erase(app.QuasiSweepParameter1UnitLabel.Text, "单位：")), ...
                "Parameter1Start", parameter1Values(1), ...
                "Parameter1Stop", parameter1Values(end), ...
                "Parameter1Step", ...
                    app.QuasiSweepParameter1StepField.Value, ...
                "Parameter2Path", parameter2Path, ...
                "Parameter2Unit", parameter2Unit, ...
                "Parameter2Start", parameter2Values(1), ...
                "Parameter2Stop", parameter2Values(end), ...
                "Parameter2Step", ...
                    app.QuasiSweepParameter2StepField.Value, ...
                "CaseCount", numel(parameter1Values) * ...
                    max(1, numel(parameter2Values)), ...
                "ShowSummaryPlot", true, ...
                "SaveResults", logical(app.QuasiSaveCheckBox.Value), ...
                "StoreCaseDetails", true);
        end

        function reportQuasiStaticSweepProgress(app, progress)
            %REPORTQUASISTATICSWEEPPROGRESS 接收扫描脚本的客户端进度消息。
            fraction = min(max(double(progress.Fraction), 0.0), 1.0);
            app.QuasiSweepProgressGauge.Value = 100.0 * fraction;

            if isfield(progress, "Reset") && progress.Reset
                app.QuasiSweepProgressLabel.Text = char(sprintf( ...
                    "准备扫描：0 / %d（0.0%%）", progress.Total));
            elseif isfield(progress, "Complete") && progress.Complete
                app.QuasiSweepProgressLabel.Text = char(sprintf( ...
                    "扫描完成：%d / %d（100%%），成功 %d，失败 %d", ...
                    progress.Completed, progress.Total, ...
                    progress.SuccessCount, progress.FailureCount));
            else
                currentCase = sprintf('%s=%.6g', ...
                    char(string(progress.Parameter1)), progress.Value1);
                if string(progress.Parameter2) ~= "none"
                    currentCase = sprintf('%s, %s=%.6g', ...
                        currentCase, char(string(progress.Parameter2)), ...
                        progress.Value2);
                end
                lapTimeText = '';
                if isfinite(progress.LapTime)
                    lapTimeText = sprintf( ...
                        '，圈时 %.3f s', progress.LapTime);
                end
                app.QuasiSweepProgressLabel.Text = char(sprintf( ...
                    '%d / %d（%.1f%%）：%s%s', ...
                    progress.Completed, progress.Total, 100.0 * fraction, ...
                    currentCase, lapTimeText));
            end
            drawnow limitrate;
        end

        function files = listResultFiles(app)
            %LISTRESULTFILES 返回结果浏览器当前索引的 MAT 文件。
            files = app.ResultFiles;
        end

        function files = listQuasiStaticResultFiles(app)
            %LISTQUASISTATICRESULTFILES 返回准静态圈速结果索引。
            files = app.QuasiResultFiles;
        end

        function catalog = listQuasiStaticSweepParameterCatalog(app)
            %LISTQUASISTATICSWEEPPARAMETERCATALOG 返回 GUI 可扫描参数目录。
            catalog = app.quasiSweepParameterCatalog();
        end

        function catalog = listVehicleParameterCatalog(app)
            %LISTVEHICLEPARAMETERCATALOG 返回车辆参数调整页的完整目录。
            catalog = app.VehicleParameterCatalog;
        end

        function displayQuasiStaticSweepResults(app, results, scanConfig)
            %DISPLAYQUASISTATICSWEEPRESULTS 在 GUI 中显示本次参数扫描结果。
            assert(isstruct(results) && ~isempty(results), ...
                "FSAE:App:EmptySweepResult", "参数扫描没有返回结果。");
            requiredFields = {'Success', 'LapTime', ...
                'Parameter1Value', 'Parameter2Value'};
            assert(all(isfield(results, requiredFields)), ...
                "FSAE:App:InvalidSweepResult", ...
                "参数扫描结果缺少绘图所需字段。");
            assert(isstruct(scanConfig) && all(isfield(scanConfig, ...
                {'Mode', 'Event', 'Parameter1', 'Parameter2'})), ...
                "FSAE:App:InvalidSweepConfig", ...
                "参数扫描配置缺少绘图所需字段。");

            mode = lower(string(scanConfig.Mode));
            parameter1 = string(scanConfig.Parameter1);
            parameter2 = string(scanConfig.Parameter2);
            p1 = double([results.Parameter1Value]).';
            p2 = double([results.Parameter2Value]).';
            lapTime = double([results.LapTime]).';
            valid = logical([results.Success]).' & ...
                isfinite(p1) & isfinite(lapTime);
            if mode == "double"
                valid = valid & isfinite(p2);
            end
            assert(any(valid), "FSAE:App:NoSweepPlotResult", ...
                "参数扫描没有可绘制的成功工况。");

            cla(app.QuasiSweepResultAxes);
            axis(app.QuasiSweepResultAxes, "normal");
            colorbar(app.QuasiSweepResultAxes, "off");

            if mode == "single"
                [parameterValues, order] = sort(p1(valid));
                eventTimes = lapTime(valid);
                plot(app.QuasiSweepResultAxes, parameterValues, ...
                    eventTimes(order), "-o", "LineWidth", 1.2);
                xlabel(app.QuasiSweepResultAxes, parameter1, ...
                    "Interpreter", "none");
                ylabel(app.QuasiSweepResultAxes, "Event time (s)");
            else
                parameter1Values = unique(p1(valid), "sorted");
                parameter2Values = unique(p2(valid), "sorted");
                [~, row] = ismember(p2(valid), parameter2Values);
                [~, column] = ismember(p1(valid), parameter1Values);
                lapTimeGrid = accumarray([row(:), column(:)], ...
                    lapTime(valid), ...
                    [numel(parameter2Values), numel(parameter1Values)], ...
                    @min, NaN);
                imagesc(app.QuasiSweepResultAxes, parameter1Values, ...
                    parameter2Values, lapTimeGrid);
                set(app.QuasiSweepResultAxes, "YDir", "normal");
                sweepColorbar = colorbar(app.QuasiSweepResultAxes);
                sweepColorbar.Label.String = "Event time (s)";
                xlabel(app.QuasiSweepResultAxes, parameter1, ...
                    "Interpreter", "none");
                ylabel(app.QuasiSweepResultAxes, parameter2, ...
                    "Interpreter", "none");
            end
            grid(app.QuasiSweepResultAxes, "on");
            box(app.QuasiSweepResultAxes, "on");
            title(app.QuasiSweepResultAxes, ...
                "准静态参数扫描：" + string(scanConfig.Event), ...
                "Interpreter", "none");

            bestLapTime = min(lapTime(valid));
            app.QuasiSweepProgressLabel.Text = char(sprintf( ...
                '结果图已生成：成功 %d / %d，最佳圈时 %.3f s', ...
                nnz(valid), numel(results), bestLapTime));
            app.QuasiTabGroup.SelectedTab = app.QuasiSweepTab;
            app.appendQuasiLog('本次参数扫描结果已显示在参数扫描页。');
            drawnow;
        end

        function changes = buildVehicleParameterChanges(app)
            %BUILDVEHICLEPARAMETERCHANGES 返回已修改且通过校验的参数。
            changeMask = app.vehicleParameterChangeMask();
            changes = app.VehicleParameterCatalog(changeMask, ...
                ["Path", "Value"]);
            for row = 1:height(changes)
                catalogRow = app.VehicleParameterCatalog( ...
                    app.VehicleParameterCatalog.Path == changes.Path(row), :);
                app.validateVehicleParameterValue( ...
                    catalogRow, changes.Value(row));
            end
        end

        function loadResult(app, filePath)
            %LOADRESULT 加载结果并刷新摘要和轨迹页。
            filePath = string(filePath);
            [result, resolvedPath] = loadLapSimulationResult(filePath);
            app.CurrentResult = result;
            app.CurrentResultFile = resolvedPath;
            app.updateSummary(result, resolvedPath);
            app.updateTrackPreview(result);
            app.updateTimeAnalysisCatalog(result);
            app.StatusLabel.Text = char("已加载：" + resolvedPath);
            app.appendLog("加载结果：" + resolvedPath);
        end

        function loadQuasiStaticResult(app, filePath)
            %LOADQUASISTATICRESULT 加载准静态圈速结果并刷新模块。
            filePath = string(filePath);
            loaded = load(filePath, "quasiStaticResult");
            assert(isfield(loaded, "quasiStaticResult") && ...
                isstruct(loaded.quasiStaticResult), ...
                "FSAE:App:QuasiStaticResult", ...
                "MAT 文件不包含 quasiStaticResult：%s", filePath);
            app.CurrentQuasiResult = loaded.quasiStaticResult;
            app.CurrentQuasiResultFile = filePath;
            app.updateQuasiStaticResult(loaded.quasiStaticResult, filePath);
            app.updateQuasiAnalysisCatalog(loaded.quasiStaticResult);
            app.QuasiStatusLabel.Text = char("已加载：" + filePath);
            app.appendQuasiLog("加载结果：" + filePath);
        end

        function refreshResults(app)
            %REFRESHRESULTS 重新扫描已保存的圈速结果。
            inventory = app.buildResultInventory();
            app.ResultFiles = inventory.ResultFile;
            app.ResultsTable.Data = inventory(:, ["Event", "Run", ...
                "Modified", "EventTime_s", "LapTime_s", "Status"]);
            if isempty(app.ResultFiles)
                app.SelectedResultIndex = NaN;
                app.StatusLabel.Text = '未找到已保存的圈速结果';
            else
                app.SelectedResultIndex = 1;
                app.StatusLabel.Text = char(sprintf( ...
                    '就绪：发现 %d 个圈速结果', numel(app.ResultFiles)));
            end
        end

        function refreshQuasiStaticResults(app)
            %REFRESHQUASISTATICRESULTS 重新扫描准静态单次圈速结果。
            inventory = app.buildQuasiStaticResultInventory();
            app.QuasiResultFiles = inventory.ResultFile;
            app.QuasiResultsTable.Data = inventory(:, ...
                ["Event", "Run", "Modified"]);
            if isempty(app.QuasiResultFiles)
                app.QuasiSelectedResultIndex = NaN;
                app.QuasiStatusLabel.Text = '未找到准静态单次圈速结果';
            else
                app.QuasiSelectedResultIndex = 1;
                app.QuasiStatusLabel.Text = char(sprintf( ...
                    '就绪：发现 %d 个准静态圈速结果', ...
                    numel(app.QuasiResultFiles)));
            end
        end
    end

    methods (Access = private)
        function createComponents(app)
            app.UIFigure = uifigure( ...
                "Name", "FSAE 整车仿真中心", ...
                "Visible", "off", ...
                "Color", [0.96, 0.97, 0.99], ...
                "Position", [80, 60, 1460, 860]);
            app.UIFigure.CloseRequestFcn = ...
                @(source, ~) FSAESimulationApp.closeFigure(source);

            moduleGrid = uigridlayout(app.UIFigure, [1, 1]);
            moduleGrid.Padding = [8, 8, 8, 8];
            app.ModuleTabGroup = uitabgroup(moduleGrid);
            app.QuasiStaticModuleTab = uitab(app.ModuleTabGroup, ...
                "Title", "准静态");
            app.TimeDomainModuleTab = uitab(app.ModuleTabGroup, ...
                "Title", "时域闭环");
            app.VehicleParameterModuleTab = uitab(app.ModuleTabGroup, ...
                "Title", "车辆参数");

            rootGrid = uigridlayout(app.TimeDomainModuleTab, [1, 2]);
            rootGrid.ColumnWidth = {350, '1x'};
            rootGrid.RowHeight = {'1x'};
            rootGrid.Padding = [12, 12, 12, 12];
            rootGrid.ColumnSpacing = 12;

            configPanel = uipanel(rootGrid, ...
                "Title", "仿真配置", ...
                "FontWeight", "bold", ...
                "BackgroundColor", [1, 1, 1]);
            configPanel.Layout.Row = 1;
            configPanel.Layout.Column = 1;
            configGrid = uigridlayout(configPanel, [15, 2]);
            configGrid.ColumnWidth = {132, '1x'};
            configGrid.RowHeight = {48, 30, 30, 30, 30, 30, 30, 30, ...
                30, 30, 32, 38, 38, 34, '1x'};
            configGrid.Padding = [12, 8, 12, 12];
            configGrid.RowSpacing = 8;

            titleLabel = uilabel(configGrid, ...
                "Text", "FSAE Simulation", ...
                "FontSize", 21, ...
                "FontWeight", "bold", ...
                "FontColor", [0.02, 0.25, 0.48]);
            titleLabel.Layout.Row = 1;
            titleLabel.Layout.Column = [1, 2];

            app.addConfigLabel(configGrid, 2, "赛事");
            app.EventDropDown = uidropdown(configGrid, ...
                "Items", {'acceleration', 'skidpad', 'autocross', 'endurance'}, ...
                "Value", 'autocross');
            app.EventDropDown.Layout.Row = 2;
            app.EventDropDown.Layout.Column = 2;
            app.EventDropDown.ValueChangedFcn = ...
                @(~, ~) app.synchronizeConfiguration("event");

            app.addConfigLabel(configGrid, 3, "目标圈数");
            app.LapsField = uieditfield(configGrid, "numeric", ...
                "Limits", [1, Inf], "RoundFractionalValues", "on", ...
                "Value", 1);
            app.LapsField.Layout.Row = 3;
            app.LapsField.Layout.Column = 2;

            app.addConfigLabel(configGrid, 4, "采样距离 (m)");
            app.SampleDistanceField = uieditfield(configGrid, "numeric", ...
                "Limits", [0.01, Inf], "Value", 0.5);
            app.SampleDistanceField.Layout.Row = 4;
            app.SampleDistanceField.Layout.Column = 2;

            app.addConfigLabel(configGrid, 5, "车辆动力学");
            app.DynamicsDropDown = uidropdown(configGrid, ...
                "Items", {'7DOF', '10DOF'}, "Value", '7DOF');
            app.DynamicsDropDown.Layout.Row = 5;
            app.DynamicsDropDown.Layout.Column = 2;
            app.DynamicsDropDown.ValueChangedFcn = ...
                @(~, ~) app.synchronizeConfiguration("dynamics");

            app.addConfigLabel(configGrid, 6, "驾驶员");
            app.DriverDropDown = uidropdown(configGrid, ...
                "Items", {'adaptive_autocross', 'reference_speed'}, ...
                "Value", 'adaptive_autocross');
            app.DriverDropDown.Layout.Row = 6;
            app.DriverDropDown.Layout.Column = 2;
            app.DriverDropDown.ValueChangedFcn = ...
                @(~, ~) app.synchronizeConfiguration("driver");

            app.addConfigLabel(configGrid, 7, "轮胎模型");
            app.TireDropDown = uidropdown(configGrid, ...
                "Items", {'mf62', 'ttc_map'}, "Value", 'mf62');
            app.TireDropDown.Layout.Row = 7;
            app.TireDropDown.Layout.Column = 2;

            app.addConfigLabel(configGrid, 8, "求解精度");
            app.SolverDropDown = uidropdown(configGrid, ...
                "Items", {'fast', 'standard'}, "Value", 'standard');
            app.SolverDropDown.Layout.Row = 8;
            app.SolverDropDown.Layout.Column = 2;

            app.addConfigLabel(configGrid, 9, "停止时间 (s)");
            app.StopTimeField = uieditfield(configGrid, "numeric", ...
                "Limits", [0.1, Inf], "Value", 90.0);
            app.StopTimeField.Layout.Row = 9;
            app.StopTimeField.Layout.Column = 2;

            app.addConfigLabel(configGrid, 10, "运行名称");
            app.RunNameField = uieditfield(configGrid, "text", ...
                "Value", 'gui');
            app.RunNameField.Layout.Row = 10;
            app.RunNameField.Layout.Column = 2;

            app.SaveFigureCheckBox = uicheckbox(configGrid, ...
                "Text", "保存最终轨迹 FIG/PNG", "Value", true);
            app.SaveFigureCheckBox.Layout.Row = 11;
            app.SaveFigureCheckBox.Layout.Column = [1, 2];

            app.RunButton = uibutton(configGrid, "push", ...
                "Text", "运行圈速仿真", ...
                "FontWeight", "bold", ...
                "FontColor", [1, 1, 1], ...
                "BackgroundColor", [0.00, 0.42, 0.72], ...
                "ButtonPushedFcn", @(~, ~) app.runSimulation());
            app.RunButton.Layout.Row = 12;
            app.RunButton.Layout.Column = [1, 2];

            buttonGrid = uigridlayout(configGrid, [1, 2]);
            buttonGrid.Layout.Row = 13;
            buttonGrid.Layout.Column = [1, 2];
            buttonGrid.ColumnWidth = {'1x', '1x'};
            buttonGrid.Padding = [0, 0, 0, 0];
            app.RefreshButton = uibutton(buttonGrid, "push", ...
                "Text", "刷新结果", ...
                "ButtonPushedFcn", @(~, ~) app.refreshResults());
            app.BrowseButton = uibutton(buttonGrid, "push", ...
                "Text", "打开其他 MAT…", ...
                "ButtonPushedFcn", @(~, ~) app.browseResult());

            app.StatusLabel = uilabel(configGrid, ...
                "Text", "正在初始化…", ...
                "WordWrap", "on", ...
                "FontColor", [0.18, 0.24, 0.31]);
            app.StatusLabel.Layout.Row = 14;
            app.StatusLabel.Layout.Column = [1, 2];


            app.createResultArea(rootGrid);
            app.createQuasiStaticModule(app.QuasiStaticModuleTab);
            app.createVehicleParameterModule(app.VehicleParameterModuleTab);
        end

        function createVehicleParameterModule(app, parentTab)
            rootGrid = uigridlayout(parentTab, [5, 1]);
            rootGrid.RowHeight = {54, 42, '1x', 104, 44};
            rootGrid.Padding = [14, 12, 14, 12];
            rootGrid.RowSpacing = 8;

            titleGrid = uigridlayout(rootGrid, [1, 2]);
            titleGrid.Layout.Row = 1;
            titleGrid.ColumnWidth = {'1x', 420};
            titleGrid.Padding = [0, 0, 0, 0];
            titleLabel = uilabel(titleGrid, ...
                "Text", "车辆参数调整", ...
                "FontSize", 20, ...
                "FontWeight", "bold", ...
                "FontColor", [0.02, 0.25, 0.48]);
            titleLabel.Layout.Column = 1;
            hintLabel = uilabel(titleGrid, ...
                "Text", "修改先保留在界面中，点击“保存到数据字典”后才会生效。", ...
                "HorizontalAlignment", "right", ...
                "FontColor", [0.38, 0.42, 0.47]);
            hintLabel.Layout.Column = 2;

            filterGrid = uigridlayout(rootGrid, [1, 5]);
            filterGrid.Layout.Row = 2;
            filterGrid.ColumnWidth = {72, 210, 72, '1x', 170};
            filterGrid.Padding = [0, 0, 0, 0];
            uilabel(filterGrid, "Text", "参数分类", ...
                "HorizontalAlignment", "right");
            app.VehicleParameterGroupDropDown = uidropdown(filterGrid, ...
                "Items", {'全部参数'}, ...
                "ItemsData", {'all'}, ...
                "Value", 'all', ...
                "ValueChangedFcn", ...
                @(~, ~) app.refreshVehicleParameterTable());
            uilabel(filterGrid, "Text", "搜索", ...
                "HorizontalAlignment", "right");
            app.VehicleParameterSearchField = uieditfield(filterGrid, ...
                "text", ...
                "Placeholder", "参数名、字段路径、来源或说明", ...
                "ValueChangedFcn", ...
                @(~, ~) app.refreshVehicleParameterTable());
            app.VehicleParameterCountLabel = uilabel(filterGrid, ...
                "Text", "正在读取参数…", ...
                "HorizontalAlignment", "right", ...
                "FontColor", [0.10, 0.36, 0.56]);

            app.VehicleParameterTable = uitable(rootGrid, ...
                "ColumnName", {'分类', '参数名', '字段路径', '当前值', ...
                    '单位', '下限', '上限', '来源', '置信度', '占位', '说明'}, ...
                "ColumnEditable", [false, false, false, true, false, ...
                    false, false, false, false, false, false], ...
                "ColumnWidth", {'auto', 'auto', 'auto', 'auto', ...
                    'auto', 'auto', 'auto', 'auto', 'auto', 'auto', ...
                    '1x'}, ...
                "CellEditCallback", ...
                @(~, event) app.editVehicleParameter(event), ...
                "CellSelectionCallback", ...
                @(~, event) app.selectVehicleParameter(event));
            app.VehicleParameterTable.Layout.Row = 3;

            app.VehicleParameterDetailsArea = uitextarea(rootGrid, ...
                "Editable", "off", ...
                "Value", {'选择参数可查看来源、边界与说明。'}, ...
                "FontName", "Consolas");
            app.VehicleParameterDetailsArea.Layout.Row = 4;

            actionGrid = uigridlayout(rootGrid, [1, 4]);
            actionGrid.Layout.Row = 5;
            actionGrid.ColumnWidth = {'1x', 150, 150, 190};
            actionGrid.Padding = [0, 0, 0, 0];
            app.VehicleParameterStatusLabel = uilabel(actionGrid, ...
                "Text", "正在读取 VehicleData.sldd…", ...
                "FontColor", [0.18, 0.24, 0.31]);
            app.VehicleParameterRefreshButton = uibutton(actionGrid, ...
                "push", ...
                "Text", "重新读取字典", ...
                "ButtonPushedFcn", ...
                @(~, ~) app.loadVehicleParameterCatalog());
            app.VehicleParameterRevertButton = uibutton(actionGrid, ...
                "push", ...
                "Text", "撤销未保存修改", ...
                "Enable", "off", ...
                "ButtonPushedFcn", ...
                @(~, ~) app.revertVehicleParameterEdits());
            app.VehicleParameterSaveButton = uibutton(actionGrid, ...
                "push", ...
                "Text", "保存到数据字典", ...
                "Enable", "off", ...
                "FontWeight", "bold", ...
                "FontColor", [1, 1, 1], ...
                "BackgroundColor", [0.00, 0.42, 0.72], ...
                "ButtonPushedFcn", ...
                @(~, ~) app.saveVehicleParameterEdits());

            app.loadVehicleParameterCatalog();
        end

        function createQuasiStaticModule(app, parentTab)
            rootGrid = uigridlayout(parentTab, [1, 2]);
            rootGrid.ColumnWidth = {350, '1x'};
            rootGrid.RowHeight = {'1x'};
            rootGrid.Padding = [12, 12, 12, 12];
            rootGrid.ColumnSpacing = 12;

            configPanel = uipanel(rootGrid, ...
                "Title", "准静态配置", ...
                "FontWeight", "bold", ...
                "BackgroundColor", [1, 1, 1]);
            configGrid = uigridlayout(configPanel, [12, 2]);
            configGrid.ColumnWidth = {132, '1x'};
            configGrid.RowHeight = {48, 30, 30, 30, 30, 32, 38, 38, ...
                38, 38, 42, '1x'};
            configGrid.Padding = [12, 8, 12, 12];
            configGrid.RowSpacing = 8;

            titleLabel = uilabel(configGrid, ...
                "Text", "Quasi-Static", ...
                "FontSize", 21, ...
                "FontWeight", "bold", ...
                "FontColor", [0.12, 0.42, 0.24]);
            titleLabel.Layout.Row = 1;
            titleLabel.Layout.Column = [1, 2];

            app.addConfigLabel(configGrid, 2, "赛事");
            app.QuasiEventDropDown = uidropdown(configGrid, ...
                "Items", {'acceleration', 'skidpad', 'autocross'}, ...
                "Value", 'autocross');
            app.QuasiEventDropDown.Layout.Row = 2;
            app.QuasiEventDropDown.Layout.Column = 2;

            app.addConfigLabel(configGrid, 3, "GGV 文件");
            ggvFileGrid = uigridlayout(configGrid, [1, 2]);
            ggvFileGrid.Layout.Row = 3;
            ggvFileGrid.Layout.Column = 2;
            ggvFileGrid.ColumnWidth = {'1x', 84};
            ggvFileGrid.ColumnSpacing = 6;
            ggvFileGrid.Padding = [0, 0, 0, 0];
            app.QuasiGGVFileField = uieditfield(ggvFileGrid, "text", ...
                "Value", '', ...
                "Placeholder", "留空自动选择最新 GGV");
            app.QuasiGGVFileField.Layout.Column = 1;
            app.QuasiGGVBrowseButton = uibutton(ggvFileGrid, "push", ...
                "Text", "打开文件夹", ...
                "ButtonPushedFcn", ...
                @(~, ~) app.browseQuasiStaticGGV());
            app.QuasiGGVBrowseButton.Layout.Column = 2;

            app.addConfigLabel(configGrid, 4, "传播次数");
            app.QuasiPassCountField = uieditfield(configGrid, "numeric", ...
                "Limits", [1, Inf], ...
                "RoundFractionalValues", "on", ...
                "Value", 4);
            app.QuasiPassCountField.Layout.Row = 4;
            app.QuasiPassCountField.Layout.Column = 2;

            app.addConfigLabel(configGrid, 5, "最高速度 (m/s)");
            app.QuasiMaximumSpeedField = uieditfield( ...
                configGrid, "numeric", ...
                "Limits", [0.1, Inf], ...
                "Value", Inf);
            app.QuasiMaximumSpeedField.Layout.Row = 5;
            app.QuasiMaximumSpeedField.Layout.Column = 2;

            app.QuasiSaveCheckBox = uicheckbox(configGrid, ...
                "Text", "保存 MAT 和 FIG", "Value", true);
            app.QuasiSaveCheckBox.Layout.Row = 6;
            app.QuasiSaveCheckBox.Layout.Column = [1, 2];

            app.QuasiRunButton = uibutton(configGrid, "push", ...
                "Text", "运行准静态圈速", ...
                "FontWeight", "bold", ...
                "FontColor", [1, 1, 1], ...
                "BackgroundColor", [0.10, 0.50, 0.28], ...
                "ButtonPushedFcn", @(~, ~) app.runQuasiStaticLapTime());
            app.QuasiRunButton.Layout.Row = 7;
            app.QuasiRunButton.Layout.Column = [1, 2];

            app.QuasiRefreshButton = uibutton(configGrid, "push", ...
                "Text", "刷新准静态结果", ...
                "ButtonPushedFcn", ...
                @(~, ~) app.refreshQuasiStaticResults());
            app.QuasiRefreshButton.Layout.Row = 8;
            app.QuasiRefreshButton.Layout.Column = [1, 2];

            app.QuasiGGVGenerateButton = uibutton(configGrid, "push", ...
                "Text", "生成 GGV", ...
                "FontWeight", "bold", ...
                "ButtonPushedFcn", ...
                @(~, ~) app.generateQuasiStaticGGV());
            app.QuasiGGVGenerateButton.Layout.Row = 9;
            app.QuasiGGVGenerateButton.Layout.Column = [1, 2];

            app.QuasiOpenLatestGGVButton = uibutton(configGrid, "push", ...
                "Text", "打开最新 GGV 图窗", ...
                "ButtonPushedFcn", ...
                @(~, ~) app.openLatestQuasiStaticFigure("ggv"));
            app.QuasiOpenLatestGGVButton.Layout.Row = 10;
            app.QuasiOpenLatestGGVButton.Layout.Column = [1, 2];

            app.QuasiStatusLabel = uilabel(configGrid, ...
                "Text", "正在初始化…", ...
                "WordWrap", "on", ...
                "FontColor", [0.18, 0.24, 0.31]);
            app.QuasiStatusLabel.Layout.Row = 11;
            app.QuasiStatusLabel.Layout.Column = [1, 2];

            noteLabel = uilabel(configGrid, ...
                "Text", "准静态模块使用已保存 GGV 计算圈速；" + ...
                newline + "GGV 生成读取当前 VehicleData.sldd 参数，参数扫描位于右侧标签页。", ...
                "WordWrap", "on", ...
                "VerticalAlignment", "top", ...
                "FontColor", [0.42, 0.45, 0.50]);
            noteLabel.Layout.Row = 12;
            noteLabel.Layout.Column = [1, 2];

            app.QuasiTabGroup = uitabgroup(rootGrid);
            app.QuasiTabGroup.Layout.Column = 2;
            app.QuasiOverviewTab = uitab(app.QuasiTabGroup, ...
                "Title", "结果摘要");
            summaryGrid = uigridlayout(app.QuasiOverviewTab, [1, 1]);
            summaryGrid.Padding = [10, 10, 10, 10];
            app.QuasiSummaryTable = uitable(summaryGrid, ...
                "ColumnName", {'指标', '值'}, ...
                "ColumnEditable", [false, false], ...
                "ColumnWidth", {'auto', '1x'}, ...
                "Data", app.emptyQuasiSummaryData());

            trackTab = uitab(app.QuasiTabGroup, "Title", "轨迹预览");
            trackGrid = uigridlayout(trackTab, [1, 1]);
            trackGrid.Padding = [8, 8, 8, 8];
            app.QuasiTrackAxes = uiaxes(trackGrid);
            title(app.QuasiTrackAxes, "尚未加载准静态结果");
            xlabel(app.QuasiTrackAxes, "Global X (m)");
            ylabel(app.QuasiTrackAxes, "Global Y (m)");
            grid(app.QuasiTrackAxes, "on");

            resultsTab = uitab(app.QuasiTabGroup, "Title", "历史结果");
            resultsGrid = uigridlayout(resultsTab, [2, 1]);
            resultsGrid.RowHeight = {'1x', 40};
            resultsGrid.Padding = [8, 8, 8, 8];
            app.QuasiResultsTable = uitable(resultsGrid, ...
                "ColumnName", {'赛事', '运行', '修改时间'}, ...
                "ColumnEditable", false, ...
                "ColumnWidth", {'auto', 'auto', '1x'}, ...
                "CellSelectionCallback", ...
                @(~, event) app.selectQuasiStaticResultRow(event));
            resultButtons = uigridlayout(resultsGrid, [1, 3]);
            resultButtons.Layout.Row = 2;
            resultButtons.ColumnWidth = {'1x', '1x', '1x'};
            resultButtons.Padding = [0, 0, 0, 0];
            app.QuasiLoadButton = uibutton(resultButtons, "push", ...
                "Text", "打开所选结果", ...
                "ButtonPushedFcn", ...
                @(~, ~) app.loadSelectedQuasiStaticResult());
            uibutton(resultButtons, "push", ...
                "Text", "打开结果目录", ...
                "ButtonPushedFcn", ...
                @(~, ~) app.openQuasiStaticResultFolder());
            uibutton(resultButtons, "push", ...
                "Text", "刷新列表", ...
                "ButtonPushedFcn", ...
                @(~, ~) app.refreshQuasiStaticResults());

            app.QuasiToolsTab = uitab(app.QuasiTabGroup, ...
                "Title", "工具与分析");
            toolsGrid = uigridlayout(app.QuasiToolsTab, [5, 1]);
            toolsGrid.RowHeight = {40, 36, 38, '1x', 36};
            toolsGrid.Padding = [18, 18, 18, 18];
            toolsGrid.RowSpacing = 8;
            toolsTitle = uilabel(toolsGrid, ...
                "Text", "准静态结果分析", ...
                "FontSize", 18, "FontWeight", "bold");
            toolsTitle.Layout.Row = 1;

            analysisControls = uigridlayout(toolsGrid, [1, 6]);
            analysisControls.Layout.Row = 2;
            analysisControls.ColumnWidth = {64, 130, 58, 190, 72, '1x'};
            analysisControls.Padding = [0, 0, 0, 0];
            uilabel(analysisControls, "Text", "绘图格式", ...
                "HorizontalAlignment", "right");
            app.QuasiAnalysisPlotTypeDropDown = uidropdown( ...
                analysisControls, "Items", {'折线图', '赛道图'}, ...
                "ItemsData", {'line', 'track'}, "Value", 'line', ...
                "ValueChangedFcn", @(~, ~) ...
                    app.refreshQuasiAnalysisControls());
            uilabel(analysisControls, "Text", "X 轴", ...
                "HorizontalAlignment", "right");
            app.QuasiAnalysisXAxisDropDown = uidropdown(analysisControls, ...
                "Items", {'赛道累计距离 | SpeedProfile.ProgressS (m)', ...
                    '仿真时间 | SpeedProfile.Time (s)'}, ...
                "ItemsData", {'SpeedProfile.ProgressS', ...
                    'SpeedProfile.Time'}, ...
                "Value", 'SpeedProfile.ProgressS');
            uilabel(analysisControls, "Text", "分析参数", ...
                "HorizontalAlignment", "right");
            app.QuasiAnalysisDropDown = uidropdown(analysisControls, ...
                "Items", {'车速 | SpeedProfile.Speed (m/s)'}, ...
                "ItemsData", {'SpeedProfile.Speed'}, ...
                "Value", 'SpeedProfile.Speed');

            buttonGrid = uigridlayout(toolsGrid, [1, 2]);
            buttonGrid.Layout.Row = 3;
            buttonGrid.ColumnWidth = {'1x', '1x'};
            buttonGrid.Padding = [0, 0, 0, 0];
            app.QuasiAnalysisButton = uibutton(buttonGrid, "push", ...
                "Text", "绘制到界面", ...
                "ButtonPushedFcn", @(~, ~) app.renderQuasiAnalysis());
            uibutton(buttonGrid, "push", ...
                "Text", "打开当前准静态 FIG", ...
                "ButtonPushedFcn", ...
                @(~, ~) app.openCurrentQuasiStaticFigure());

            app.QuasiAnalysisTabGroup = uitabgroup(toolsGrid);
            app.QuasiAnalysisTabGroup.Layout.Row = 4;
            initialTab = uitab(app.QuasiAnalysisTabGroup, "Title", "分析图");
            initialGrid = uigridlayout(initialTab, [1, 1]);
            initialGrid.RowHeight = {'1x'};
            initialGrid.ColumnWidth = {'1x'};
            initialGrid.Padding = [8, 8, 8, 8];
            app.QuasiAnalysisAxes = uiaxes(initialGrid);
            grid(app.QuasiAnalysisAxes, "on");
            title(app.QuasiAnalysisAxes, "选择绘图格式、X 轴和分析参数");

            app.QuasiAnalysisSummaryTable = struct( ...
                "Data", {app.emptyAnalysisSummaryData()});
            app.QuasiAnalysisStatusLabel = uilabel(toolsGrid, ...
                "Text", "准静态分析尚未绘制。", ...
                "WordWrap", "on", ...
                "VerticalAlignment", "top", ...
                "FontColor", [0.18, 0.24, 0.31]);
            app.QuasiAnalysisStatusLabel.Layout.Row = 5;

            app.QuasiSweepTab = uitab(app.QuasiTabGroup, ...
                "Title", "参数扫描");
            sweepGrid = uigridlayout(app.QuasiSweepTab, [5, 1]);
            sweepGrid.RowHeight = {40, 42, 190, 88, '1x'};
            sweepGrid.Padding = [18, 18, 18, 18];
            sweepGrid.RowSpacing = 10;

            sweepTitle = uilabel(sweepGrid, ...
                "Text", "准静态圈时参数扫描配置", ...
                "FontSize", 18, "FontWeight", "bold");
            sweepTitle.Layout.Row = 1;

            sweepToolbar = uigridlayout(sweepGrid, [1, 6]);
            sweepToolbar.Layout.Row = 2;
            sweepToolbar.ColumnWidth = {64, 170, 72, 170, '1x', 190};
            sweepToolbar.Padding = [0, 0, 0, 0];
            eventLabel = uilabel(sweepToolbar, "Text", "赛道", ...
                "HorizontalAlignment", "right");
            eventLabel.Layout.Column = 1;
            app.QuasiSweepEventDropDown = uidropdown(sweepToolbar, ...
                "Items", {'直线加速 | acceleration', ...
                    '八字绕环 | skidpad', '高速避障 | autocross'}, ...
                "ItemsData", {'acceleration', 'skidpad', 'autocross'}, ...
                "Value", 'autocross');
            app.QuasiSweepEventDropDown.Layout.Column = 2;
            modeLabel = uilabel(sweepToolbar, "Text", "扫描模式", ...
                "HorizontalAlignment", "right");
            modeLabel.Layout.Column = 3;
            app.QuasiSweepModeDropDown = uidropdown(sweepToolbar, ...
                "Items", {'单参数', '双参数'}, ...
                "ItemsData", {'single', 'double'}, ...
                "Value", 'double', ...
                "ValueChangedFcn", @(~, ~) ...
                    app.refreshQuasiSweepControls());
            app.QuasiSweepModeDropDown.Layout.Column = 4;
            app.QuasiSweepCaseCountLabel = uilabel(sweepToolbar, ...
                "Text", "预计工况数：—", ...
                "HorizontalAlignment", "center", ...
                "FontWeight", "bold", ...
                "FontColor", [0.10, 0.36, 0.56]);
            app.QuasiSweepCaseCountLabel.Layout.Column = 5;
            app.QuasiSweepRunButton = uibutton(sweepToolbar, "push", ...
                "Text", "开始参数扫描", ...
                "FontWeight", "bold", ...
                "FontColor", [1, 1, 1], ...
                "BackgroundColor", [0.10, 0.50, 0.28], ...
                "ButtonPushedFcn", ...
                @(~, ~) app.runQuasiStaticParameterSweep());
            app.QuasiSweepRunButton.Layout.Column = 6;

            parameterGrid = uigridlayout(sweepGrid, [1, 2]);
            parameterGrid.Layout.Row = 3;
            parameterGrid.ColumnWidth = {'1x', '1x'};
            parameterGrid.Padding = [0, 0, 0, 0];
            parameterGrid.ColumnSpacing = 12;
            app.createQuasiSweepParameterPanel(parameterGrid, 1);
            app.createQuasiSweepParameterPanel(parameterGrid, 2);

            progressPanel = uipanel(sweepGrid, ...
                "Title", "运行进度", "FontWeight", "bold");
            progressPanel.Layout.Row = 4;
            progressGrid = uigridlayout(progressPanel, [2, 1]);
            progressGrid.RowHeight = {24, 30};
            progressGrid.Padding = [10, 4, 10, 8];
            progressGrid.RowSpacing = 4;
            app.QuasiSweepProgressLabel = uilabel(progressGrid, ...
                "Text", "尚未开始扫描。", ...
                "HorizontalAlignment", "center");
            app.QuasiSweepProgressGauge = uigauge(progressGrid, "linear", ...
                "Limits", [0, 100], "Value", 0);
            app.QuasiSweepProgressGauge.Layout.Row = 2;

            resultPanel = uipanel(sweepGrid, ...
                "Title", "扫描结果", "FontWeight", "bold", ...
                "BackgroundColor", [1, 1, 1]);
            resultPanel.Layout.Row = 5;
            resultGrid = uigridlayout(resultPanel, [1, 1]);
            resultGrid.RowHeight = {'1x'};
            resultGrid.ColumnWidth = {'1x'};
            resultGrid.Padding = [8, 8, 8, 8];
            app.QuasiSweepResultAxes = uiaxes(resultGrid);
            grid(app.QuasiSweepResultAxes, "on");
            title(app.QuasiSweepResultAxes, "扫描完成后在此显示结果");

            app.applyQuasiSweepParameterDefaults(1);
            app.applyQuasiSweepParameterDefaults(2);
            app.refreshQuasiSweepControls();

            logTab = uitab(app.QuasiTabGroup, "Title", "运行日志");
            logGrid = uigridlayout(logTab, [1, 1]);
            logGrid.Padding = [8, 8, 8, 8];
            app.QuasiLogTextArea = uitextarea(logGrid, ...
                "Editable", "off", ...
                "FontName", "Consolas", ...
                "Value", {'准静态模块已启动。'});
        end

        function createResultArea(app, rootGrid)
            app.TabGroup = uitabgroup(rootGrid);
            app.TabGroup.Layout.Row = 1;
            app.TabGroup.Layout.Column = 2;

            app.OverviewTab = uitab(app.TabGroup, "Title", "结果摘要");
            overviewGrid = uigridlayout(app.OverviewTab, [2, 1]);
            overviewGrid.RowHeight = {'1x', 84};
            overviewGrid.Padding = [10, 10, 10, 10];
            app.SummaryTable = uitable(overviewGrid, ...
                "ColumnName", {'指标', '值'}, ...
                "ColumnEditable", [false, false], ...
                "ColumnWidth", {'auto', '1x'}, ...
                "Data", app.emptySummaryData());
            app.SummaryTable.Layout.Row = 1;
            resultHint = uitextarea(overviewGrid, ...
                "Editable", "off", ...
                "Value", {'从“历史结果”页选择一行并加载，即可查看摘要和轨迹。'; ...
                '运行新仿真后，界面会自动加载新结果。'});
            resultHint.Layout.Row = 2;

            app.TrackTab = uitab(app.TabGroup, "Title", "轨迹预览");
            trackGrid = uigridlayout(app.TrackTab, [1, 1]);
            trackGrid.Padding = [8, 8, 8, 8];
            app.TrackAxes = uiaxes(trackGrid);
            app.TrackAxes.Layout.Row = 1;
            title(app.TrackAxes, "尚未加载结果");
            xlabel(app.TrackAxes, "Global X (m)");
            ylabel(app.TrackAxes, "Global Y (m)");
            grid(app.TrackAxes, "on");

            app.ResultsTab = uitab(app.TabGroup, "Title", "历史结果");
            resultsGrid = uigridlayout(app.ResultsTab, [2, 1]);
            resultsGrid.RowHeight = {'1x', 40};
            resultsGrid.Padding = [8, 8, 8, 8];
            app.ResultsTable = uitable(resultsGrid, ...
                "ColumnName", {'赛事', '运行', '修改时间', ...
                '赛事时间 (s)', '圈时 (s)', '状态'}, ...
                "ColumnEditable", false, ...
                "ColumnWidth", {'auto', 'auto', 'auto', 'auto', ...
                    'auto', '1x'}, ...
                "CellSelectionCallback", ...
                @(~, event) app.selectResultRow(event));
            app.ResultsTable.Layout.Row = 1;
            resultButtons = uigridlayout(resultsGrid, [1, 3]);
            resultButtons.Layout.Row = 2;
            resultButtons.ColumnWidth = {'1x', '1x', '1x'};
            resultButtons.Padding = [0, 0, 0, 0];
            app.LoadButton = uibutton(resultButtons, "push", ...
                "Text", "打开所选结果", ...
                "ButtonPushedFcn", @(~, ~) app.loadSelectedResult());
            app.OpenFolderButton = uibutton(resultButtons, "push", ...
                "Text", "打开结果目录", ...
                "ButtonPushedFcn", @(~, ~) app.openResultFolder());
            uibutton(resultButtons, "push", ...
                "Text", "刷新列表", ...
                "ButtonPushedFcn", @(~, ~) app.refreshResults());

            app.AnalysisTab = uitab(app.TabGroup, "Title", "分析绘图");
            analysisGrid = uigridlayout(app.AnalysisTab, [5, 1]);
            analysisGrid.RowHeight = {42, 36, 38, '1x', 36};
            analysisGrid.Padding = [18, 18, 18, 18];
            analysisGrid.RowSpacing = 8;
            analysisTitle = uilabel(analysisGrid, ...
                "Text", "时域闭环分析（结果直接显示在界面）", ...
                "FontSize", 18, "FontWeight", "bold");
            analysisTitle.Layout.Row = 1;

            analysisControls = uigridlayout(analysisGrid, [1, 6]);
            analysisControls.Layout.Row = 2;
            analysisControls.ColumnWidth = {64, 130, 58, 190, 72, '1x'};
            analysisControls.Padding = [0, 0, 0, 0];
            uilabel(analysisControls, "Text", "绘图格式", ...
                "HorizontalAlignment", "right");
            app.AnalysisPlotTypeDropDown = uidropdown(analysisControls, ...
                "Items", {'折线图', '赛道图'}, ...
                "ItemsData", {'line', 'track'}, "Value", 'line', ...
                "ValueChangedFcn", @(~, ~) ...
                    app.refreshTimeAnalysisControls());
            uilabel(analysisControls, "Text", "X 轴", ...
                "HorizontalAlignment", "right");
            app.AnalysisXAxisDropDown = uidropdown(analysisControls, ...
                "Items", {'实际累计距离 | Distance (m)', ...
                    '仿真时间 | Time (s)'}, ...
                "ItemsData", {'Distance', 'Time'}, "Value", 'Distance');
            uilabel(analysisControls, "Text", "分析参数", ...
                "HorizontalAlignment", "right");
            app.AnalysisDropDown = uidropdown(analysisControls, ...
                "Items", {'车速 | Vehicle.Speed (m/s)'}, ...
                "ItemsData", {'Vehicle.Speed'}, "Value", 'Vehicle.Speed');

            analysisButtonGrid = uigridlayout(analysisGrid, [1, 2]);
            analysisButtonGrid.Layout.Row = 3;
            analysisButtonGrid.ColumnWidth = {'1x', '1x'};
            analysisButtonGrid.Padding = [0, 0, 0, 0];
            app.OpenAnalysisButton = uibutton(analysisButtonGrid, "push", ...
                "Text", "绘制到界面", ...
                "ButtonPushedFcn", @(~, ~) app.renderAnalysisInApp());
            app.DetailedAnalysisButton = uibutton(analysisButtonGrid, "push", ...
                "Text", "打开独立图窗", ...
                "ButtonPushedFcn", @(~, ~) app.openAnalysis());

            app.AnalysisOutputTabGroup = uitabgroup(analysisGrid);
            app.AnalysisOutputTabGroup.Layout.Row = 4;
            initialTab = uitab(app.AnalysisOutputTabGroup, "Title", "分析图");
            initialGrid = uigridlayout(initialTab, [1, 1]);
            initialGrid.RowHeight = {'1x'};
            initialGrid.ColumnWidth = {'1x'};
            initialGrid.Padding = [8, 8, 8, 8];
            app.AnalysisAxes = uiaxes(initialGrid);
            grid(app.AnalysisAxes, "on");
            title(app.AnalysisAxes, "选择绘图格式、X 轴和分析参数");

            app.AnalysisSummaryTable = struct( ...
                "Data", {app.emptyAnalysisSummaryData()});
            app.AnalysisStatusLabel = uilabel(analysisGrid, ...
                "Text", "时域分析尚未绘制。请先在历史结果页加载结果。", ...
                "WordWrap", "on", ...
                "VerticalAlignment", "top", ...
                "FontColor", [0.18, 0.24, 0.31]);
            app.AnalysisStatusLabel.Layout.Row = 5;

            logTab = uitab(app.TabGroup, "Title", "运行日志");
            logGrid = uigridlayout(logTab, [1, 1]);
            logGrid.Padding = [8, 8, 8, 8];
            app.LogTextArea = uitextarea(logGrid, ...
                "Editable", "off", ...
                "FontName", "Consolas", ...
                "Value", {'FSAE 仿真中心已启动。'});
        end

        function runQuasiStaticLapTime(app)
            if app.IsRunning
                return
            end
            cfg = app.buildQuasiStaticConfig();
            app.IsRunning = true;
            app.setQuasiStaticBusyState(true);
            app.QuasiStatusLabel.Text = '正在运行准静态圈速…';
            app.appendQuasiLog("开始运行：" + cfg.EventName);
            drawnow;

            FSAE_GUI_QUASI_CONFIG = cfg; %#ok<NASGU>
            quasiStaticResult = struct;
            resultPath = "";
            figureHandle = [];
            scriptPath = fullfile(app.ProjectRoot, "simulation", ...
                "quasi_static", "simulation", "runQuasiStaticLapTime.m");
            try
                run(scriptPath);
                if ~isempty(figureHandle) && isgraphics(figureHandle, "figure")
                    close(figureHandle);
                end
                app.refreshQuasiStaticResults();
                if strlength(string(resultPath)) > 0 && isfile(resultPath)
                    app.loadQuasiStaticResult(resultPath);
                else
                    app.CurrentQuasiResult = quasiStaticResult;
                    app.CurrentQuasiResultFile = "";
                    app.updateQuasiStaticResult(quasiStaticResult, "");
                    app.QuasiStatusLabel.Text = '准静态圈速完成（未保存）';
                end
                app.QuasiTabGroup.SelectedTab = app.QuasiOverviewTab;
                app.appendQuasiLog("准静态圈速完成。");
            catch exception
                if ~isempty(figureHandle) && isgraphics(figureHandle, "figure")
                    close(figureHandle);
                end
                app.QuasiStatusLabel.Text = char("运行失败：" + ...
                    string(exception.message));
                app.appendQuasiLog("运行失败：" + string(exception.message));
                uialert(app.UIFigure, exception.message, "准静态圈速失败");
            end
            app.IsRunning = false;
            app.setQuasiStaticBusyState(false);
        end

        function generateQuasiStaticGGV(app)
            %GENERATEQUASISTATICGGV 使用当前 VehicleData 参数生成 GGV。
            if app.IsRunning
                return
            end
            app.IsRunning = true;
            app.setQuasiStaticBusyState(true);
            app.QuasiStatusLabel.Text = '正在生成 GGV，请稍候…';
            app.appendQuasiLog('开始生成 GGV（读取当前 VehicleData.sldd 参数）。');
            drawnow;

            figureHandle = [];
            matPath = "";
            scriptPath = fullfile(app.ProjectRoot, "simulation", ...
                "quasi_static", "simulation", "plotCurrentGGV3D.m");
            try
                run(scriptPath);
                if ~isempty(figureHandle) && isgraphics(figureHandle, "figure")
                    close(figureHandle);
                end
                if strlength(string(matPath)) > 0 && isfile(matPath)
                    app.QuasiGGVFileField.Value = char(matPath);
                    app.QuasiStatusLabel.Text = char( ...
                        "GGV 生成完成，已选择：" + string(matPath));
                    app.appendQuasiLog("GGV 生成完成，已回填文件：" + ...
                        string(matPath));
                else
                    app.QuasiStatusLabel.Text = ...
                        'GGV 生成完成，但未找到保存的 MAT 文件。';
                    app.appendQuasiLog('GGV 生成完成，但未找到保存的 MAT 文件。');
                end
            catch exception
                if ~isempty(figureHandle) && isgraphics(figureHandle, "figure")
                    close(figureHandle);
                end
                app.QuasiStatusLabel.Text = char("GGV 生成失败：" + ...
                    string(exception.message));
                app.appendQuasiLog("GGV 生成失败：" + string(exception.message));
                uialert(app.UIFigure, exception.message, "GGV 生成失败");
            end
            app.IsRunning = false;
            app.setQuasiStaticBusyState(false);
        end

        function runQuasiStaticParameterSweep(app)
            %RUNQUASISTATICPARAMETERSWEEP 从 GUI 启动参数扫描脚本。
            if app.IsRunning
                return
            end
            try
                cfg = app.buildQuasiStaticSweepConfig();
            catch exception
                app.QuasiSweepProgressLabel.Text = char( ...
                    "配置无效：" + string(exception.message));
                uialert(app.UIFigure, exception.message, "参数扫描配置无效");
                return
            end
            app.IsRunning = true;
            app.setQuasiStaticBusyState(true);
            app.QuasiTabGroup.SelectedTab = app.QuasiSweepTab;
            app.QuasiStatusLabel.Text = '正在运行参数扫描，请稍候…';
            app.appendQuasiLog(sprintf( ...
                '开始参数扫描：%s，%s，预计 %d 个工况。', ...
                cfg.ScanMode, cfg.EventName, cfg.CaseCount));
            drawnow;

            summaryFigure = [];
            FSAE_GUI_SWEEP_CONFIG = cfg; %#ok<NASGU>
            FSAE_GUI_SWEEP_PROGRESS = ...
                @(progress) app.reportQuasiStaticSweepProgress(progress); %#ok<NASGU>
            scriptPath = fullfile(app.ProjectRoot, "simulation", ...
                "quasi_static", "simulation", "runLapTimeParameterSweep.m");
            try
                run(scriptPath);
                app.displayQuasiStaticSweepResults(results, scanConfig);
                if ~isempty(summaryFigure) && isgraphics(summaryFigure, "figure")
                    close(summaryFigure);
                end
                if cfg.SaveResults
                    app.QuasiStatusLabel.Text = ...
                        '参数扫描完成，结果已显示在参数扫描页。';
                else
                    app.QuasiStatusLabel.Text = ...
                        '参数扫描完成（未保存），结果已显示在参数扫描页。';
                end
                app.appendQuasiLog('参数扫描完成。');
            catch exception
                if ~isempty(summaryFigure) && isgraphics(summaryFigure, "figure")
                    close(summaryFigure);
                end
                app.QuasiStatusLabel.Text = char("参数扫描失败：" + ...
                    string(exception.message));
                app.QuasiSweepProgressLabel.Text = char( ...
                    "扫描失败：" + string(exception.message));
                app.appendQuasiLog("参数扫描失败：" + string(exception.message));
                uialert(app.UIFigure, exception.message, "参数扫描失败");
            end
            app.IsRunning = false;
            app.setQuasiStaticBusyState(false);
        end

        function setQuasiStaticBusyState(app, isBusy)
            app.setBusyState(isBusy);
            if ~isBusy
                app.refreshQuasiSweepControls();
            end
        end

        function inventory = buildQuasiStaticResultInventory(app)
            resultRoot = fullfile(app.ProjectRoot, "results", ...
                "quasi_static", "lap_time");
            files = dir(fullfile(resultRoot, "**", "*.mat"));
            eventNames = strings(0, 1);
            runNames = strings(0, 1);
            modified = strings(0, 1);
            resultPaths = strings(0, 1);
            dates = zeros(0, 1);
            for index = 1:numel(files)
                filePath = string(fullfile(files(index).folder, ...
                    files(index).name));
                variables = whos("-file", char(filePath));
                if ~any(string({variables.name}) == "quasiStaticResult")
                    continue
                end
                relativeFolder = erase(string(files(index).folder), ...
                    string(resultRoot) + filesep);
                parts = split(relativeFolder, filesep);
                eventNames(end + 1, 1) = parts(1); %#ok<AGROW>
                [~, runName] = fileparts(filePath);
                runNames(end + 1, 1) = string(runName); %#ok<AGROW>
                timestamp = datetime(files(index).datenum, ...
                    "ConvertFrom", "datenum", ...
                    "Format", "yyyy-MM-dd HH:mm:ss");
                modified(end + 1, 1) = string(timestamp); %#ok<AGROW>
                resultPaths(end + 1, 1) = filePath; %#ok<AGROW>
                dates(end + 1, 1) = files(index).datenum; %#ok<AGROW>
            end
            [~, order] = sort(dates, "descend");
            inventory = table(eventNames(order), runNames(order), ...
                modified(order), resultPaths(order), ...
                'VariableNames', {'Event', 'Run', 'Modified', 'ResultFile'});
        end

        function selectQuasiStaticResultRow(app, event)
            if isempty(event.Indices)
                return
            end
            app.QuasiSelectedResultIndex = event.Indices(1, 1);
        end

        function loadSelectedQuasiStaticResult(app)
            if isempty(app.QuasiResultFiles) || ...
                    ~isfinite(app.QuasiSelectedResultIndex)
                uialert(app.UIFigure, "请先选择一个准静态结果。", ...
                    "未选择结果");
                return
            end
            filePath = app.QuasiResultFiles( ...
                app.QuasiSelectedResultIndex);
            startedAt = tic;
            app.setResultLoadBusyState("quasi", true);
            loadCleanup = onCleanup(@() ...
                app.setResultLoadBusyState("quasi", false));
            app.QuasiStatusLabel.Text = '正在读取准静态结果，请稍候…';
            app.appendQuasiLog("开始读取结果：" + filePath);
            drawnow;
            try
                app.loadQuasiStaticResult( ...
                    filePath);
                app.QuasiTabGroup.SelectedTab = app.QuasiOverviewTab;
                elapsed = toc(startedAt);
                app.QuasiStatusLabel.Text = char(sprintf( ...
                    '已加载（%.1f s）：%s', elapsed, filePath));
                app.appendQuasiLog(sprintf( ...
                    '结果打开完成（%.1f s）。', elapsed));
            catch exception
                app.QuasiStatusLabel.Text = char("加载失败：" + ...
                    string(exception.message));
                app.appendQuasiLog("加载失败：" + ...
                    string(exception.message));
                clear loadCleanup
                uialert(app.UIFigure, exception.message, "加载结果失败");
                return
            end
            clear loadCleanup
        end

        function setResultLoadBusyState(app, moduleName, isBusy)
            if isBusy
                state = "off";
                pointer = "watch";
            else
                state = "on";
                pointer = "arrow";
            end
            if moduleName == "quasi"
                app.QuasiLoadButton.Enable = state;
                app.QuasiResultsTable.Enable = state;
            else
                app.LoadButton.Enable = state;
                app.BrowseButton.Enable = state;
                app.OpenFolderButton.Enable = state;
                app.ResultsTable.Enable = state;
            end
            if ~isempty(app.UIFigure) && isvalid(app.UIFigure)
                app.UIFigure.Pointer = pointer;
            end
            drawnow;
        end

        function updateQuasiStaticResult(app, result, filePath)
            eventName = "unknown";
            eventTime = NaN;
            fullCourseTime = NaN;
            eventDistance = NaN;
            trackLength = NaN;
            maximumSpeed = NaN;
            if isfield(result, "Event")
                eventName = string(result.Event);
            end
            if isfield(result, "EventMetric") && ...
                    isfield(result.EventMetric, "Time")
                eventTime = double(result.EventMetric.Time);
            end
            if isfield(result, "SpeedProfile")
                profile = result.SpeedProfile;
                if isfield(profile, "LapTime")
                    fullCourseTime = double(profile.LapTime);
                end
                if isfield(profile, "TrackLength")
                    trackLength = double(profile.TrackLength);
                end
                if isfield(profile, "Speed")
                    maximumSpeed = app.maximumFinite(profile.Speed);
                end
            end
            if isfield(result, "EventMetric") && ...
                    isfield(result.EventMetric, "Distance")
                eventDistance = double(result.EventMetric.Distance);
            end
            ggvSource = "";
            if isfield(result, "GGVSourceFile")
                ggvSource = string(result.GGVSourceFile);
            end
            app.QuasiSummaryTable.Data = {
                '赛事', char(eventName);
                '赛事时间', app.numberText(eventTime, 's');
                '全程时间', app.numberText(fullCourseTime, 's');
                '赛事距离', app.numberText(eventDistance, 'm');
                '赛道长度', app.numberText(trackLength, 'm');
                '最高速度', app.numberText(maximumSpeed, 'm/s');
                'GGV 来源', char(ggvSource);
                '结果文件', char(string(filePath))};
            app.updateQuasiStaticTrackPreview(result);
        end

        function updateQuasiStaticTrackPreview(app, result)
            colorbar(app.QuasiTrackAxes, "off");
            cla(app.QuasiTrackAxes);
            if ~isfield(result, "Track") || ...
                    ~isfield(result.Track, "X") || ...
                    ~isfield(result.Track, "Y") || ...
                    ~isfield(result, "SpeedProfile") || ...
                    ~isfield(result.SpeedProfile, "Speed")
                title(app.QuasiTrackAxes, "结果中没有准静态轨迹");
                return
            end
            x = double(result.Track.X(:));
            y = double(result.Track.Y(:));
            speed = double(result.SpeedProfile.Speed(:));
            count = min([numel(x), numel(y), numel(speed)]);
            x = x(1:count);
            y = y(1:count);
            speed = speed(1:count);
            valid = isfinite(x) & isfinite(y) & isfinite(speed);
            scatter(app.QuasiTrackAxes, x(valid), y(valid), 16, ...
                speed(valid), "filled");
            speedColorbar = colorbar(app.QuasiTrackAxes);
            speedColorbar.Label.String = "Speed (m/s)";
            axis(app.QuasiTrackAxes, "equal");
            grid(app.QuasiTrackAxes, "on");
            box(app.QuasiTrackAxes, "on");
            xlabel(app.QuasiTrackAxes, "Global X (m)");
            ylabel(app.QuasiTrackAxes, "Global Y (m)");
            title(app.QuasiTrackAxes, "准静态速度分布");
        end

        function openQuasiStaticResultFolder(app)
            filePath = app.CurrentQuasiResultFile;
            if strlength(filePath) == 0 && ...
                    ~isempty(app.QuasiResultFiles) && ...
                    isfinite(app.QuasiSelectedResultIndex)
                filePath = app.QuasiResultFiles( ...
                    app.QuasiSelectedResultIndex);
            end
            if strlength(filePath) == 0
                uialert(app.UIFigure, "请先选择或加载一个准静态结果。", ...
                    "没有结果");
                return
            end
            winopen(char(fileparts(filePath)));
        end

        function browseQuasiStaticGGV(app)
            %BROWSEQUASISTATICGGV 从项目根目录选择含 GGV 的 MAT 文件。
            [fileName, folderName] = uigetfile( ...
                {'*.mat', 'GGV MATLAB file (*.mat)'}, ...
                '选择 GGV 文件', app.ProjectRoot);
            if isequal(fileName, 0)
                return
            end

            filePath = string(fullfile(folderName, fileName));
            try
                variables = whos("-file", char(filePath));
                if ~any(string({variables.name}) == "ggv")
                    uialert(app.UIFigure, ...
                        "所选 MAT 文件不包含 ggv 变量：" + filePath, ...
                        "不是 GGV 文件");
                    return
                end
                app.QuasiGGVFileField.Value = char(filePath);
                app.QuasiStatusLabel.Text = char("已选择 GGV：" + filePath);
                app.appendQuasiLog("选择 GGV 文件：" + filePath);
            catch exception
                uialert(app.UIFigure, exception.message, "读取 GGV 文件失败");
            end
        end

        function openLatestQuasiStaticFigure(app, selection)
            switch string(selection)
                case "ggv"
                    root = fullfile(app.ProjectRoot, "results", ...
                        "quasi_static", "ggv");
                case "lap_time"
                    root = fullfile(app.ProjectRoot, "results", ...
                        "quasi_static", "lap_time");
            end
            candidates = dir(fullfile(root, "**", "*.fig"));
            if isempty(candidates)
                uialert(app.UIFigure, "没有找到可打开的 FIG。", ...
                    "没有图形结果");
                return
            end
            [~, newest] = max([candidates.datenum]);
            figurePath = fullfile(candidates(newest).folder, ...
                candidates(newest).name);
            openfig(figurePath, "new", "visible");
        end

        function openCurrentQuasiStaticFigure(app)
            resultPath = app.CurrentQuasiResultFile;
            if strlength(resultPath) == 0
                uialert(app.UIFigure, ...
                    "当前准静态结果没有可关联的 MAT 文件。" + ...
                    "请先保存结果或从历史结果页加载结果。", ...
                    "当前结果没有 FIG");
                return
            end

            [resultFolder, resultStem] = fileparts(char(resultPath));
            figurePath = fullfile(resultFolder, [resultStem, '.fig']);
            if ~isfile(figurePath)
                uialert(app.UIFigure, ...
                    "当前结果对应的 FIG 不存在：" + string(figurePath), ...
                    "当前结果没有 FIG");
                return
            end

            try
                openfig(figurePath, "new", "visible");
                app.appendQuasiLog( ...
                    "打开当前准静态 FIG：" + string(figurePath));
            catch exception
                uialert(app.UIFigure, exception.message, ...
                    "打开当前准静态 FIG 失败");
            end
        end

        function createQuasiSweepParameterPanel(app, parent, index)
            catalog = app.quasiSweepParameterCatalog();
            panel = uipanel(parent, ...
                "Title", sprintf("参数 %d", index), ...
                "FontWeight", "bold", ...
                "BackgroundColor", [1, 1, 1]);
            panel.Layout.Column = index;
            gridLayout = uigridlayout(panel, [4, 3]);
            gridLayout.ColumnWidth = {'1x', '1x', '1x'};
            gridLayout.RowHeight = {34, 22, 34, '1x'};
            gridLayout.Padding = [10, 8, 10, 8];
            gridLayout.RowSpacing = 4;
            gridLayout.ColumnSpacing = 8;

            parameterDropDown = uidropdown(gridLayout, ...
                "Items", cellstr(catalog.Label), ...
                "ItemsData", cellstr(catalog.Path), ...
                "Value", char(catalog.Path(index)), ...
                "ValueChangedFcn", @(~, ~) ...
                    app.applyQuasiSweepParameterDefaults(index));
            parameterDropDown.Layout.Row = 1;
            parameterDropDown.Layout.Column = [1, 3];

            pointCountLabel = uilabel(gridLayout, ...
                "Text", "点数：—", ...
                "HorizontalAlignment", "center", ...
                "FontWeight", "bold", ...
                "FontColor", [0.10, 0.36, 0.56]);
            pointCountLabel.Layout.Row = 4;
            pointCountLabel.Layout.Column = 3;

            headers = ["起点", "终点", "步长"];
            for column = 1:3
                header = uilabel(gridLayout, ...
                    "Text", headers(column), ...
                    "HorizontalAlignment", "center", ...
                    "FontColor", [0.36, 0.40, 0.46]);
                header.Layout.Row = 2;
                header.Layout.Column = column;
            end

            startField = uieditfield(gridLayout, "numeric", ...
                "ValueChangedFcn", @(~, ~) ...
                    app.refreshQuasiSweepControls());
            startField.Layout.Row = 3;
            startField.Layout.Column = 1;
            stopField = uieditfield(gridLayout, "numeric", ...
                "ValueChangedFcn", @(~, ~) ...
                    app.refreshQuasiSweepControls());
            stopField.Layout.Row = 3;
            stopField.Layout.Column = 2;
            stepField = uieditfield(gridLayout, "numeric", ...
                "Limits", [eps, Inf], ...
                "ValueChangedFcn", @(~, ~) ...
                    app.refreshQuasiSweepControls());
            stepField.Layout.Row = 3;
            stepField.Layout.Column = 3;
            unitLabel = uilabel(gridLayout, ...
                "Text", "单位：—", ...
                "HorizontalAlignment", "center", ...
                "FontColor", [0.26, 0.31, 0.37]);
            unitLabel.Layout.Row = 4;
            unitLabel.Layout.Column = [1, 2];

            if index == 1
                app.QuasiSweepParameter1DropDown = parameterDropDown;
                app.QuasiSweepParameter1StartField = startField;
                app.QuasiSweepParameter1StopField = stopField;
                app.QuasiSweepParameter1StepField = stepField;
                app.QuasiSweepParameter1UnitLabel = unitLabel;
                app.QuasiSweepParameter1PointCountLabel = pointCountLabel;
            else
                app.QuasiSweepParameter2DropDown = parameterDropDown;
                app.QuasiSweepParameter2StartField = startField;
                app.QuasiSweepParameter2StopField = stopField;
                app.QuasiSweepParameter2StepField = stepField;
                app.QuasiSweepParameter2UnitLabel = unitLabel;
                app.QuasiSweepParameter2PointCountLabel = pointCountLabel;
            end
        end

        function applyQuasiSweepParameterDefaults(app, index)
            catalog = app.quasiSweepParameterCatalog();
            if index == 1
                path = string(app.QuasiSweepParameter1DropDown.Value);
            else
                path = string(app.QuasiSweepParameter2DropDown.Value);
            end
            row = find(catalog.Path == path, 1);
            assert(~isempty(row), "FSAE:App:UnknownSweepParameter", ...
                "未知参数扫描字段：%s", path);

            if index == 1
                app.QuasiSweepParameter1StartField.Value = ...
                    catalog.Start(row);
                app.QuasiSweepParameter1StopField.Value = catalog.Stop(row);
                app.QuasiSweepParameter1StepField.Value = catalog.Step(row);
                app.QuasiSweepParameter1UnitLabel.Text = ...
                    char("单位：" + catalog.Unit(row));
            else
                app.QuasiSweepParameter2StartField.Value = ...
                    catalog.Start(row);
                app.QuasiSweepParameter2StopField.Value = catalog.Stop(row);
                app.QuasiSweepParameter2StepField.Value = catalog.Step(row);
                app.QuasiSweepParameter2UnitLabel.Text = ...
                    char("单位：" + catalog.Unit(row));
            end
            app.refreshQuasiSweepControls();
        end

        function refreshQuasiSweepControls(app)
            isDouble = string(app.QuasiSweepModeDropDown.Value) == "double";
            if isDouble
                secondState = "on";
            else
                secondState = "off";
            end
            secondControls = [app.QuasiSweepParameter2DropDown, ...
                app.QuasiSweepParameter2StartField, ...
                app.QuasiSweepParameter2StopField, ...
                app.QuasiSweepParameter2StepField];
            for control = reshape(secondControls, 1, [])
                control.Enable = secondState;
            end

            try
                cfg = app.buildQuasiStaticSweepConfig();
                values1 = createSweepValues(cfg.Parameter1Start, ...
                    cfg.Parameter1Stop, cfg.Parameter1Step);
                app.QuasiSweepParameter1PointCountLabel.Text = ...
                    char(sprintf("点数：%d", numel(values1)));
                if isDouble
                    values2 = createSweepValues(cfg.Parameter2Start, ...
                        cfg.Parameter2Stop, cfg.Parameter2Step);
                    app.QuasiSweepParameter2PointCountLabel.Text = ...
                        char(sprintf("点数：%d", numel(values2)));
                    app.QuasiSweepParameter2PointCountLabel.FontColor = ...
                        [0.10, 0.36, 0.56];
                else
                    app.QuasiSweepParameter2PointCountLabel.Text = ...
                        '点数：未启用';
                    app.QuasiSweepParameter2PointCountLabel.FontColor = ...
                        [0.48, 0.51, 0.56];
                end
                app.QuasiSweepCaseCountLabel.Text = char(sprintf( ...
                    "预计工况数：%d", cfg.CaseCount));
                app.QuasiSweepCaseCountLabel.FontColor = [0.10, 0.36, 0.56];
                if ~app.IsRunning
                    app.QuasiSweepRunButton.Enable = "on";
                end
            catch exception
                app.QuasiSweepParameter1PointCountLabel.Text = '点数：—';
                app.QuasiSweepParameter2PointCountLabel.Text = '点数：—';
                app.QuasiSweepCaseCountLabel.Text = char( ...
                    "配置无效：" + string(exception.message));
                app.QuasiSweepCaseCountLabel.FontColor = [0.72, 0.16, 0.12];
                app.QuasiSweepRunButton.Enable = "off";
            end
        end

        function catalog = quasiSweepParameterCatalog(app)
            if isempty(app.QuasiSweepCatalog)
                app.QuasiSweepCatalog = listQuasiStaticSweepParameters( ...
                    app.ProjectRoot);
            end
            catalog = app.QuasiSweepCatalog;
        end

        function loadVehicleParameterCatalog(app)
            catalog = listVehicleParameters(app.ProjectRoot);
            app.VehicleParameterCatalog = catalog;
            app.OriginalVehicleParameterCatalog = catalog;

            [groups, firstRows] = unique(catalog.Group, "stable");
            groupItems = ["全部参数"; ...
                catalog.GroupLabel(firstRows) + " | " + groups];
            groupData = ["all"; groups];
            app.VehicleParameterGroupDropDown.Items = cellstr(groupItems);
            app.VehicleParameterGroupDropDown.ItemsData = cellstr(groupData);
            app.VehicleParameterGroupDropDown.Value = 'all';
            app.VehicleParameterSearchField.Value = '';
            app.refreshVehicleParameterTable();
            app.VehicleParameterStatusLabel.Text = char(sprintf( ...
                "已从 VehicleData.sldd 读取 %d 个可调整标量参数。", ...
                height(catalog)));
            app.VehicleParameterStatusLabel.FontColor = [0.10, 0.36, 0.56];
            app.updateVehicleParameterDirtyState();
        end

        function refreshVehicleParameterTable(app)
            catalog = app.VehicleParameterCatalog;
            if isempty(catalog)
                app.VehicleParameterTable.Data = table();
                app.VehicleParameterCountLabel.Text = '显示 0 项';
                return
            end

            selectedGroup = string( ...
                app.VehicleParameterGroupDropDown.Value);
            mask = true(height(catalog), 1);
            if selectedGroup ~= "all"
                mask = mask & catalog.Group == selectedGroup;
            end

            query = lower(strtrim(string( ...
                app.VehicleParameterSearchField.Value)));
            if strlength(query) > 0
                searchableText = lower(catalog.GroupLabel + " " + ...
                    catalog.Name + " " + catalog.Path + " " + ...
                    catalog.Source + " " + catalog.Notes);
                mask = mask & contains(searchableText, query);
            end

            app.VehicleParameterTable.Data = catalog(mask, ...
                ["GroupLabel", "Name", "Path", "Value", "Unit", ...
                "LowerBound", "UpperBound", "Source", "Confidence", ...
                "IsPlaceholder", "Notes"]);
            app.VehicleParameterCountLabel.Text = char(sprintf( ...
                "显示 %d / %d 项", nnz(mask), height(catalog)));
        end

        function editVehicleParameter(app, event)
            if isempty(event.Indices) || event.Indices(2) ~= 4
                return
            end
            view = app.VehicleParameterTable.Data;
            path = string(view.Path(event.Indices(1)));
            catalogRowIndex = find( ...
                app.VehicleParameterCatalog.Path == path, 1);
            catalogRow = app.VehicleParameterCatalog(catalogRowIndex, :);
            try
                app.validateVehicleParameterValue(catalogRow, event.NewData);
            catch exception
                app.refreshVehicleParameterTable();
                app.VehicleParameterStatusLabel.Text = char( ...
                    "修改无效：" + string(exception.message));
                app.VehicleParameterStatusLabel.FontColor = [0.72, 0.16, 0.12];
                return
            end

            app.VehicleParameterCatalog.Value(catalogRowIndex) = ...
                double(event.NewData);
            app.refreshVehicleParameterTable();
            app.updateVehicleParameterDirtyState();
        end

        function selectVehicleParameter(app, event)
            if isempty(event.Indices) || ...
                    isempty(app.VehicleParameterTable.Data)
                return
            end
            row = event.Indices(1, 1);
            view = app.VehicleParameterTable.Data;
            catalogRow = app.VehicleParameterCatalog( ...
                app.VehicleParameterCatalog.Path == string(view.Path(row)), :);
            lowerText = app.vehicleParameterBoundText( ...
                catalogRow.LowerBound);
            upperText = app.vehicleParameterBoundText( ...
                catalogRow.UpperBound);
            app.VehicleParameterDetailsArea.Value = {
                char("字段：" + catalogRow.Path)
                char("当前值：" + string(catalogRow.Value) + ...
                    " " + catalogRow.Unit)
                char("允许范围：[" + lowerText + ", " + upperText + "]")
                char("来源：" + catalogRow.Source + ...
                    "；置信度：" + catalogRow.Confidence)
                char("说明：" + catalogRow.Notes)};
        end

        function revertVehicleParameterEdits(app)
            app.VehicleParameterCatalog = ...
                app.OriginalVehicleParameterCatalog;
            app.refreshVehicleParameterTable();
            app.updateVehicleParameterDirtyState();
            app.VehicleParameterStatusLabel.Text = '已撤销所有未保存修改。';
            app.VehicleParameterStatusLabel.FontColor = [0.10, 0.36, 0.56];
        end

        function saveVehicleParameterEdits(app)
            try
                changes = app.buildVehicleParameterChanges();
                if isempty(changes)
                    app.VehicleParameterStatusLabel.Text = '没有需要保存的修改。';
                    return
                end
                receipt = updateVehicleParameters(changes, ...
                    ProjectRoot = app.ProjectRoot);
                app.QuasiSweepCatalog = table();
                app.loadVehicleParameterCatalog();
                app.VehicleParameterStatusLabel.Text = char(sprintf( ...
                    "已保存 %d 项修改；备份位于 %s", ...
                    receipt.ChangedCount, receipt.BackupPath));
                app.VehicleParameterDetailsArea.Value = {
                    '参数修改已写入 VehicleData.sldd。'
                    char("备份：" + receipt.BackupPath)};
            catch exception
                app.VehicleParameterStatusLabel.Text = char( ...
                    "保存失败：" + string(exception.message));
                app.VehicleParameterStatusLabel.FontColor = [0.72, 0.16, 0.12];
                uialert(app.UIFigure, exception.message, "车辆参数保存失败");
            end
        end

        function updateVehicleParameterDirtyState(app)
            changedCount = nnz(app.vehicleParameterChangeMask());
            hasChanges = changedCount > 0;
            enableState = "off";
            if hasChanges
                enableState = "on";
            end
            app.VehicleParameterSaveButton.Enable = enableState;
            app.VehicleParameterRevertButton.Enable = enableState;
            if hasChanges
                app.VehicleParameterStatusLabel.Text = char(sprintf( ...
                    "已修改 %d 项，尚未写入数据字典。", changedCount));
                app.VehicleParameterStatusLabel.FontColor = [0.78, 0.42, 0.06];
            end
        end

        function mask = vehicleParameterChangeMask(app)
            if isempty(app.VehicleParameterCatalog) || ...
                    isempty(app.OriginalVehicleParameterCatalog)
                mask = false(0, 1);
                return
            end
            currentValues = app.VehicleParameterCatalog.Value;
            originalValues = app.OriginalVehicleParameterCatalog.Value;
            mask = ~(currentValues == originalValues | ...
                (isnan(currentValues) & isnan(originalValues)));
        end

        function validateVehicleParameterValue(~, catalogRow, value)
            assert(isnumeric(value) && isreal(value) && isscalar(value) && ...
                isfinite(value), "FSAE:VehicleParameters:NonFiniteValue", ...
                "%s 必须是有限实数。", catalogRow.Path);
            assert(~isfinite(catalogRow.LowerBound) || ...
                value >= catalogRow.LowerBound, ...
                "FSAE:VehicleParameters:BelowLowerBound", ...
                "%s 不能小于下限 %.6g。", ...
                catalogRow.Path, catalogRow.LowerBound);
            assert(~isfinite(catalogRow.UpperBound) || ...
                value <= catalogRow.UpperBound, ...
                "FSAE:VehicleParameters:AboveUpperBound", ...
                "%s 不能大于上限 %.6g。", ...
                catalogRow.Path, catalogRow.UpperBound);
        end

        function textValue = vehicleParameterBoundText(~, value)
            if isfinite(value)
                textValue = string(sprintf("%.6g", value));
            else
                textValue = "未指定";
            end
        end

        function appendQuasiLog(app, message)
            timestamp = string(datetime("now", "Format", "HH:mm:ss"));
            newLine = char("[" + timestamp + "] " + string(message));
            values = app.QuasiLogTextArea.Value;
            if ischar(values)
                values = {values};
            end
            app.QuasiLogTextArea.Value = [values; {newLine}];
            drawnow limitrate;
        end

        function addConfigLabel(~, parent, row, textValue)
            label = uilabel(parent, "Text", textValue, ...
                "HorizontalAlignment", "right");
            label.Layout.Row = row;
            label.Layout.Column = 1;
        end

        function synchronizeConfiguration(app, ~)
            eventName = string(app.EventDropDown.Value);
            if eventName == "endurance"
                app.LapsField.Enable = "on";
            else
                app.LapsField.Value = 1;
                app.LapsField.Enable = "off";
            end

        end

        function runSimulation(app)
            if app.IsRunning
                return
            end

            try
                cfg = app.buildRunConfig();
            catch exception
                uialert(app.UIFigure, exception.message, "配置无效");
                return
            end

            app.IsRunning = true;
            app.setBusyState(true);
            app.StatusLabel.Text = '正在运行圈速仿真…';
            app.appendLog("开始运行：" + cfg.Track.Name + ", " + ...
                cfg.Vehicle.DynamicsModel + ", " + cfg.Driver.Model);
            drawnow;

            FSAE_GUI_RUN_CONFIG = cfg; %#ok<NASGU>
            output = struct;
            result = struct;
            scriptPath = fullfile(app.ProjectRoot, "simulation", ...
                "time_domain_closed_loop", "simulation", ...
                "runLapSimulation.m");
            try
                run(scriptPath);
                app.refreshResults();
                if isfield(output, "ResultFile") && ...
                        strlength(string(output.ResultFile)) > 0
                    app.loadResult(output.ResultFile);
                elseif isfield(result, "Meta") && ...
                        isfield(result.Meta, "ResultFile")
                    app.loadResult(result.Meta.ResultFile);
                end
                app.TabGroup.SelectedTab = app.OverviewTab;
                app.appendLog("仿真完成。");
            catch exception
                app.StatusLabel.Text = char("仿真失败：" + ...
                    string(exception.message));
                app.appendLog("仿真失败：" + string(exception.message));
                uialert(app.UIFigure, exception.message, "圈速仿真失败");
            end
            app.IsRunning = false;
            app.setBusyState(false);
        end

        function setBusyState(app, isBusy)
            if isBusy
                state = "off";
            else
                state = "on";
            end
            controlTypes = ["uibutton", "uidropdown", "uieditfield", ...
                "uicheckbox", "uitable"];
            for controlType = controlTypes
                controls = findall(app.UIFigure, "Type", controlType);
                for control = reshape(controls, 1, [])
                    control.Enable = state;
                end
            end
        end

        function inventory = buildResultInventory(app)
            resultRoot = fullfile(app.ProjectRoot, "results", ...
                "time_domain_closed_loop");
            files = dir(fullfile(resultRoot, "**", ...
                "lap_simulation_result.mat"));
            if isempty(files)
                inventory = table( ...
                    strings(0, 1), strings(0, 1), strings(0, 1), ...
                    zeros(0, 1), zeros(0, 1), strings(0, 1), ...
                    strings(0, 1), ...
                    'VariableNames', {'Event', 'Run', 'Modified', ...
                    'EventTime_s', 'LapTime_s', 'Status', 'ResultFile'});
                return
            end

            [~, order] = sort([files.datenum], "descend");
            files = files(order);
            count = numel(files);
            eventNames = strings(count, 1);
            runNames = strings(count, 1);
            modified = strings(count, 1);
            eventTimes = NaN(count, 1);
            lapTimes = NaN(count, 1);
            statuses = repmat("—", count, 1);
            resultPaths = strings(count, 1);

            for index = 1:count
                resultPaths(index) = string(fullfile( ...
                    files(index).folder, files(index).name));
                relativeFolder = erase(string(files(index).folder), ...
                    string(resultRoot) + filesep);
                parts = split(relativeFolder, filesep);
                eventNames(index) = parts(1);
                runNames(index) = parts(end);
                timestamp = datetime(files(index).datenum, ...
                    "ConvertFrom", "datenum", ...
                    "Format", "yyyy-MM-dd HH:mm:ss");
                modified(index) = string(timestamp);
                summaryPath = fullfile(files(index).folder, ...
                    "simulation_summary.json");
                [eventTime, lapTime, status] = ...
                    app.readResultSummary(summaryPath);
                eventTimes(index) = eventTime;
                lapTimes(index) = lapTime;
                statuses(index) = status;
            end

            inventory = table(eventNames, runNames, modified, eventTimes, ...
                lapTimes, statuses, resultPaths, ...
                'VariableNames', {'Event', 'Run', 'Modified', ...
                'EventTime_s', 'LapTime_s', 'Status', 'ResultFile'});
        end

        function [eventTime, lapTime, status] = ...
                readResultSummary(~, summaryPath)
            eventTime = NaN;
            lapTime = NaN;
            status = "—";
            if ~isfile(summaryPath)
                return
            end
            try
                summary = jsondecode(fileread(summaryPath));
                if isfield(summary, "Metrics")
                    if isfield(summary.Metrics, "EventTime")
                        candidate = summary.Metrics.EventTime;
                        if isnumeric(candidate) && isscalar(candidate)
                            eventTime = double(candidate);
                        end
                    end
                    if isfield(summary.Metrics, "LapTime")
                        candidate = summary.Metrics.LapTime;
                        if isnumeric(candidate) && isscalar(candidate)
                            lapTime = double(candidate);
                        end
                    end
                end
                if isfield(summary, "Valid") && ...
                        islogical(summary.Valid) && isscalar(summary.Valid)
                    if logical(summary.Valid)
                        status = "PASS";
                    else
                        status = "FAIL";
                    end
                end
            catch
                status = "JSON error";
            end
        end

        function selectResultRow(app, event)
            if isempty(event.Indices)
                return
            end
            app.SelectedResultIndex = event.Indices(1, 1);
        end

        function loadSelectedResult(app)
            if isempty(app.ResultFiles) || ...
                    ~isfinite(app.SelectedResultIndex)
                uialert(app.UIFigure, "请先选择一个结果。", "未选择结果");
                return
            end
            filePath = app.ResultFiles(app.SelectedResultIndex);
            startedAt = tic;
            app.setResultLoadBusyState("time", true);
            loadCleanup = onCleanup(@() ...
                app.setResultLoadBusyState("time", false));
            app.StatusLabel.Text = '正在读取时域结果，请稍候…';
            app.appendLog("开始读取结果：" + filePath);
            drawnow;
            try
                app.loadResult(filePath);
                app.TabGroup.SelectedTab = app.OverviewTab;
                elapsed = toc(startedAt);
                app.StatusLabel.Text = char(sprintf( ...
                    '已加载（%.1f s）：%s', elapsed, filePath));
                app.appendLog(sprintf( ...
                    '结果打开完成（%.1f s）。', elapsed));
            catch exception
                app.StatusLabel.Text = char("加载失败：" + ...
                    string(exception.message));
                app.appendLog("加载失败：" + string(exception.message));
                clear loadCleanup
                uialert(app.UIFigure, exception.message, "加载结果失败");
                return
            end
            clear loadCleanup
        end

        function browseResult(app)
            startFolder = fullfile(app.ProjectRoot, "results", ...
                "time_domain_closed_loop");
            [fileName, folderName] = uigetfile( ...
                {'*.mat', 'MATLAB result (*.mat)'}, ...
                '选择 lap_simulation_result.mat', startFolder);
            if isequal(fileName, 0)
                return
            end
            filePath = string(fullfile(folderName, fileName));
            startedAt = tic;
            app.setResultLoadBusyState("time", true);
            loadCleanup = onCleanup(@() ...
                app.setResultLoadBusyState("time", false));
            app.StatusLabel.Text = '正在读取时域结果，请稍候…';
            app.appendLog("开始读取结果：" + filePath);
            drawnow;
            try
                app.loadResult(filePath);
                app.TabGroup.SelectedTab = app.OverviewTab;
                elapsed = toc(startedAt);
                app.StatusLabel.Text = char(sprintf( ...
                    '已加载（%.1f s）：%s', elapsed, filePath));
                app.appendLog(sprintf( ...
                    '结果打开完成（%.1f s）。', elapsed));
            catch exception
                app.StatusLabel.Text = char("加载失败：" + ...
                    string(exception.message));
                app.appendLog("加载失败：" + string(exception.message));
                clear loadCleanup
                uialert(app.UIFigure, exception.message, "加载结果失败");
                return
            end
            clear loadCleanup
        end

        function openResultFolder(app)
            filePath = app.selectedOrCurrentResultFile();
            if strlength(filePath) == 0
                uialert(app.UIFigure, "请先选择或加载一个结果。", ...
                    "没有结果");
                return
            end
            winopen(char(fileparts(filePath)));
        end

        function filePath = selectedOrCurrentResultFile(app)
            filePath = app.CurrentResultFile;
            if strlength(filePath) == 0 && ~isempty(app.ResultFiles) && ...
                    isfinite(app.SelectedResultIndex)
                filePath = app.ResultFiles(app.SelectedResultIndex);
            end
        end

        function updateSummary(app, result, filePath)
            eventName = app.metaValue(result, "Event", "unknown");
            valid = app.metaValue(result, "Valid", false);
            completed = app.metaValue(result, "Completed", false);
            eventTime = app.metricValue(result, "EventTime");
            lapTime = app.metricValue(result, "LapTime");
            maximumSpeed = app.maximumFinite(result.Vehicle.Speed);
            maximumAcceleration = app.metricValue( ...
                result, "MaximumAcceleration");
            minimumClearance = app.metricValue( ...
                result, "MinimumVehicleEnvelopeClearance");
            maximumViolation = app.metricValue( ...
                result, "MaximumVehicleEnvelopeViolation");
            missingCount = numel(app.metaValue( ...
                result, "MissingSignals", strings(0, 1)));

            app.SummaryTable.Data = {
                '赛事', char(string(eventName));
                '结果有效', app.logicalText(valid);
                '完成赛项', app.logicalText(completed);
                '赛事时间', app.numberText(eventTime, 's');
                '圈时', app.numberText(lapTime, 's');
                '最高速度', app.numberText(maximumSpeed, 'm/s');
                '最大合加速度', app.numberText(maximumAcceleration, 'm/s^2');
                '最小车辆包络净空', app.numberText(minimumClearance, 'm');
                '最大车辆包络越界', app.numberText(maximumViolation, 'm');
                '缺失信号数', sprintf('%d', missingCount);
                '结果文件', char(filePath)};
        end

        function updateTrackPreview(app, result)
            colorbar(app.TrackAxes, "off");
            cla(app.TrackAxes);
            if ~isfield(result, "Vehicle") || ...
                    isempty(result.Vehicle.X) || isempty(result.Vehicle.Y)
                title(app.TrackAxes, "结果中没有车辆轨迹");
                return
            end

            x = double(result.Vehicle.X(:));
            y = double(result.Vehicle.Y(:));
            count = min(numel(x), numel(y));
            x = x(1:count);
            y = y(1:count);
            valid = isfinite(x) & isfinite(y);
            speed = [];
            if isfield(result.Vehicle, "Speed") && ...
                    numel(result.Vehicle.Speed) >= count
                speed = double(result.Vehicle.Speed(1:count));
                speed = speed(:);
                valid = valid & isfinite(speed);
            end

            hold(app.TrackAxes, "on");
            if isempty(speed)
                plot(app.TrackAxes, x(valid), y(valid), ...
                    "LineWidth", 1.4, "Color", [0.00, 0.42, 0.72]);
            else
                scatter(app.TrackAxes, x(valid), y(valid), 14, ...
                    speed(valid), "filled");
                speedColorbar = colorbar(app.TrackAxes);
                speedColorbar.Label.String = "Speed (m/s)";
            end
            first = find(valid, 1, "first");
            last = find(valid, 1, "last");
            if ~isempty(first)
                plot(app.TrackAxes, x(first), y(first), 'o', ...
                    "MarkerFaceColor", [0.00, 0.65, 0.20], ...
                    "MarkerEdgeColor", [0.00, 0.35, 0.10]);
                plot(app.TrackAxes, x(last), y(last), 's', ...
                    "MarkerFaceColor", [0.85, 0.10, 0.10], ...
                    "MarkerEdgeColor", [0.45, 0.00, 0.00]);
            end
            hold(app.TrackAxes, "off");
            axis(app.TrackAxes, "equal");
            grid(app.TrackAxes, "on");
            box(app.TrackAxes, "on");
            xlabel(app.TrackAxes, "Global X (m)");
            ylabel(app.TrackAxes, "Global Y (m)");
            title(app.TrackAxes, "车辆轨迹");
        end

        function openAnalysis(app)
            if isempty(fieldnames(app.CurrentResult))
                uialert(app.UIFigure, "请先加载一个结果。", "没有结果");
                return
            end
            figureHandle = uifigure("Name", "时域闭环分析", ...
                "Position", [120, 80, 1100, 760]);
            outputTabs = uitabgroup(figureHandle, ...
                "Position", [10, 10, 1080, 740]);
            try
                renderRacecarAnalysisTabs(outputTabs, app.CurrentResult, ...
                    "time_domain", string(app.AnalysisPlotTypeDropDown.Value), ...
                    string(app.AnalysisXAxisDropDown.Value), ...
                    string(app.AnalysisDropDown.Value));
                app.appendLog("打开独立分析图窗：" + ...
                    string(app.AnalysisDropDown.Value));
            catch exception
                delete(figureHandle);
                uialert(app.UIFigure, exception.message, "分析绘图失败");
            end
        end

        function openLegacyAnalysis(app)
            if strlength(app.CurrentResultFile) == 0
                uialert(app.UIFigure, "请先加载一个结果。", "没有结果");
                return
            end
            source = app.CurrentResultFile;
            trackName = lower(string(app.metaValue( ...
                app.CurrentResult, "Event", app.EventDropDown.Value)));
            selection = string(app.AnalysisDropDown.Value);
            try
                switch selection
                    case "Vehicle dynamics"
                        plotVehicleDynamicsDashboard(source, ...
                            Track = trackName, Visible = "on");
                    case "Path tracking"
                        plotPathTrackingPerformance(source, ...
                            Track = trackName, Visible = "on");
                    case "Tire utilization"
                        plotTireUtilization(source, ...
                            Track = trackName, Visible = "on");
                    case "Suspension loads"
                        plotSuspensionLoads(source, ...
                            Track = trackName, Visible = "on");
                    case "Energy analysis"
                        plotEnergyAnalysis(source, ...
                            Track = trackName, Visible = "on");
                    case "Control allocation"
                        plotControlAllocation(source, ...
                            Track = trackName, Visible = "on");
                    case "Lap performance"
                        plotLapPerformance(source, ...
                            Track = trackName, Visible = "on");
                    case "Motor operating map"
                        plotMotorOperatingMap(source, ...
                            Track = trackName, Visible = "on");
                end
                app.appendLog("打开分析图：" + selection);
            catch exception
                uialert(app.UIFigure, exception.message, "分析绘图失败");
            end
        end

        function renderAnalysisInApp(app)
            %RENDERANALYSISINAPP 按所选格式绘制时域结果。
            if isempty(fieldnames(app.CurrentResult))
                uialert(app.UIFigure, "请先在历史结果页加载一个结果。", ...
                    "没有结果");
                return
            end
            parameter = string(app.AnalysisDropDown.Value);
            try
                [app.AnalysisAxes, plotInfo] = ...
                    renderRacecarAnalysisTabs(app.AnalysisOutputTabGroup, ...
                    app.CurrentResult, "time_domain", ...
                    string(app.AnalysisPlotTypeDropDown.Value), ...
                    string(app.AnalysisXAxisDropDown.Value), parameter);
                app.AnalysisSummaryTable.Data = ...
                    app.analysisPlotSummary(plotInfo);
                app.AnalysisStatusLabel.Text = char( ...
                    "已在界面绘制：" + plotInfo.DisplayName);
                app.appendLog("界面分析完成：" + parameter);
            catch exception
                app.showAnalysisFailure(app.AnalysisOutputTabGroup, ...
                    "时域分析字段不可用", exception.message, "time");
                uialert(app.UIFigure, exception.message, "界面分析失败");
            end
        end

        function renderLegacyAnalysisInApp(app)
            %RENDERANALYSISINAPP 在时域分析页的 UIAxes 中绘制结果。
            if isempty(fieldnames(app.CurrentResult))
                uialert(app.UIFigure, "请先在历史结果页加载一个结果。", ...
                    "没有结果");
                return
            end
            selection = string(app.AnalysisDropDown.Value);
            cla(app.AnalysisAxes);
            axis(app.AnalysisAxes, "normal");
            colorbar(app.AnalysisAxes, "off");
            rows = app.emptyAnalysisSummaryData();
            try
                switch selection
                    case "Vehicle dynamics"
                        [x, y] = app.analysisData("Vehicle.X", "Distance");
                        [~, speed] = app.analysisData("Vehicle.Speed", "Distance");
                        count = min([numel(x), numel(y), numel(speed)]);
                        x = x(1:count); y = y(1:count); speed = speed(1:count);
                        valid = isfinite(x) & isfinite(y) & isfinite(speed);
                        scatter(app.AnalysisAxes, x(valid), y(valid), 14, ...
                            speed(valid), "filled");
                        axis(app.AnalysisAxes, "equal");
                        cb = colorbar(app.AnalysisAxes);
                        cb.Label.String = "Speed (m/s)";
                        xlabel(app.AnalysisAxes, "Global X (m)");
                        ylabel(app.AnalysisAxes, "Global Y (m)");
                        title(app.AnalysisAxes, "Vehicle dynamics — 速度着色轨迹");
                        rows = { ...
                            '样本数', sprintf('%d', nnz(valid)); ...
                            '最高速度', app.numberText(app.maximumFinite(speed), 'm/s'); ...
                            '最大纵向加速度', app.numberText( ...
                                app.maximumFinite(app.CurrentResult.Vehicle.Ax), 'm/s^2'); ...
                            '最大横向加速度', app.numberText( ...
                                app.maximumFinite(app.CurrentResult.Vehicle.Ay), 'm/s^2')};
                    case "Path tracking"
                        [x, lateral, lateralInfo] = ...
                            app.analysisData("Track.LateralError", "Distance");
                        [~, boundary] = app.analysisData( ...
                            "Track.VehicleEnvelopeClearance", "Distance");
                        count = min([numel(x), size(lateral, 1), size(boundary, 1)]);
                        x = x(1:count); lateral = lateral(1:count, :);
                        boundary = boundary(1:count, :);
                        hold(app.AnalysisAxes, "on");
                        plot(app.AnalysisAxes, x, lateral, ...
                            "LineWidth", 1.2, "DisplayName", lateralInfo.DisplayName);
                        plot(app.AnalysisAxes, x, boundary, ...
                            "LineWidth", 1.2, "DisplayName", ...
                            "Vehicle envelope clearance");
                        yline(app.AnalysisAxes, 0, "--", "HandleVisibility", "off");
                        hold(app.AnalysisAxes, "off");
                        xlabel(app.AnalysisAxes, "Distance (m)");
                        ylabel(app.AnalysisAxes, "Error / clearance (m)");
                        legend(app.AnalysisAxes, "Location", "best");
                        title(app.AnalysisAxes, "Path tracking — 路径误差与包络净空");
                        rows = { ...
                            '横向误差 RMS', app.numberText( ...
                                app.metricValue(app.CurrentResult, "LateralErrorRMS"), 'm'); ...
                            '最小车辆包络净空', app.numberText( ...
                                app.minimumFinite(boundary), 'm'); ...
                            '最大越界', app.numberText( ...
                                app.metricValue(app.CurrentResult, ...
                                "MaximumVehicleEnvelopeViolation"), 'm')};
                    case "Tire utilization"
                        [x, y, info] = app.analysisData( ...
                            "Tire.MuUtilization", "Distance");
                        plot(app.AnalysisAxes, x(1:size(y, 1)), y, "LineWidth", 1.1);
                        xlabel(app.AnalysisAxes, "Distance (m)");
                        ylabel(app.AnalysisAxes, "Mu utilization (1)");
                        legend(app.AnalysisAxes, info.ChannelNames, ...
                            "Location", "best");
                        title(app.AnalysisAxes, "Tire utilization — 四轮利用率");
                        rows = { ...
                            '最大利用率', app.numberText( ...
                                app.maximumFinite(y), '1'); ...
                            '超过 1 的采样点', sprintf('%d', nnz(y > 1)); ...
                            '通道', char(strjoin(info.ChannelNames, ", "))};
                    case "Suspension loads"
                        [x, y, info] = app.analysisData( ...
                            "Wheel.NormalLoad", "Distance");
                        plot(app.AnalysisAxes, x(1:size(y, 1)), y, "LineWidth", 1.1);
                        xlabel(app.AnalysisAxes, "Distance (m)");
                        ylabel(app.AnalysisAxes, "Normal load (N)");
                        legend(app.AnalysisAxes, info.ChannelNames, ...
                            "Location", "best");
                        title(app.AnalysisAxes, "Suspension loads — 四轮法向载荷");
                        rows = { ...
                            '最大法向载荷', app.numberText( ...
                                app.maximumFinite(y), 'N'); ...
                            '最小法向载荷', app.numberText( ...
                                app.minimumFinite(y), 'N'); ...
                            '通道', char(strjoin(info.ChannelNames, ", "))};
                    case "Energy analysis"
                        [x, power, powerInfo] = app.analysisData( ...
                            "Battery.Power", "Time");
                        [~, soc, socInfo] = app.analysisData("Battery.SOC", "Time");
                        yyaxis(app.AnalysisAxes, "left");
                        plot(app.AnalysisAxes, x(1:size(power, 1)), power, ...
                            "LineWidth", 1.2);
                        ylabel(app.AnalysisAxes, "Battery power (W)");
                        yyaxis(app.AnalysisAxes, "right");
                        plot(app.AnalysisAxes, x(1:size(soc, 1)), soc, ...
                            "LineWidth", 1.2, "Color", [0.85, 0.33, 0.10]);
                        ylabel(app.AnalysisAxes, "SOC (1)");
                        xlabel(app.AnalysisAxes, "Time (s)");
                        title(app.AnalysisAxes, "Energy analysis — 电池功率与 SOC");
                        rows = { ...
                            '电池能量消耗', app.numberText( ...
                                app.metricValue(app.CurrentResult, ...
                                "BatteryEnergyUsed"), 'J'); ...
                            '最大电池功率', app.numberText( ...
                                app.maximumFinite(power), powerInfo.Unit); ...
                            'SOC 范围', sprintf('[%.4g, %.4g]', ...
                                app.minimumFinite(soc), app.maximumFinite(soc)); ...
                            'SOC 信号', char(socInfo.DisplayName)};
                    case "Control allocation"
                        [x, desired] = app.analysisData( ...
                            "Controller.DesiredYawMoment", "Time");
                        [~, allocated] = app.analysisData( ...
                            "Controller.AllocatedYawMoment", "Time");
                        count = min([numel(x), size(desired, 1), size(allocated, 1)]);
                        hold(app.AnalysisAxes, "on");
                        plot(app.AnalysisAxes, x(1:count), desired(1:count), ...
                            "LineWidth", 1.2, "DisplayName", "Desired");
                        plot(app.AnalysisAxes, x(1:count), allocated(1:count), ...
                            "LineWidth", 1.2, "DisplayName", "Allocated");
                        hold(app.AnalysisAxes, "off");
                        xlabel(app.AnalysisAxes, "Time (s)");
                        ylabel(app.AnalysisAxes, "Yaw moment (N*m)");
                        legend(app.AnalysisAxes, "Location", "best");
                        title(app.AnalysisAxes, "Control allocation — 目标与分配横摆力矩");
                        rows = { ...
                            '最大分配误差', app.numberText( ...
                                app.maximumFinite(allocated - desired), 'N*m'); ...
                            'TV 激活采样点', sprintf('%d', ...
                                app.countTrueField("Controller.TVActive")); ...
                            '控制器饱和采样点', sprintf('%d', ...
                                app.countTrueField("Controller.ControllerSaturated"))};
                    case "Lap performance"
                        [x, speed, speedInfo] = app.analysisData( ...
                            "Vehicle.Speed", "Distance");
                        [~, reference, referenceInfo] = app.analysisData( ...
                            "Track.ReferenceSpeed", "Distance");
                        count = min([numel(x), size(speed, 1), size(reference, 1)]);
                        hold(app.AnalysisAxes, "on");
                        plot(app.AnalysisAxes, x(1:count), speed(1:count), ...
                            "LineWidth", 1.3, "DisplayName", speedInfo.DisplayName);
                        plot(app.AnalysisAxes, x(1:count), reference(1:count), ...
                            "--", "LineWidth", 1.1, "DisplayName", ...
                            referenceInfo.DisplayName);
                        hold(app.AnalysisAxes, "off");
                        xlabel(app.AnalysisAxes, "Distance (m)");
                        ylabel(app.AnalysisAxes, "Speed (m/s)");
                        legend(app.AnalysisAxes, "Location", "best");
                        title(app.AnalysisAxes, "Lap performance — 实际与参考速度");
                        rows = { ...
                            '圈时', app.numberText( ...
                                app.metricValue(app.CurrentResult, "LapTime"), 's'); ...
                            '赛事时间', app.numberText( ...
                                app.metricValue(app.CurrentResult, "EventTime"), 's'); ...
                            '最高速度', app.numberText( ...
                                app.maximumFinite(speed), speedInfo.Unit)};
                    case "Motor operating map"
                        [speed, ~, speedInfo] = app.analysisData( ...
                            "Powertrain.MotorSpeed", "Time");
                        [~, torque, torqueInfo] = app.analysisData( ...
                            "Powertrain.MotorTorqueActual", "Time");
                        count = min([size(speed, 1), size(torque, 1)]);
                        valid = isfinite(speed(1:count, :)) & ...
                            isfinite(torque(1:count, :));
                        scatter(app.AnalysisAxes, speed(1:count, :), ...
                            torque(1:count, :), 12, "filled");
                        xlabel(app.AnalysisAxes, "Motor speed (rad/s)");
                        ylabel(app.AnalysisAxes, "Motor torque (N*m)");
                        title(app.AnalysisAxes, "Motor operating map");
                        rows = { ...
                            '有效采样点', sprintf('%d', nnz(valid)); ...
                            '最高转速', app.numberText( ...
                                app.maximumFinite(speed), speedInfo.Unit); ...
                            '最大电机扭矩', app.numberText( ...
                                app.maximumFinite(torque), torqueInfo.Unit)};
                end
                grid(app.AnalysisAxes, "on");
                box(app.AnalysisAxes, "on");
                app.AnalysisSummaryTable.Data = rows;
                app.AnalysisStatusLabel.Text = char("已在界面绘制：" + selection);
                app.appendLog("界面分析完成：" + selection);
            catch exception
                cla(app.AnalysisAxes);
                title(app.AnalysisAxes, "分析字段不可用");
                app.AnalysisSummaryTable.Data = { ...
                    '状态', '分析失败'; '原因', char(exception.message)};
                app.AnalysisStatusLabel.Text = char("分析失败：" + ...
                    string(exception.message));
                uialert(app.UIFigure, exception.message, "界面分析失败");
            end
        end

        function [x, data, info] = analysisData(app, parameter, axisName)
            [x, ~] = resolveRacecarAnalysisAxis(app.CurrentResult, axisName);
            [data, info] = getLapResultSignal(app.CurrentResult, parameter);
            if isempty(data)
                error("FSAE:App:AnalysisSignal", ...
                    "结果缺少分析字段：%s。", parameter);
            end
            data = double(data);
            if isvector(data)
                data = data(:);
            end
            count = min(numel(x), size(data, 1));
            x = x(1:count);
            data = data(1:count, :);
        end

        function count = countTrueField(app, path)
            [~, data] = app.analysisData(path, "Time");
            count = nnz(data ~= 0 & isfinite(data));
        end

        function updateTimeAnalysisCatalog(app, result)
            [axisCatalog, parameterCatalog] = ...
                listRacecarAnalysisCatalog(result, "time_domain");
            app.setAnalysisDropDownCatalog(app.AnalysisXAxisDropDown, ...
                axisCatalog, "Distance");
            app.setAnalysisDropDownCatalog(app.AnalysisDropDown, ...
                parameterCatalog, "Vehicle.Speed");
            app.refreshTimeAnalysisControls();
        end

        function updateQuasiAnalysisCatalog(app, result)
            [axisCatalog, parameterCatalog] = ...
                listRacecarAnalysisCatalog(result, "quasi_static");
            app.setAnalysisDropDownCatalog(app.QuasiAnalysisXAxisDropDown, ...
                axisCatalog, "SpeedProfile.ProgressS");
            app.setAnalysisDropDownCatalog(app.QuasiAnalysisDropDown, ...
                parameterCatalog, "SpeedProfile.Speed");
            app.refreshQuasiAnalysisControls();
        end

        function refreshTimeAnalysisControls(app)
            isLine = string(app.AnalysisPlotTypeDropDown.Value) == "line";
            if isLine
                app.AnalysisXAxisDropDown.Enable = "on";
            else
                app.AnalysisXAxisDropDown.Enable = "off";
            end
        end

        function refreshQuasiAnalysisControls(app)
            isLine = string(app.QuasiAnalysisPlotTypeDropDown.Value) == "line";
            if isLine
                app.QuasiAnalysisXAxisDropDown.Enable = "on";
            else
                app.QuasiAnalysisXAxisDropDown.Enable = "off";
            end
        end

        function setAnalysisDropDownCatalog(~, component, catalog, preferred)
            assert(~isempty(catalog), "FSAE:App:EmptyAnalysisCatalog", ...
                "当前结果没有可用的分析字段。");
            current = string(component.Value);
            component.ItemsData = {};
            component.Items = cellstr(catalog.Label);
            component.ItemsData = cellstr(catalog.Path);
            if ismember(current, catalog.Path)
                component.Value = char(current);
            elseif ismember(string(preferred), catalog.Path)
                component.Value = char(string(preferred));
            else
                component.Value = char(catalog.Path(1));
            end
        end

        function rows = analysisPlotSummary(app, plotInfo)
            if plotInfo.Format == "line"
                formatName = "折线图";
                xName = plotInfo.XParameter;
            else
                formatName = "赛道图";
                xName = "Global X / Global Y";
            end
            rows = { ...
                '绘图格式', char(formatName); ...
                'X 轴', char(xName); ...
                '分析参数', char(plotInfo.Parameter); ...
                '通道', char(strjoin(string(plotInfo.ChannelNames), ", ")); ...
                '图页数', sprintf('%d', plotInfo.AxesCount); ...
                '有效样本数', char(strjoin(string(plotInfo.SampleCounts), ", ")); ...
                '最小值', app.numberText(plotInfo.Minimum, plotInfo.Unit); ...
                '最大值', app.numberText(plotInfo.Maximum, plotInfo.Unit)};
        end

        function showAnalysisFailure(app, tabGroup, titleText, message, module)
            delete(tabGroup.Children);
            tab = uitab(tabGroup, "Title", "错误");
            errorGrid = uigridlayout(tab, [1, 1]);
            errorGrid.RowHeight = {'1x'};
            errorGrid.ColumnWidth = {'1x'};
            errorGrid.Padding = [8, 8, 8, 8];
            axesHandle = uiaxes(errorGrid);
            title(axesHandle, titleText);
            axis(axesHandle, "off");
            rows = {'状态', '分析失败'; '原因', char(message)};
            if module == "time"
                app.AnalysisAxes = axesHandle;
                app.AnalysisSummaryTable.Data = rows;
                app.AnalysisStatusLabel.Text = char("分析失败：" + string(message));
            else
                app.QuasiAnalysisAxes = axesHandle;
                app.QuasiAnalysisSummaryTable.Data = rows;
                app.QuasiAnalysisStatusLabel.Text = char( ...
                    "分析失败：" + string(message));
            end
        end

        function renderQuasiAnalysis(app)
            if isempty(fieldnames(app.CurrentQuasiResult))
                uialert(app.UIFigure, "请先加载一个准静态圈速结果。", ...
                    "没有结果");
                return
            end
            parameter = string(app.QuasiAnalysisDropDown.Value);
            try
                [app.QuasiAnalysisAxes, plotInfo] = ...
                    renderRacecarAnalysisTabs(app.QuasiAnalysisTabGroup, ...
                    app.CurrentQuasiResult, "quasi_static", ...
                    string(app.QuasiAnalysisPlotTypeDropDown.Value), ...
                    string(app.QuasiAnalysisXAxisDropDown.Value), parameter);
                app.QuasiAnalysisSummaryTable.Data = ...
                    app.analysisPlotSummary(plotInfo);
                app.QuasiAnalysisStatusLabel.Text = char( ...
                    "已在界面绘制：" + plotInfo.DisplayName);
                app.appendQuasiLog("界面分析完成：" + parameter);
            catch exception
                app.showAnalysisFailure(app.QuasiAnalysisTabGroup, ...
                    "准静态分析字段不可用", exception.message, "quasi");
                uialert(app.UIFigure, exception.message, "准静态分析失败");
            end
        end

        function renderLegacyQuasiAnalysis(app)
            selection = string(app.QuasiAnalysisDropDown.Value);
            cla(app.QuasiAnalysisAxes);
            axis(app.QuasiAnalysisAxes, "normal");
            colorbar(app.QuasiAnalysisAxes, "off");
            rows = app.emptyAnalysisSummaryData();
            try
                switch selection
                    case "GGV 能力"
                        [ggv, source] = app.latestGGVData();
                        speed = double(ggv.Speed(:));
                        hold(app.QuasiAnalysisAxes, "on");
                        plot(app.QuasiAnalysisAxes, speed, ...
                            app.firstColumn(ggv.AxMax), "LineWidth", 1.2, ...
                            "DisplayName", "Ax max");
                        plot(app.QuasiAnalysisAxes, speed, ...
                            app.firstColumn(ggv.AxMin), "LineWidth", 1.2, ...
                            "DisplayName", "Ax min");
                        plot(app.QuasiAnalysisAxes, speed, ...
                            app.firstColumn(ggv.AyPositive), "LineWidth", 1.2, ...
                            "DisplayName", "Ay positive");
                        plot(app.QuasiAnalysisAxes, speed, ...
                            app.firstColumn(ggv.AyNegative), "LineWidth", 1.2, ...
                            "DisplayName", "Ay negative");
                        hold(app.QuasiAnalysisAxes, "off");
                        xlabel(app.QuasiAnalysisAxes, "Speed (m/s)");
                        ylabel(app.QuasiAnalysisAxes, "Acceleration (m/s^2)");
                        legend(app.QuasiAnalysisAxes, "Location", "best");
                        title(app.QuasiAnalysisAxes, "GGV 能力包络");
                        rows = {'来源', char(source); ...
                            '速度层数', sprintf('%d', numel(speed)); ...
                            '最大纵向加速度', app.numberText( ...
                            app.maximumFinite(ggv.AxMax), 'm/s^2'); ...
                            '最大横向加速度', app.numberText( ...
                            max([app.maximumFinite(ggv.AyPositive), ...
                            abs(app.minimumFinite(ggv.AyNegative))]), 'm/s^2')};
                    case "参数扫描圈时"
                        [scan, source] = app.latestSweepData();
                        p1 = double([scan.results.Parameter1Value]).';
                        p2 = double([scan.results.Parameter2Value]).';
                        lap = double([scan.results.LapTime]).';
                        valid = isfinite(p1) & isfinite(p2) & isfinite(lap);
                        p1Values = unique(p1(valid));
                        p2Values = unique(p2(valid));
                        gridData = NaN(numel(p1Values), numel(p2Values));
                        for k = 1:numel(lap)
                            if valid(k)
                                i1 = find(p1Values == p1(k), 1);
                                i2 = find(p2Values == p2(k), 1);
                                gridData(i1, i2) = lap(k);
                            end
                        end
                        imagesc(app.QuasiAnalysisAxes, p2Values, p1Values, gridData);
                        set(app.QuasiAnalysisAxes, "YDir", "normal");
                        cb = colorbar(app.QuasiAnalysisAxes);
                        cb.Label.String = "Lap time (s)";
                        xlabel(app.QuasiAnalysisAxes, char(scan.scanConfig.Parameter2));
                        ylabel(app.QuasiAnalysisAxes, char(scan.scanConfig.Parameter1));
                        title(app.QuasiAnalysisAxes, "准静态参数扫描圈时");
                        rows = {'来源', char(source); ...
                            '成功工况', sprintf('%d', nnz(valid)); ...
                            '最佳圈时', app.numberText(app.minimumFinite(lap), 's'); ...
                            '参数 1', char(scan.scanConfig.Parameter1); ...
                            '参数 2', char(scan.scanConfig.Parameter2)};
                    case "准静态速度轨迹"
                        q = app.requireQuasiResult();
                        x = double(q.Track.X(:)); y = double(q.Track.Y(:));
                        speed = double(q.SpeedProfile.Speed(:));
                        count = min([numel(x), numel(y), numel(speed)]);
                        valid = isfinite(x(1:count)) & isfinite(y(1:count)) & ...
                            isfinite(speed(1:count));
                        scatter(app.QuasiAnalysisAxes, x(1:count), y(1:count), ...
                            14, speed(1:count), "filled");
                        axis(app.QuasiAnalysisAxes, "equal");
                        cb = colorbar(app.QuasiAnalysisAxes);
                        cb.Label.String = "Speed (m/s)";
                        xlabel(app.QuasiAnalysisAxes, "Global X (m)");
                        ylabel(app.QuasiAnalysisAxes, "Global Y (m)");
                        title(app.QuasiAnalysisAxes, "准静态速度轨迹");
                        rows = {'赛事', char(string(q.Event)); ...
                            '有效点', sprintf('%d', nnz(valid)); ...
                            '圈时', app.numberText(q.SpeedProfile.LapTime, 's'); ...
                            '最高速度', app.numberText( ...
                            app.maximumFinite(speed), 'm/s')};
                    case "曲率限速"
                        q = app.requireQuasiResult();
                        s = double(q.SpeedProfile.ProgressS(:));
                        speed = double(q.SpeedProfile.Speed(:));
                        limit = double(q.SpeedProfile.CurvatureSpeedLimit(:));
                        count = min([numel(s), numel(speed), numel(limit)]);
                        hold(app.QuasiAnalysisAxes, "on");
                        plot(app.QuasiAnalysisAxes, s(1:count), speed(1:count), ...
                            "LineWidth", 1.2, "DisplayName", "Profile speed");
                        plot(app.QuasiAnalysisAxes, s(1:count), limit(1:count), ...
                            "--", "LineWidth", 1.1, "DisplayName", "Curvature limit");
                        hold(app.QuasiAnalysisAxes, "off");
                        xlabel(app.QuasiAnalysisAxes, "Track path s (m)");
                        ylabel(app.QuasiAnalysisAxes, "Speed (m/s)");
                        legend(app.QuasiAnalysisAxes, "Location", "best");
                        title(app.QuasiAnalysisAxes, "准静态曲率限速");
                        rows = {'限速最低值', app.numberText( ...
                            app.minimumFinite(limit), 'm/s'); ...
                            '剖面最高速度', app.numberText( ...
                            app.maximumFinite(speed), 'm/s'); ...
                            '有效点', sprintf('%d', nnz(isfinite(limit)))};
                end
                grid(app.QuasiAnalysisAxes, "on");
                box(app.QuasiAnalysisAxes, "on");
                app.QuasiAnalysisSummaryTable.Data = rows;
                app.QuasiAnalysisStatusLabel.Text = char("已在界面绘制：" + selection);
                app.appendQuasiLog("界面分析完成：" + selection);
            catch exception
                title(app.QuasiAnalysisAxes, "准静态分析字段不可用");
                app.QuasiAnalysisSummaryTable.Data = { ...
                    '状态', '分析失败'; '原因', char(exception.message)};
                app.QuasiAnalysisStatusLabel.Text = char("分析失败：" + ...
                    string(exception.message));
                uialert(app.UIFigure, exception.message, "准静态分析失败");
            end
        end

        function result = requireQuasiResult(app)
            if isempty(fieldnames(app.CurrentQuasiResult))
                error("FSAE:App:NoQuasiResult", ...
                    "请先在准静态历史结果页加载一个结果。");
            end
            result = app.CurrentQuasiResult;
        end

        function [ggv, source] = latestGGVData(app)
            root = fullfile(app.ProjectRoot, "results", "quasi_static", "ggv");
            files = dir(fullfile(root, "**", "*.mat"));
            if ~isempty(files)
                [~, index] = max([files.datenum]);
                source = string(fullfile(files(index).folder, files(index).name));
                loaded = load(source, "ggv");
                if isfield(loaded, "ggv")
                    ggv = loaded.ggv;
                    return
                end
            end
            if ~isempty(fieldnames(app.CurrentQuasiResult)) && ...
                    isfield(app.CurrentQuasiResult, "GGV")
                ggv = app.CurrentQuasiResult.GGV;
                source = app.CurrentQuasiResultFile;
                if strlength(source) == 0
                    source = "当前准静态结果";
                end
                return
            end
            error("FSAE:App:NoGGVResult", "没有找到 GGV MAT 结果。");
        end

        function [scan, source] = latestSweepData(app)
            root = fullfile(app.ProjectRoot, "results", ...
                "quasi_static", "parameter_sweep");
            files = dir(fullfile(root, "**", "*.mat"));
            if isempty(files)
                error("FSAE:App:NoSweepResult", ...
                    "没有找到准静态参数扫描 MAT 结果。");
            end
            [~, index] = max([files.datenum]);
            source = string(fullfile(files(index).folder, files(index).name));
            scan = load(source, "results", "scanConfig", "scanSummary");
            if ~isfield(scan, "results") || ~isfield(scan, "scanConfig")
                error("FSAE:App:SweepResult", ...
                    "参数扫描 MAT 缺少 results 或 scanConfig。");
            end
        end

        function value = firstColumn(~, data)
            data = double(data);
            if isvector(data)
                value = data(:);
            else
                value = data(:, 1);
            end
        end

        function appendLog(app, message)
            timestamp = string(datetime("now", "Format", "HH:mm:ss"));
            newLine = char("[" + timestamp + "] " + string(message));
            values = app.LogTextArea.Value;
            if ischar(values)
                values = {values};
            end
            app.LogTextArea.Value = [values; {newLine}];
            drawnow limitrate;
        end

        function addProjectPaths(app)
            folders = [
                fullfile(app.ProjectRoot, "apps")
                fullfile(app.ProjectRoot, "scripts", "ggv")
                fullfile(app.ProjectRoot, "scripts", "initialization")
                fullfile(app.ProjectRoot, "scripts", "reporting")
                fullfile(app.ProjectRoot, "scripts", "simulation")
                fullfile(app.ProjectRoot, "scripts", "track")
                fullfile(app.ProjectRoot, "simulation", ...
                    "quasi_static", "plotting")
                fullfile(app.ProjectRoot, "simulation", ...
                    "time_domain_closed_loop", "plotting")
                ];
            for folder = reshape(folders, 1, [])
                addpath(folder, "-begin");
            end
        end
    end

    methods (Static, Access = private)
        function closeFigure(figureHandle)
            %CLOSEFIGURE 关闭窗口本身，不依赖可能已失效的 app 对象。
            if isempty(figureHandle) || ~isvalid(figureHandle)
                return
            end
            figureHandle.CloseRequestFcn = [];
            delete(figureHandle);
        end

        function root = resolveProjectRoot(requestedRoot)
            root = string(requestedRoot);
            if strlength(root) == 0
                root = string(fileparts(fileparts(mfilename("fullpath"))));
            end
            assert(isfile(fullfile(root, "FSAE_Simulation.prj")), ...
                "FSAE:App:ProjectRoot", ...
                "FSAE 项目文件不存在：%s", root);
        end

        function data = emptySummaryData()
            data = {
                '状态', '尚未加载结果';
                '提示', '请在历史结果页选择结果'};
        end

        function data = emptyQuasiSummaryData()
            data = {
                '状态', '尚未加载准静态结果';
                '提示', '运行单次圈速或从历史结果中加载'};
        end

        function data = emptyAnalysisSummaryData()
            data = {
                '状态', '尚未绘制分析';
                '提示', '选择分析类型并点击绘制'};
        end

        function value = metaValue(result, fieldName, defaultValue)
            value = defaultValue;
            if isfield(result, "Meta") && ...
                    isfield(result.Meta, fieldName)
                value = result.Meta.(fieldName);
            end
        end

        function value = metricValue(result, fieldName)
            value = NaN;
            if isfield(result, "Metrics") && ...
                    isfield(result.Metrics, fieldName)
                candidate = result.Metrics.(fieldName);
                if isnumeric(candidate) && isscalar(candidate)
                    value = double(candidate);
                end
            end
        end

        function value = maximumFinite(data)
            value = NaN;
            if isempty(data)
                return
            end
            finiteData = double(data(isfinite(data)));
            if ~isempty(finiteData)
                value = max(finiteData);
            end
        end

        function value = minimumFinite(data)
            value = NaN;
            if isempty(data)
                return
            end
            finiteData = double(data(isfinite(data)));
            if ~isempty(finiteData)
                value = min(finiteData);
            end
        end

        function textValue = numberText(value, unit)
            if ~isscalar(value) || ~isfinite(value)
                textValue = '—';
            else
                textValue = sprintf('%.6g %s', value, unit);
            end
        end

        function textValue = logicalText(value)
            if islogical(value) && isscalar(value) && value
                textValue = '是';
            else
                textValue = '否';
            end
        end
    end
end
