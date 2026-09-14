function cfg = createLapSimulationConfig()
%CREATELAPSIMULATIONCONFIG 返回圈速时域仿真的规范默认配置。
%   CFG = CREATELAPSIMULATIONCONFIG() 创建与 runLapSimulation 用户配置区
%   对应的完整结构。图形界面在此基础上只覆盖用户选择的字段。

cfg.Track.Name = "autocross";
cfg.Track.SampleDistance = 0.5;
cfg.Track.NumberOfLaps = 1;
cfg.Track.ImagePath = "";
cfg.Track.UseSavedData = true;
cfg.Track.DataFolder = "";

cfg.Vehicle.TireModel = "mf62";
cfg.Vehicle.DynamicsModel = "7DOF";

cfg.SpeedPlanner.MotorSpeedUtilization = 0.95;
cfg.SpeedPlanner.SpeedStep = 3.0;
cfg.SpeedPlanner.ConstraintScale = 0.90;
cfg.SpeedPlanner.LateralPointCount = 9;
cfg.SpeedPlanner.EnvelopePointCount = 32;
cfg.SpeedPlanner.PassCount = 6;
cfg.SpeedPlanner.AllocationMode = "fast";
cfg.SpeedPlanner.MaxLateralAcceleration = 20.0;
cfg.SpeedPlanner.MaxLongitudinalAcceleration = 20.0;
cfg.SpeedPlanner.MaxBisectionIterations = 24;
cfg.SpeedPlanner.LongitudinalBracketPointCount = 25;
cfg.SpeedPlanner.MaxSearchExpansionCount = 3;
cfg.SpeedPlanner.HighCurvatureThreshold = 0.30;
cfg.SpeedPlanner.HighCurvatureSafetyFactor = 1.50;
cfg.SpeedPlanner.UseCache = true;
cfg.SpeedPlanner.ReferenceMaximumSpeed = 22.5;
cfg.SpeedPlanner.ReferenceMaximumAcceleration = 6.25;
cfg.SpeedPlanner.ReferencePlanningDeceleration = 2.50;
cfg.SpeedPlanner.ReferenceLateralAccelerationLimit = 4.10;
cfg.SpeedPlanner.ReferenceSpeedSafetyFactor = 1.00;

cfg.Driver.Model = "adaptive_autocross";
cfg.Driver.HeadingGain = 1.8;
cfg.Driver.CrossTrackGain = 4.0;
cfg.Driver.BoundaryGain = 2.0;
cfg.Driver.MaximumSteeringRate = 6.0;
cfg.Driver.MaximumSteeringAngle = 0.50;
cfg.Driver.MinimumSteeringSpeed = 1.0;
cfg.Driver.ProjectionSearchDistance = 15.0;
cfg.Driver.MinimumSteeringPreview = 1.00;
cfg.Driver.SteeringPreviewTime = 0.18;
cfg.Driver.MaximumSteeringPreview = 4.0;
cfg.Driver.LateralAccelerationLimit = 13.5;
cfg.Driver.MaximumAcceleration = 16.0;
cfg.Driver.MaximumDeceleration = 8.0;
cfg.Driver.PlanningDeceleration = 6.0;
cfg.Driver.MaximumSpeed = 32.0;
cfg.Driver.SpeedPreviewDistance = 190.0;
cfg.Driver.SpeedPreviewStep = 0.75;
cfg.Driver.CurvatureFilterHalfWindow = 2;
cfg.Driver.CurvatureSafetyFactor = 1.0;
cfg.Driver.SpeedSafetyFactor = 1.0;
cfg.Driver.SpeedKp = 4.0;
cfg.Driver.SpeedKi = 0.20;
cfg.Driver.SpeedFeedforwardGain = 0.60;
cfg.Driver.ExitSpeedFeedforwardGain = 0.60;
cfg.Driver.AntiWindupGain = 1.0;
cfg.Driver.GGVConstraintScale = 1.0;
cfg.Driver.MaximumGGVLapTimeRatio = 1.20;
cfg.Driver.RacingLineBoundaryReserve = 0.60;
cfg.Driver.RacingLineMaximumOffset = 0.40;
cfg.Driver.RacingLineOffsetVariationWeight = 0.05;
cfg.Driver.RacingLineMaximumOffsetRate = 0.02;
cfg.Driver.RacingLineLockCurvatureThreshold = 0.25;
cfg.Driver.RacingLineLockBufferDistance = 15.0;
cfg.Driver.RacingLineCurvaturePenaltyWeight = 0.020;
cfg.Driver.RacingLineStartLockDistance = 15.0;
cfg.Driver.ReferencePathCurvatureBlendStart = 0.15;
cfg.Driver.ReferencePathCurvatureBlendEnd = 0.30;
cfg.Driver.ReferencePathMaximumBlend = 0.0;
cfg.Driver.BoundaryPreviewDistance = 6.0;
cfg.Driver.TireSectionWidth = 0.1905;
cfg.Driver.BoundaryReserve = 0.50;
cfg.Driver.EmergencyBoundaryMargin = 0.25;
cfg.Driver.EnableTV = true;

cfg.Simulation.StopTime = 90.0;
cfg.Simulation.FinishCheckPeriod = 1.00;
cfg.Simulation.SolverProfile = "standard";
cfg.Simulation.SignalLogging = false;
cfg.Simulation.OutputDecimation = 1;

cfg.Visualization.Enabled = false;

cfg.Output.RunName = "gui";
cfg.Output.ResultsRoot = "";
cfg.Output.SaveRawSimulationOutput = false;
cfg.Output.SaveFinalTrackView = true;
cfg.Output.Overwrite = false;
end
