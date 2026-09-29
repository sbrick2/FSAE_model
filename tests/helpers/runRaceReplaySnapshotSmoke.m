function receipt = runRaceReplaySnapshotSmoke(projectRoot)
%RUNRACEREPLAYSNAPSHOTSMOKE Exercise the new-run snapshot/save path in isolation.
arguments
    projectRoot (1, 1) string
end
addpath(fullfile(projectRoot, "scripts", "simulation"));
FSAE_GUI_RUN_CONFIG = createLapSimulationConfig();
FSAE_GUI_RUN_CONFIG.Vehicle.DynamicsModel = "7DOF";
FSAE_GUI_RUN_CONFIG.Simulation.StopTime = 0.1;
FSAE_GUI_RUN_CONFIG.Visualization.Enabled = false;
FSAE_GUI_RUN_CONFIG.Output.SaveFinalTrackView = false;
FSAE_GUI_RUN_CONFIG.Output.ResultsRoot = fullfile(projectRoot, "results", ...
    "replay_validation", "new_runs");
FSAE_GUI_RUN_CONFIG.Output.RunName = "replay_snapshot_smoke";
run(fullfile(projectRoot, "simulation", "time_domain_closed_loop", ...
    "simulation", "runLapSimulation.m"));
assert(isfield(result.Meta, "Replay"), "FSAE:Replay:SnapshotMissing", ...
    "The new simulation result must include its pre-run replay snapshot.");
saved = load(output.ResultFile, "result");
assert(isequaln(saved.result.Meta.Replay, result.Meta.Replay), ...
    "FSAE:Replay:SnapshotChanged", "Saved snapshot differs from in-memory result.");
prepared = prepareRaceReplayData(output.ResultFile);
receipt = struct("File", output.ResultFile, "Samples", numel(prepared.Time), ...
    "SnapshotVersion", result.Meta.Replay.Version, ...
    "TireModel", result.Meta.Replay.Tire.Model, "Passed", true);
end
