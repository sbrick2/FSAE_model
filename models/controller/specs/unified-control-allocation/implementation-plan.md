# UnifiedControl Unified Torque Allocation Implementation Plan

## Status: Complete — functional MIL baseline

**Last Updated:** 2026-08-02  
**Architecture:** [Architecture spec](architecture.md)  
**Tests:** [Test plan](test-plan.md)

## 1. Progress Summary

| Phase | Status | Deliverables |
|---|---|---|
| 0 Interface and specification | Complete | Frozen model ports, modes, parameters, tests |
| 1 Component implementation | Complete | UnifiedControl parameter installer and allocation functions |
| 2 Model integration | Complete | `TorqueAllocator.slx`, `UnifiedControlVehicleController.slx` |
| 3 Closed-loop integration | Complete | `FSAE_UnifiedControl_ClosedLoop.slx`, scenario verification |
| 4 UnifiedControl verification and documentation | Complete | Gherkin matrix, robustness report, roadmap update |

## 2. Model Hierarchy

```text
TorqueAllocator.slx
└── UnifiedAllocationLogic             # Stateflow state/mode wrapper
    └── unifiedControlUnifiedAllocationStep        # external code-generation-safe function
        ├── unifiedControlEstimateWheelConstraints
        ├── unifiedControlProjectWheelForces
        └── unifiedControlMixActuators

UnifiedControlVehicleController.slx
├── YawController                      -> YawController.slx
├── TorqueAllocator                    -> TorqueAllocator.slx
├── ActuatorCommandBus
└── ControllerDebugBus

FSAE_UnifiedControl_ClosedLoop.slx
├── UnifiedControlPathTrackingDriver               -> UnifiedControlPathTrackingDriver.slx
├── UnifiedControlVehicleController                -> UnifiedControlVehicleController.slx
├── PathTrackingSensorModel                      -> PathTrackingSensorModel.slx
└── VehiclePlant                       -> VehiclePlant10DOF.slx
```

## 3. Dependencies

| Toolbox | Use | Required |
|---|---|---|
| MATLAB | Fixed-size allocation functions | Yes |
| Simulink | Production and wrapper models | Yes |
| Stateflow | Explicit state/mode wrapper | Yes |
| Simulink Test | Persistent Gherkin tests | Yes for automated feature execution |
| Optimization Toolbox | None | No |

## 4. Build Phases

### Phase 0: Interface freeze

- Keep all project buses and wheel order `[FL FR RL RR]` unchanged.
- Freeze the 15 allocator inputs and 12 outputs in the system spec.
- Record tentative values in `UnifiedControlDesign`, never as measured vehicle data.

Checkpoint: specifications cross-check the roadmap, TorqueVectoring equations, sensor
boundary, powertrain interface, and current hydraulic-brake behavior.

### Phase 1: Components and parameters

1. Add `scripts/unified_control/installUnifiedControlData.m`.
2. Add estimator, projection, mixer, and orchestrator MATLAB functions.
3. Run Code Analyzer and direct numerical assertions for signs, dimensions,
   limits, and nonfinite inputs.

Checkpoint: pure functions satisfy the nominal and single-constraint cases.

### Phase 2: Model integration

1. Build `TorqueAllocator.slx` with `model_edit`, one scope at a time.
2. Verify with `model_read`, `model_check`, and Stateflow lint.
3. Build `UnifiedControlVehicleController.slx` without altering TorqueVectoring or Vehicle10DOF models.
4. Map existing buses to the frozen allocator ports and status outputs.

Checkpoint: both models update, simulate for 20 ms, and have no structural
error-severity findings.

### Phase 3: System integration

1. Build `FSAE_UnifiedControl_ClosedLoop.slx` by reusing the Vehicle10DOF Plant and TorqueVectoring driver/sensor.
2. Add UnifiedControl scenario creation and verification functions using
   `Simulink.SimulationInput`.
3. Run short closed-loop and selected Skidpad/Autocross cases.

Checkpoint: no numerical failure, actuator constraints remain bounded, and
control status is logged through the unchanged buses.

### Phase 4: Verification and closeout

1. Create scalar test wrapper under `tests/unified_control/` with the required
   `TEST_ONLY_NOT_FOR_PRODUCTION` description.
2. Add persistent Gherkin tests for nominal, simultaneous constraints,
   regeneration, measurement degradation, saturation recovery, and mode switch.
3. Run draft mode where data types permit, then full compilation; run decision
   coverage if available without making it a blocking requirement.
4. Update README, roadmap, parameter/source/interface documents, changelog, and
   an UnifiedControl verification report with exact pass/fail results.

## 5. Parameter Sources

Physical aliases come from the shared records. New calibrations are defined in
the system spec and installed with source, confidence, placeholder, and update
metadata. `UnifiedControlMotorTorqueRateLimit` is derived from the current motor limit and
response-time record so a future bench-data update propagates deterministically.

## 6. Verification at Every Checkpoint

1. `model_read` for topology and expressions.
2. `model_query_params` for sample time, data dictionary, port sizes, and model references.
3. `model_check` for unconnected ports/lines and Stateflow lint.
4. Code Analyzer for each new MATLAB function.
5. Component Gherkin, direct function assertions, then integrated MIL.

## 7. Definition of Done

- [x] Interfaces and modes frozen; no bus changes.
- [x] All constraints enter one allocation calculation rather than sequential overrides.
- [x] Torque allocator and controller models structurally healthy.
- [x] Unit and constraint-matrix tests pass in full compilation.
- [x] UnifiedControl closed loop runs on the Vehicle10DOF Plant without numerical failure.
- [x] Saturation recovery and feature switching meet quantitative criteria.
- [x] Documentation distinguishes verified behavior from tentative calibration.

## 8. Risks and Mitigations

| Risk | Mitigation |
|---|---|
| Approximate projected solve does not converge enough in 24 iterations | Normalize objectives, bound step from Hessian trace, assert residual/constraint quality, increase only fixed design-time count if needed |
| TC chatters around threshold | Stateful on/off hysteresis and transition test |
| Regen rate/power limit changes net braking | Friction mixer fills the exact residual brake magnitude |
| User Vehicle10DOF work is overwritten | Add new UnifiedControl assets only; do not modify dirty Vehicle10DOF `.slx` files |
| Full Gherkin vector types fail in draft harness | Use scalar wrapper and require final full-compilation run |
