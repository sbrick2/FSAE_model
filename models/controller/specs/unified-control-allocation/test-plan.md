# UnifiedControl Unified Torque Allocation Test Plan

## Status: Functional MIL baseline executed; extended calibration sweeps deferred

**Last Updated:** 2026-08-02  
**Architecture:** [Architecture spec](architecture.md)

## 1. Validation Stages

1. Pure-function MATLAB assertions for estimator, optimizer, and mixer.
2. Persistent component MIL on `TorqueAllocator.slx` through a scalar test wrapper.
3. Controller-model structural/update checks.
4. System-in-loop MIL on `FSAE_UnifiedControl_ClosedLoop.slx` with the Vehicle10DOF Plant.
5. Robustness sweeps for speed, friction estimate, power, validity, and mode transitions.

SIL/PIL is deferred to Deployment because the target platform and scheduling contract
are not yet confirmed.

## 2. Component and Integrated Allocation Matrix

| ID | Constraint combination | Expected result | Acceptance |
|---|---|---|---|
| UnifiedControl-T01 | Disabled with nonzero Fx/Mz | Immediate safe zero | All actuator outputs exactly zero; mode 0 |
| UnifiedControl-T02 | Nominal drive, no TV | Balanced common force | Four motor torques within 0.05 N*m after rate ramp |
| UnifiedControl-T03 | Nominal drive + positive TV | Right torque greater than left | Allocated yaw positive and within 2% when feasible |
| UnifiedControl-T04 | TC overslip on FL + TV | FL drive upper bound reduced while other wheels retain authority | `TCActive`; FL torque below FR; all bounds satisfied |
| UnifiedControl-T05 | Motor magnitude + speed + drive-power saturation | All three limits simultaneous | `|T_i|<=limit`; no accelerating torque above speed limit; power <= limit+1 W |
| UnifiedControl-T06 | Tire capacity + lateral acceleration + yaw | Wheel force stays inside estimated friction circle | Max estimated utilization <= 1+1e-6; saturation asserted if objective infeasible |
| UnifiedControl-T07 | Regen + regen-power + friction residual | Negative motor torque plus nonnegative friction | Regen power <= limit+1 W; reconstructed brake force matches feasible allocation |
| UnifiedControl-T08 | Regen disabled | Friction-only braking | Motor torque >= 0; regen diagnostic zero; friction positive |
| UnifiedControl-T09 | SOC at/above upper guard | Regen prohibited | Same criteria as T08 |
| UnifiedControl-T10 | Energy mode + TV + drive power | Energy budget active without losing yaw sign | Power <= energy budget+1 W; `EnergyManagementActive=true` |
| UnifiedControl-T11 | Invalid FL wheel speed | Degraded drive inhibit | FL positive torque zero; mode 3 |
| UnifiedControl-T12 | Powertrain invalid | Degraded status and conservative actuation | Mode 3; outputs finite and within limits |
| UnifiedControl-T13 | TC on/off hysteresis | No threshold chatter | Latch sets above 0.12 and clears only below 0.08 |
| UnifiedControl-T14 | Saturation then command release | Bounded recovery | Per-sample motor change <= rate*Ts+1e-9; saturation clears within 0.25 s |
| UnifiedControl-T15 | TV/TC/regen/energy enable switching | Bumpless enabled-mode transfer | Motor step <= rate*Ts+1e-9; no NaN/Inf |

## 3. System-in-Loop Validation

| Scenario | Setup | Acceptance |
|---|---|---|
| Short compile/smoke | UnifiedControl controller + Vehicle10DOF Plant, 0.1 s | Update diagram and simulation complete; no nonfinite top-level signals |
| Skidpad TV off/on | Paired scenario, identical fingerprint except feature enables | No boundary/numerical failure; constraints bounded; metrics reported without performance overclaim |
| Autocross TV/TC/energy combinations | Matrix covering single and multiple feature enables | Every combination completes or reports an explicit failure reason; no sequential overwrite signature |
| Brake-entry transient | Vehicle10DOF Plant with negative Fx and yaw demand | No negative normal load truth; regen/friction bounds satisfied; requested braking remains continuous |

Because Vehicle10DOF calibration is not final, closed-loop tests gate robustness and
constraint satisfaction, not absolute lap-time or handling performance.

## 4. Robustness and Sensitivity

| Parameter/input | Sweep | Acceptance |
|---|---|---|
| `UnifiedControlFrictionEstimate` | 0.6, 1.0, 1.2 | Finite results and estimated utilization <= 1+tol |
| Longitudinal speed | -1, 0, 0.1, 5, 30 m/s | No division by zero; low-speed slip blends safely |
| Motor speed | 0, 0.99, 1.00, 1.01 times limit | Speed boundary changes only the accelerating direction |
| AccelX/AccelY | +/- design envelope | Loads remain positive and renormalized to `m*g` |
| Power limits | 0, nominal, 120% nominal | Commands respect supplied runtime limit; zero means no corresponding motor power |
| Measurement validity | Every single-wheel dropout and all invalid | Degraded status; invalid wheels receive no positive drive torque |

## 5. Simulation Configuration

| Setting | Value |
|---|---|
| Component solver | Fixed-step discrete, 0.005 s |
| Closed-loop solver | Inherit verified Vehicle10DOF configuration |
| Component duration | 20 ms to 500 ms depending on rate-ramp scenario |
| Logging | All allocator outputs plus constraint bit mask; top-level buses for system tests |
| Initial state | Zero previous torque, TC latches false |

## 6. Persistent Gherkin Strategy

`tests/unified_control/TorqueAllocatorTestWrapper.slx` converts vector inputs/outputs to
scalar ports so assertions are clear and reusable. The wrapper is test-only,
is never referenced by production models, and is marked accordingly. Draft mode
is used only for syntax iteration; final acceptance requires `draft_mode=false`
because boolean, uint, and vector behavior must compile with real types.

## 7. Execution Commands

```matlab
project = initProject();
addpath(fullfile(project.RootFolder, "scripts", "unified_control"));
installUnifiedControlData();
report = runUnifiedControlVerification(UseFastRestart=true, SaveSummary=true);
```

The `.feature` file is run through `model_test` in full-compilation mode. The
verification script uses `Simulink.SimulationInput` and returns a structured
report containing scenario configuration, constraint maxima, feature status,
recovery metrics, failure reasons, and parameter-source caveats.

## 8. Executed Results

| Layer | Result | Key evidence |
|---|---|---|
| Pure functions | PASS, 10 scenario groups | Balanced nominal allocation; `+100 N*m` yaw sign; TC hysteresis; `66.316 kW` peak drive; `10.000 kW` peak regen; `5.25 N*m` maximum recovery step |
| Component Gherkin | 8/8 scenarios, 45/45 assertions | Full compilation (`draft_mode=false`) on scalar test wrapper |
| Model structure/update | PASS | Four production UnifiedControl models structurally healthy; allocator Stateflow lint healthy; 20 ms allocator/controller simulation passed |
| Integrated 10DOF MIL | 3/3 scenarios | Baseline, all features, and no-regen profiles completed for 0.5 s with finite, bounded outputs |

The integrated runs observed maximum motor request `8.098 N*m`, maximum drive
power `4.884 kW`, minimum normal load `476.713 N`, and maximum motor torque step
`5.25 N*m`. These are regression observations for the current synthetic
Skidpad setup, not vehicle performance claims.

The full friction, speed, motor-speed, acceleration, and every-channel dropout
sweeps listed in Section 4 remain repeatable follow-up work after calibration
evidence is available; they are not claimed as executed by this baseline.
