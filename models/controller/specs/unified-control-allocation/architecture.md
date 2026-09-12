# UnifiedControl Unified Torque Allocation Architecture

## Status: Implemented and verified

**Last Updated:** 2026-08-02  
**Parent Spec:** [System spec](system.md)

## 1. Overview

`TorqueAllocator.slx` is a separate controller model called by
`UnifiedControlVehicleController.slx`. It receives force/moment objectives plus controller-
available measurements, estimates per-wheel constraints, solves a bounded
four-wheel allocation, and mixes motor regeneration with residual friction
braking without changing the frozen project buses.

## 2. Goals, Non-Goals, and Constraints

| ID | Architectural decision |
|---|---|
| G1 | Separate constraint estimation, force allocation, and actuator mixing in the model documentation and function-level code |
| G2 | Keep a single state owner for prior motor torque and TC hysteresis |
| G3 | Use only measured/estimated controller signals; never tire-force truth |
| C1 | 5 ms single-rate, fixed-size arrays, R2026a |
| C2 | Existing Bus definitions remain unchanged |
| C3 | Current Plant supports only fixed-proportion total hydraulic braking |
| C4 | Current Vehicle10DOF physical calibration remains uncertain; UnifiedControl uses traceable placeholders where required |

## 3. Functional Decomposition

```text
Fx/Mz objectives + Sensor/Powertrain measurements
                       |
                       v
          [Measurement & Load Estimation]
          Fz_est, slip_est, lateral reserve
                       |
                       v
          [Mode and Constraint Synthesis]
       motor/tire/TC/rate/power upper/lower bounds
                       |
                       v
        [Projected Weighted Force Allocation]
          four desired longitudinal wheel forces
                       |
                       v
       [Motor/Regen/Friction Actuator Mixing]
        requests + status + state for next sample
```

### 3.1 Component Catalog

| Component | Implementation | DFT | Responsibility |
|---|---|---|---|
| `UnifiedAllocationLogic` | Stateflow chart with one persistent active state | Partial | Own previous torque/TC state, reset, and mode behavior |
| `unifiedControlEstimateWheelConstraints` | MATLAB function called by allocation step | Yes | Estimate slip, normal load, lateral reserve, and wheel force bounds |
| `unifiedControlProjectWheelForces` | MATLAB function called by allocation step | Yes | Fixed-iteration projected weighted least-squares allocation |
| `unifiedControlMixActuators` | MATLAB function called by allocation step | Yes | Map wheel force to motor, regen diagnostic, and residual friction brake |
| `unifiedControlUnifiedAllocationStep` | Orchestrator MATLAB function | Partial | Call components, update state, compute diagnostics |
| `YawController` | Existing model reference | Partial | Produce `DesiredYawMoment`; UnifiedControl does not change PI internals |

The Stateflow state is not an all-in-one MATLAB Function block: it is the
explicit state/mode boundary, while estimator, optimizer, and mixer remain
separate testable MATLAB functions.

## 4. Algorithm Details

### 4.1 Measurement and load estimation

Slip is estimated using measured wheel speed and longitudinal speed:

```text
kappa_i = (R*omega_i - Ux) / max(abs(Ux), UnifiedControlLowSpeedEpsilon)
```

At low speed, the estimate is blended toward zero. Static axle loads use the
shared front distribution. Longitudinal transfer is `m*ax*h/L`; lateral
transfer is split front/rear by the shared roll-stiffness distribution and
the applicable track. Every load is clamped above
`UnifiedControlMinimumNormalLoad`, then the four loads are renormalized to `m*g`.

Estimated lateral force is distributed by static axle load. Available
longitudinal tire force is the nonnegative friction-circle remainder:

```text
FxCap_i = sqrt(max((mu_est*Fz_est_i)^2 - Fy_est_i^2, 0))
```

### 4.2 Constraint synthesis

Motor torque-speed bounds, rate bounds about the previous request, estimated
tire capacity, wheel validity, TC derating, drive power, regen power, SOC, and
feature enables are converted into explicit upper/lower limits before solving.
TC uses hysteresis (`SlipOn`, `SlipOff`) and linearly derates positive drive
capacity to zero at `SlipFullCut`.

### 4.3 Projected allocation

For wheel-force vector `f=[FL FR RL RR]'`,

```text
Fx_alloc = [1 1 1 1] f
Mz_alloc = [-tf/2 tf/2 -tr/2 tr/2] f
```

The solver minimizes normalized weighted residuals in `Fx` and `Mz` plus a
small balanced-allocation regularizer. A fixed number of gradient/projection
iterations applies box bounds and the linear drive-power half-space. The yaw
weight is greater than the force weight, so power saturation primarily removes
the common-force component. This is deterministic and toolbox-independent.

### 4.4 Actuator mixing

Positive wheel force maps to motor torque. Negative wheel force uses available
regeneration first. When regen is disabled, SOC-limited, rate-limited, or
power-limited, nonnegative wheel-end friction torque fills the remaining brake
demand. Because the Plant redistributes hydraulic demand, `AllocatedYawMoment`
reports only the motor-generated yaw moment and saturation is asserted when the
requested braking yaw cannot be retained.

### 4.5 State and mode behavior

The chart owns `PreviousMotorTorque(4)` and `TCState(4)`. Disabled entry resets
both and outputs zero. Active/degraded execution updates both at 5 ms. Feature
switches are bumpless because the motor rate box is centered on the previous
request; safety disable is intentionally immediate.

## 5. Saturation, Numerical Safety, and Loops

| Concern | Approach |
|---|---|
| Motor magnitude/speed | Per-wheel box bound; drive beyond speed limit prohibited, opposing regen retained |
| Torque rate | Per-sample upper/lower bound from previous request |
| Drive/regen power | Projection/mixing against electrical power limits with 1 W epsilon |
| Division by zero | Radius, tracks, gear product, speed, power, and normalization scales use positive floors |
| Nonfinite input | Sanitized to a safe finite default; affected validity produces degraded status |
| Algebraic loop | Previous torque and TC latches are explicit chart local state; no direct controller/Plant truth loop |
| Integrator anti-windup | UnifiedControl has no integrator; existing `YawController` retains its verified anti-windup |

## 6. Parameter Management

`installUnifiedControlData.m` creates `UnifiedControlDesign` and `UnifiedControl*` runtime
`Simulink.Parameter` aliases in `VehicleData.sldd`. Physical values are copied
from `Vehicle`, `Tire`, `Powertrain`, `Battery`, `Brake`, and `Simulation`.
New thresholds and weights are marked tentative, low-confidence design values.

## 7. Key Decisions

| # | Decision | Rationale |
|---|---|---|
| D1 | Add new UnifiedControl models instead of modifying dirty TorqueVectoring/Vehicle10DOF production baselines | Preserves regressions and user work |
| D2 | Estimate wheel constraints inside controller | Meets tire/TC roadmap needs without truth leakage |
| D3 | Use projected weighted least squares | Handles simultaneous constraints with fixed execution and no extra toolbox |
| D4 | Keep buses frozen and use a local constraint bit mask | Avoids wide impact on all referenced models |
| D5 | Treat current hydraulic brake as total fixed-proportion demand | Matches actual Plant behavior |

## 8. Known Limitations

- Normal-load and lateral-force estimates are quasi-static and use tentative
  vehicle parameters; Vehicle10DOF truth is used only for offline validation.
- No online road-friction estimator is available.
- Fixed iteration count gives a bounded approximate optimum, not a proof of the
  global nonlinear optimum.
- Formal Model Advisor standard, generated-code timing, SIL, and PIL are open.

## Appendix A: Related Documents

- [System spec](system.md)
- [Implementation plan](implementation-plan.md)
- [Test plan](test-plan.md)

## Appendix B: API Verification Notes

All model constructs reuse verified neighboring patterns: Stateflow chart model
wrappers in `TorqueVectoringAllocator.slx` and `YawController.slx`, data-dictionary
installation in `scripts/torque_vectoring/installTorqueVectoringControlData.m`, Model Reference integration
in `TorqueVectoringVehicleController.slx`, and Gherkin wrappers in `tests/torque_vectoring/`. The allocator
uses only fixed-size MATLAB arithmetic supported by the existing codebase; no
new toolbox API is assumed.
