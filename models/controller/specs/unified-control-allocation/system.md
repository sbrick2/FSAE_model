# UnifiedControl Unified Torque Allocation

## Status: Implemented and verified

**Last Updated:** 2026-08-02  
**Author:** Codex  
**Approval basis:** User request to develop UnifiedControl according to the project roadmap

## 1. Executive Summary

UnifiedControl replaces the TorqueVectoring sequential TV0/TV1 limiter with a single, deterministic,
code-generation-oriented four-wheel allocator. It tracks requested total
longitudinal force and yaw moment while enforcing motor, battery, estimated
tire, traction-control, regenerative-braking, and torque-rate constraints.

The controller boundary remains unchanged. Tire load and slip inputs are
estimated only from `SensorBus` measurements and traceable vehicle parameters;
the allocator never reads `VehicleStateBus` tire-force truth.

## 2. Problem Statement

1. TorqueVectoring applies motor and power limiting after an analytic split and does not
   include TC or estimated tire capacity in the allocation problem.
2. Sequential TC, TV, regeneration, and energy-management overrides can destroy
   the objective achieved by an earlier module.
3. Saturation recovery and feature switching need explicit, bounded behavior.

## 3. Goals and Success Metrics

| ID | Goal | Acceptance criterion |
|---|---|---|
| G1 | Simultaneous constraint satisfaction | No motor, speed, drive/regen power, estimated tire, TC, or rate bound violation above numerical tolerance |
| G2 | Preserve stability authority | Yaw moment remains the higher-priority residual objective when longitudinal demand must be reduced by power constraints |
| G3 | Smooth recovery and switching | Enabled-mode motor torque change is no greater than `UnifiedControlMotorTorqueRateLimit * UnifiedControlSampleTime` per sample; no chattering in TC hysteresis tests |
| G4 | Safe degradation | Invalid wheel-speed channels cannot receive positive drive torque; invalid powertrain feedback selects degraded mode; disabled actuation produces zero requests in one sample |
| G5 | Frozen integration contract | Existing `DriverCommandBus`, `SensorBus`, `PowertrainStateBus`, `ActuatorCommandBus`, and `ControllerDebugBus` are unchanged |

## 4. Non-Goals

| Non-goal | Rationale |
|---|---|
| Nonlinear MPC or a general online QP solver | Roadmap explicitly defers large or hard-to-generate solvers |
| Direct use of true tire forces or true road friction | Violates the controller/Plant boundary |
| Independent hydraulic brake yaw control | Current Plant redistributes total friction-brake demand using a fixed hydraulic proportion |
| Final production calibration or performance claim | TC thresholds, friction estimate, rate limit, and energy reserve remain tentative pending vehicle data |
| SIL/PIL and target timing closure | Deployment scope; UnifiedControl remains MIL with code-generation-safe constructs |

## 5. Operating Scenarios

| ID | Scenario | Expected behavior | Criterion |
|---|---|---|---|
| S1 | Disabled actuation | All actuator requests and active flags are zero | Exact zero after one invocation |
| S2 | Nominal straight drive | Four wheel motor requests are equal | Spread <= 0.05 N*m after the rate ramp |
| S3 | Positive yaw request | Right-side motor torque exceeds left-side torque | Positive allocated yaw; requested sign preserved |
| S4 | Single-wheel overslip | TC reduces only the affected positive-force upper bound | Affected drive torque below peer wheel and `TCActive=true` |
| S5 | Drive power saturation | Common longitudinal authority is reduced while yaw authority is retained where feasible | Electrical request <= active drive limit + 1 W |
| S6 | Regenerative braking | Negative motor torque is used up to motor, SOC, slip, rate, and regen-power bounds; friction fills the remaining brake demand | Regen electrical power <= limit + 1 W; friction request nonnegative |
| S7 | Regen disabled/full SOC | No negative motor torque; friction carries braking | `RegenTorqueRequest=0` and motor torque >= 0 |
| S8 | Feature switching/recovery | Torque ramps without an enabled-mode discontinuity; TC clears below the off threshold | Per-sample rate bound met; TC clears within one sample below threshold |
| S9 | Invalid measurement | Invalid wheel channel is drive-inhibited and controller mode is degraded | No positive torque on invalid wheel; mode 3 |

## 6. External Interface Contract

### 6.1 `TorqueAllocator.slx` inputs

All ports execute at `UnifiedControlSampleTime = 0.005 s`.

| Name | Size/type | Unit | Meaning |
|---|---|---|---|
| `DesiredLongitudinalForce` | scalar double | N | Positive drive, negative braking |
| `DesiredYawMoment` | scalar double | N*m | Positive left-turn moment |
| `LongitudinalSpeed` | scalar double | m/s | Measured vehicle longitudinal speed |
| `WheelSpeed` | 4x1 double | rad/s | Measured wheel speed `[FL FR RL RR]` |
| `MotorSpeed` | 4x1 double | rad/s | Measured motor speed |
| `AccelX`, `AccelY` | scalar double | m/s^2 | IMU accelerations in vehicle axes |
| `BatterySOC` | scalar double | 1 | Powertrain SOC estimate |
| `WheelSpeedValid` | 4x1 boolean | 1 | Per-wheel measurement validity |
| `PowertrainValid` | boolean | 1 | Powertrain measurement validity |
| `EnableTV`, `EnableTC`, `EnableRegen`, `EnableEnergyManagement` | boolean | 1 | Feature enables |
| `EnableActuation` | boolean | 1 | Safety actuation gate |

### 6.2 Outputs

| Name | Size/type | Unit | Meaning |
|---|---|---|---|
| `MotorTorqueRequest` | 4x1 double | N*m | Motor-shaft torque; negative is regeneration |
| `FrictionBrakeTorqueRequest` | 4x1 double | N*m | Nonnegative wheel-end brake magnitude |
| `RegenTorqueRequest` | 4x1 double | N*m | Nonnegative diagnostic magnitude, not added again by Plant |
| `TotalPowerRequest` | scalar double | W | Positive discharge, negative regeneration |
| `AllocatedYawMoment` | scalar double | N*m | Yaw moment generated by motor torque request |
| `WheelTorqueUnconstrained` | 4x1 double | N*m | Pre-constraint motor-equivalent target |
| `ControllerSaturated` | boolean | 1 | Any objective or physical constraint is active |
| `TCActive`, `RegenActive`, `EnergyManagementActive` | boolean | 1 | Feature status after validity/mode arbitration |
| `ControllerMode` | uint8 | 1 | 0 Disabled, 2 Active, 3 Degraded |
| `ConstraintFlags` | uint16 | 1 | Test/diagnostic bit mask local to UnifiedControl model |

UnifiedControl integration maps the first nine applicable outputs into the already frozen
`ActuatorCommandBus` and `ControllerDebugBus`; it does not add bus elements.

### 6.3 Parameters

Physical aliases are copied from the shared data dictionary. New values below
are explicit tentative controls, not measured values.

| Parameter | Default | Unit | Source/status |
|---|---:|---|---|
| `UnifiedControlSampleTime` | 0.005 | s | `Simulation.TsController` |
| `UnifiedControlMotorTorqueRateLimit` | `MotorTorqueLimit / MotorResponseTime` | N*m/s | Derived from current typical response-time record; tentative |
| `UnifiedControlFrictionEstimate` | 1.0 | 1 | TorqueVectoring scenario estimate; placeholder |
| `UnifiedControlTCSlipOn` / `UnifiedControlTCSlipOff` | 0.12 / 0.08 | 1 | Tentative hysteresis thresholds |
| `UnifiedControlTCSlipFullCut` | 0.25 | 1 | Tentative full-derate threshold |
| `UnifiedControlMinimumNormalLoad` | 50 | N | Numerical/tire-capacity protection |
| `UnifiedControlEnergyDriveFraction` | 0.90 | 1 | Tentative energy-management reserve |
| `UnifiedControlAllocatorIterations` | 24 | 1 | Fixed design-time iteration count |
| `UnifiedControlYawPriorityWeight` | 4.0 | 1 | Stability priority over common force under saturation |
| `UnifiedControlForcePriorityWeight` | 1.0 | 1 | Longitudinal tracking weight |
| `UnifiedControlRegularizationWeight` | 1e-4 | 1 | Unique, balanced allocation regularization |

## 7. Operating Modes

| Mode | Entry | Behavior | State action |
|---|---|---|---|
| Disabled (0) | `EnableActuation=false` | Zero outputs immediately | Previous torque and TC latches reset |
| Active (2) | Actuation and powertrain valid | All requested features participate in one constrained allocation | Previous command tracks allocated motor torque |
| Degraded (3) | Actuation enabled and powertrain invalid, or any wheel-speed invalid | Valid channels remain bounded; invalid channels are drive-inhibited; energy status is disabled if unavailable | State retained subject to rate limit |

## 8. Code Generation and Execution Constraints

- Single-rate, discrete, double-precision UnifiedControl baseline at 5 ms.
- Fixed-size arrays and fixed iteration count; no variable-size data, dynamic
  allocation, Optimization Toolbox, exceptions, or workspace-only state.
- Calibration and physical aliases reside in `VehicleData.sldd`.
- Target processor, storage classes, fixed-point strategy, WCET, and formal
  coding standard remain Deployment/open-question items.

## 9. Open Questions

1. Confirm production TC slip thresholds and hysteresis from vehicle testing.
2. Confirm motor torque slew capability from inverter/motor bench data.
3. Confirm on-vehicle road-friction and normal-load estimation strategy.
4. Confirm whether a future hydraulic system can command independent wheel brake torque.

## Appendix A: Related Documents

- [Roadmap](../../../../ROADMAP.md)
- [Signal interfaces](../../../SignalInterfaces.md)
- [TorqueVectoring design](../../Torque_Vectoring.md)
- [Architecture](architecture.md)
- [Implementation plan](implementation-plan.md)
- [Test plan](test-plan.md)

## Appendix B: Research Notes

The allocation mapping and sign convention reuse the verified TorqueVectoring equations and
project coordinate-system contract. No external standard or time-varying API is
needed for this deterministic MIL baseline.
