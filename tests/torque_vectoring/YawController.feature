# --- front-matter:toml ---
model = "YawController.slx"
[inputs]
Steering = "SteeringAngle"
Speed = "LongitudinalSpeed"
Yaw = "YawRate"
Enable = "EnableTV"
Valid = "IMUValid"
[outputs]
Reference = "ReferenceYawRate"
Error = "YawRateError"
Moment = "DesiredYawMoment"
Active = "TVActive"
Saturated = "ControllerSaturated"
# --- end front-matter ---

Feature: TorqueVectoring TV1 yaw-rate feedback
  The controller follows the project left-turn-positive sign convention.

Scenario: Positive steering requests positive yaw moment
  Given inputs
    * Steering = const(0.1)
    * Speed = const(10)
    * Yaw = const(0)
    * Enable = const(1)
    * Valid = const(1)
  When simulate for 50ms in Normal mode
  Then outputs
    * PositiveReference: Reference > 0 when t > 10ms
    * PositiveError: Error > 0 when t > 10ms
    * PositiveMoment: Moment > 0 when t > 10ms
    * MomentBounded: Moment == [-350 .. 350]
    * ControllerEnabled: Active == 1 when t > 10ms

Scenario: Excess positive yaw rate requests negative correction
  Given inputs
    * Steering = const(0.05)
    * Speed = const(10)
    * Yaw = const(1)
    * Enable = const(1)
    * Valid = const(1)
  When simulate for 50ms in Normal mode
  Then outputs
    * NegativeError: Error < 0 when t > 10ms
    * NegativeMoment: Moment < 0 when t > 10ms

Scenario: Stationary steering has zero yaw reference
  Given inputs
    * Steering = const(0.2)
    * Speed = const(0)
    * Yaw = const(0)
    * Enable = const(1)
    * Valid = const(1)
  When simulate for 50ms in Normal mode
  Then outputs
    * ZeroReference: Reference == 0
    * ZeroMoment: Moment == 0
    * LowSpeedActive: Active == 1 when t > 10ms

Scenario: Disabled controller resets its output
  Given inputs
    * Steering = const(0.2)
    * Speed = const(15)
    * Yaw = const(-1)
    * Enable = const(0)
    * Valid = const(1)
  When simulate for 50ms in Normal mode
  Then outputs
    * DisabledReferenceAvailable: Reference > 0 when t > 10ms
    * DisabledMoment: Moment == 0
    * DisabledFlag: Active == 0

Scenario: Large yaw error is moment limited
  Given inputs
    * Steering = const(0.2)
    * Speed = const(15)
    * Yaw = const(-10)
    * Enable = const(1)
    * Valid = const(1)
  When simulate for 50ms in Normal mode
  Then outputs
    * PositiveLimit: Moment == [349 .. 350] when t > 10ms
    * SaturationReported: Saturated == 1 when t > 10ms
