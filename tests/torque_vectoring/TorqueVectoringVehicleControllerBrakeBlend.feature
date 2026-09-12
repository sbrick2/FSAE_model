# --- front-matter:toml ---
model = "TorqueVectoringVehicleController.slx"
[inputs]
Steering = "DriverCommand.SteeringRackAngleRequest"
Acceleration = "DriverCommand.LongitudinalAccelerationRequest"
EnableTV = "DriverCommand.EnableTV"
EnableRegen = "DriverCommand.EnableRegen"
DriverMode = "DriverCommand.DriverMode"
Speed = "Sensor.LongitudinalSpeed"
YawRate = "Sensor.YawRate"
PoseValid = "Sensor.PoseValid"
IMUValid = "Sensor.IMUValid"
MotorSpeedFL = "PowertrainState.MotorSpeed(1)"
MotorSpeedFR = "PowertrainState.MotorSpeed(2)"
MotorSpeedRL = "PowertrainState.MotorSpeed(3)"
MotorSpeedRR = "PowertrainState.MotorSpeed(4)"
BatterySOC = "PowertrainState.BatterySOC"
[outputs]
Power = "ActuatorCommand.TotalPowerRequest"
DesiredForce = "ControllerDebug.DesiredLongitudinalForce"
RegenActive = "ControllerDebug.RegenActive"
Saturated = "ControllerDebug.ControllerSaturated"
# --- end front-matter ---

Feature: TorqueVectoring brake blending respects battery SOC availability
  High SOC disables regeneration before torque allocation so friction brakes
  receive the complete braking demand instead of losing clipped regen torque.

Scenario: SOC at upper limit transfers all braking demand to friction brakes
  Given inputs
    * Steering = const(0)
    * Acceleration = const(-4)
    * EnableTV = const(0)
    * EnableRegen = const(1)
    * DriverMode = const(2)
    * Speed = const(10)
    * YawRate = const(0)
    * PoseValid = const(1)
    * IMUValid = const(1)
    * MotorSpeedFL = const(600)
    * MotorSpeedFR = const(600)
    * MotorSpeedRL = const(600)
    * MotorSpeedRR = const(600)
    * BatterySOC = const(0.95)
  When simulate for 20ms in Normal mode
  Then outputs
    * BrakingDemandPreserved: DesiredForce == [-1281 .. -1279] when t > 5ms
    * NoRegenerationPower: Power == 0 when t > 5ms
    * RegenReportedInactive: RegenActive == 0 when t > 5ms
    * FrictionFallbackReported: Saturated == 1 when t > 5ms

Scenario: SOC below upper limit retains regenerative braking
  Given inputs
    * Steering = const(0)
    * Acceleration = const(-1)
    * EnableTV = const(0)
    * EnableRegen = const(1)
    * DriverMode = const(2)
    * Speed = const(10)
    * YawRate = const(0)
    * PoseValid = const(1)
    * IMUValid = const(1)
    * MotorSpeedFL = const(600)
    * MotorSpeedFR = const(600)
    * MotorSpeedRL = const(600)
    * MotorSpeedRR = const(600)
    * BatterySOC = const(0.8)
  When simulate for 20ms in Normal mode
  Then outputs
    * BrakingDemandPreserved: DesiredForce == [-321 .. -319] when t > 5ms
    * RegenerationPowerRequested: Power < 0 when t > 5ms
    * RegenReportedActive: RegenActive == 1 when t > 5ms
    * NoFrictionFallback: Saturated == 0 when t > 5ms
