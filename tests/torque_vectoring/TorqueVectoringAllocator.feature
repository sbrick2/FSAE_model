# --- front-matter:toml ---
model = "TorqueVectoringAllocatorTestWrapper.slx"
[inputs]
Force = "DesiredLongitudinalForce"
YawMoment = "DesiredYawMoment"
SpeedFL = "SpeedFL"
SpeedFR = "SpeedFR"
SpeedRL = "SpeedRL"
SpeedRR = "SpeedRR"
EnableTV = "EnableTV"
EnableRegen = "EnableRegen"
EnableActuation = "EnableActuation"
[outputs]
TorqueFL = "TorqueFL"
TorqueFR = "TorqueFR"
TorqueRL = "TorqueRL"
TorqueRR = "TorqueRR"
BrakeFL = "BrakeFL"
BrakeFR = "BrakeFR"
BrakeRL = "BrakeRL"
BrakeRR = "BrakeRR"
Power = "TotalPowerRequest"
AllocatedYaw = "AllocatedYawMoment"
Saturated = "ControllerSaturated"
# --- end front-matter ---

Feature: TorqueVectoring TV0 and TV1 torque allocation
  Wheel order is FL, FR, RL, RR and positive yaw is a left turn.

Scenario: TV0 divides longitudinal demand equally
  Given inputs
    * Force = const(1000)
    * YawMoment = const(0)
    * SpeedFL = const(100)
    * SpeedFR = const(100)
    * SpeedRL = const(100)
    * SpeedRR = const(100)
    * EnableTV = const(0)
    * EnableRegen = const(1)
    * EnableActuation = const(1)
  When simulate for 20ms in Normal mode
  Then outputs
    * EqualFL: TorqueFL == [4.0 .. 4.1] when t > 5ms
    * EqualFR: TorqueFR == [4.0 .. 4.1] when t > 5ms
    * EqualRL: TorqueRL == [4.0 .. 4.1] when t > 5ms
    * EqualRR: TorqueRR == [4.0 .. 4.1] when t > 5ms
    * NoYaw: AllocatedYaw == [-0.01 .. 0.01]

Scenario: Positive yaw moment increases right-wheel torque
  Given inputs
    * Force = const(1000)
    * YawMoment = const(100)
    * SpeedFL = const(100)
    * SpeedFR = const(100)
    * SpeedRL = const(100)
    * SpeedRR = const(100)
    * EnableTV = const(1)
    * EnableRegen = const(1)
    * EnableActuation = const(1)
  When simulate for 20ms in Normal mode
  Then outputs
    * LowerLeftFront: TorqueFL == [3.3 .. 3.4] when t > 5ms
    * HigherRightFront: TorqueFR == [4.7 .. 4.8] when t > 5ms
    * LowerLeftRear: TorqueRL == [3.3 .. 3.4] when t > 5ms
    * HigherRightRear: TorqueRR == [4.7 .. 4.8] when t > 5ms
    * PositiveYawDelivered: AllocatedYaw == [99 .. 101] when t > 5ms

Scenario: Total drive power is limited before plant actuation
  Given inputs
    * Force = const(100000)
    * YawMoment = const(0)
    * SpeedFL = const(1000)
    * SpeedFR = const(1000)
    * SpeedRL = const(1000)
    * SpeedRR = const(1000)
    * EnableTV = const(0)
    * EnableRegen = const(1)
    * EnableActuation = const(1)
  When simulate for 20ms in Normal mode
  Then outputs
    * PowerBound: Power == [75000 .. 76001] when t > 5ms
    * MotorBoundFL: TorqueFL == [18.9 .. 19.1] when t > 5ms
    * MotorBoundFR: TorqueFR == [18.9 .. 19.1] when t > 5ms
    * SaturationReported: Saturated == 1 when t > 5ms

Scenario: Regen disabled transfers braking demand to friction brakes
  Given inputs
    * Force = const(-1000)
    * YawMoment = const(0)
    * SpeedFL = const(100)
    * SpeedFR = const(100)
    * SpeedRL = const(100)
    * SpeedRR = const(100)
    * EnableTV = const(0)
    * EnableRegen = const(0)
    * EnableActuation = const(1)
  When simulate for 20ms in Normal mode
  Then outputs
    * NoNegativeMotorFL: TorqueFL == 0
    * NoNegativeMotorFR: TorqueFR == 0
    * FrontBrakeApplied: BrakeFL > 0 when t > 5ms
    * RearBrakeApplied: BrakeRL > 0 when t > 5ms
    * BrakingSaturationReported: Saturated == 1 when t > 5ms

Scenario: Disabled actuation produces zero requests
  Given inputs
    * Force = const(5000)
    * YawMoment = const(200)
    * SpeedFL = const(100)
    * SpeedFR = const(100)
    * SpeedRL = const(100)
    * SpeedRR = const(100)
    * EnableTV = const(1)
    * EnableRegen = const(1)
    * EnableActuation = const(0)
  When simulate for 20ms in Normal mode
  Then outputs
    * DisabledMotorFL: TorqueFL == 0
    * DisabledMotorFR: TorqueFR == 0
    * DisabledBrakeFL: BrakeFL == 0
    * DisabledPower: Power == 0
