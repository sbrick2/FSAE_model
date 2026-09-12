# --- front-matter:toml ---
model = "TorqueAllocatorTestWrapper.slx"
[inputs]
Force = "DesiredLongitudinalForce"
YawMoment = "DesiredYawMoment"
VehicleSpeed = "LongitudinalSpeed"
WheelSpeedFL = "WheelSpeedFL"
WheelSpeedFR = "WheelSpeedFR"
WheelSpeedRL = "WheelSpeedRL"
WheelSpeedRR = "WheelSpeedRR"
MotorSpeedFL = "MotorSpeedFL"
MotorSpeedFR = "MotorSpeedFR"
MotorSpeedRL = "MotorSpeedRL"
MotorSpeedRR = "MotorSpeedRR"
AccelX = "AccelX"
AccelY = "AccelY"
SOC = "BatterySOC"
WheelValidFL = "WheelValidFL"
WheelValidFR = "WheelValidFR"
WheelValidRL = "WheelValidRL"
WheelValidRR = "WheelValidRR"
PowertrainValid = "PowertrainValid"
EnableTV = "EnableTV"
EnableTC = "EnableTC"
EnableRegen = "EnableRegen"
EnableEnergy = "EnableEnergyManagement"
EnableActuation = "EnableActuation"
[outputs]
TorqueFL = "TorqueFL"
TorqueFR = "TorqueFR"
TorqueRL = "TorqueRL"
TorqueRR = "TorqueRR"
BrakeFL = "BrakeFL"
BrakeFR = "BrakeFR"
RegenFL = "RegenFL"
RegenFR = "RegenFR"
Power = "TotalPowerRequest"
AllocatedYaw = "AllocatedYawMoment"
Saturated = "ControllerSaturated"
TCActive = "TCActive"
RegenActive = "RegenActive"
EnergyActive = "EnergyManagementActive"
Mode = "ControllerMode"
Flags = "ConstraintFlags"
# --- end front-matter ---

Feature: UnifiedControl unified four-wheel torque allocation
  The allocator combines TV, TC, regeneration, energy and physical limits.

Scenario: Nominal straight drive is balanced
  Given inputs
    * Force = const(1000)
    * YawMoment = const(0)
    * VehicleSpeed = const(10)
    * WheelSpeedFL = const(50)
    * WheelSpeedFR = const(50)
    * WheelSpeedRL = const(50)
    * WheelSpeedRR = const(50)
    * MotorSpeedFL = const(637.68)
    * MotorSpeedFR = const(637.68)
    * MotorSpeedRL = const(637.68)
    * MotorSpeedRR = const(637.68)
    * AccelX = const(0)
    * AccelY = const(0)
    * SOC = const(0.5)
    * WheelValidFL = const(1)
    * WheelValidFR = const(1)
    * WheelValidRL = const(1)
    * WheelValidRR = const(1)
    * PowertrainValid = const(1)
    * EnableTV = const(0)
    * EnableTC = const(0)
    * EnableRegen = const(1)
    * EnableEnergy = const(0)
    * EnableActuation = const(1)
  When simulate for 20ms in Normal mode
  Then outputs
    * BalancedFL: TorqueFL == [4.0 .. 4.1] when t > 10ms
    * BalancedFR: TorqueFR == [4.0 .. 4.1] when t > 10ms
    * BalancedRL: TorqueRL == [4.0 .. 4.1] when t > 10ms
    * BalancedRR: TorqueRR == [4.0 .. 4.1] when t > 10ms
    * NoYaw: AllocatedYaw == [-0.01 .. 0.01]
    * ActiveMode: Mode == 2 when t > 5ms

Scenario: Positive yaw request raises right wheel torque
  Given inputs
    * Force = const(1000)
    * YawMoment = const(100)
    * VehicleSpeed = const(10)
    * WheelSpeedFL = const(50)
    * WheelSpeedFR = const(50)
    * WheelSpeedRL = const(50)
    * WheelSpeedRR = const(50)
    * MotorSpeedFL = const(637.68)
    * MotorSpeedFR = const(637.68)
    * MotorSpeedRL = const(637.68)
    * MotorSpeedRR = const(637.68)
    * AccelX = const(0)
    * AccelY = const(0)
    * SOC = const(0.5)
    * WheelValidFL = const(1)
    * WheelValidFR = const(1)
    * WheelValidRL = const(1)
    * WheelValidRR = const(1)
    * PowertrainValid = const(1)
    * EnableTV = const(1)
    * EnableTC = const(0)
    * EnableRegen = const(1)
    * EnableEnergy = const(0)
    * EnableActuation = const(1)
  When simulate for 20ms in Normal mode
  Then outputs
    * LeftFront: TorqueFL == [3.3 .. 3.4] when t > 10ms
    * RightFront: TorqueFR == [4.7 .. 4.8] when t > 10ms
    * LeftRear: TorqueRL == [3.3 .. 3.4] when t > 10ms
    * RightRear: TorqueRR == [4.7 .. 4.8] when t > 10ms
    * PositiveYaw: AllocatedYaw == [98 .. 102] when t > 10ms

Scenario: Single wheel overslip activates traction control
  Given inputs
    * Force = const(1000)
    * YawMoment = const(100)
    * VehicleSpeed = const(10)
    * WheelSpeedFL = const(60)
    * WheelSpeedFR = const(50)
    * WheelSpeedRL = const(50)
    * WheelSpeedRR = const(50)
    * MotorSpeedFL = const(765.216)
    * MotorSpeedFR = const(637.68)
    * MotorSpeedRL = const(637.68)
    * MotorSpeedRR = const(637.68)
    * AccelX = const(0)
    * AccelY = const(0)
    * SOC = const(0.5)
    * WheelValidFL = const(1)
    * WheelValidFR = const(1)
    * WheelValidRL = const(1)
    * WheelValidRR = const(1)
    * PowertrainValid = const(1)
    * EnableTV = const(1)
    * EnableTC = const(1)
    * EnableRegen = const(1)
    * EnableEnergy = const(0)
    * EnableActuation = const(1)
  When simulate for 20ms in Normal mode
  Then outputs
    * TCSet: TCActive == 1 when t > 5ms
    * FLDerated: TorqueFL == [0 .. 3.5] when t > 10ms
    * FRRetained: TorqueFR > 3.5 when t > 10ms
    * ConstraintReported: Saturated == 1 when t > 5ms

Scenario: Drive power energy and TC constraints combine
  Given inputs
    * Force = const(100000)
    * YawMoment = const(200)
    * VehicleSpeed = const(20)
    * WheelSpeedFL = const(125)
    * WheelSpeedFR = const(100)
    * WheelSpeedRL = const(100)
    * WheelSpeedRR = const(100)
    * MotorSpeedFL = const(1000)
    * MotorSpeedFR = const(1000)
    * MotorSpeedRL = const(1000)
    * MotorSpeedRR = const(1000)
    * AccelX = const(2)
    * AccelY = const(5)
    * SOC = const(0.5)
    * WheelValidFL = const(1)
    * WheelValidFR = const(1)
    * WheelValidRL = const(1)
    * WheelValidRR = const(1)
    * PowertrainValid = const(1)
    * EnableTV = const(1)
    * EnableTC = const(1)
    * EnableRegen = const(1)
    * EnableEnergy = const(1)
    * EnableActuation = const(1)
  When simulate for 30ms in Normal mode
  Then outputs
    * PowerBound: Power == [0 .. 72001] when t > 20ms
    * MotorBoundFL: TorqueFL == [0 .. 21] when t > 20ms
    * MotorBoundFR: TorqueFR == [0 .. 21] when t > 20ms
    * TCCombined: TCActive == 1 when t > 5ms
    * EnergyCombined: EnergyActive == 1 when t > 5ms
    * MultiConstraint: Flags > 0 when t > 5ms

Scenario: Regeneration shares braking with friction under power limit
  Given inputs
    * Force = const(-10000)
    * YawMoment = const(0)
    * VehicleSpeed = const(20)
    * WheelSpeedFL = const(100)
    * WheelSpeedFR = const(100)
    * WheelSpeedRL = const(100)
    * WheelSpeedRR = const(100)
    * MotorSpeedFL = const(1000)
    * MotorSpeedFR = const(1000)
    * MotorSpeedRL = const(1000)
    * MotorSpeedRR = const(1000)
    * AccelX = const(-5)
    * AccelY = const(0)
    * SOC = const(0.5)
    * WheelValidFL = const(1)
    * WheelValidFR = const(1)
    * WheelValidRL = const(1)
    * WheelValidRR = const(1)
    * PowertrainValid = const(1)
    * EnableTV = const(0)
    * EnableTC = const(0)
    * EnableRegen = const(1)
    * EnableEnergy = const(0)
    * EnableActuation = const(1)
  When simulate for 20ms in Normal mode
  Then outputs
    * RegenPowerBound: Power == [-10001 .. 0] when t > 10ms
    * RegenUsedFL: RegenFL > 0 when t > 10ms
    * RegenUsedFR: RegenFR > 0 when t > 10ms
    * FrictionResidualFL: BrakeFL > 0 when t > 10ms
    * FrictionResidualFR: BrakeFR > 0 when t > 10ms
    * RegenStatus: RegenActive == 1 when t > 5ms

Scenario: Regen disabled transfers braking to friction
  Given inputs
    * Force = const(-1000)
    * YawMoment = const(0)
    * VehicleSpeed = const(10)
    * WheelSpeedFL = const(50)
    * WheelSpeedFR = const(50)
    * WheelSpeedRL = const(50)
    * WheelSpeedRR = const(50)
    * MotorSpeedFL = const(637.68)
    * MotorSpeedFR = const(637.68)
    * MotorSpeedRL = const(637.68)
    * MotorSpeedRR = const(637.68)
    * AccelX = const(-2)
    * AccelY = const(0)
    * SOC = const(0.5)
    * WheelValidFL = const(1)
    * WheelValidFR = const(1)
    * WheelValidRL = const(1)
    * WheelValidRR = const(1)
    * PowertrainValid = const(1)
    * EnableTV = const(0)
    * EnableTC = const(0)
    * EnableRegen = const(0)
    * EnableEnergy = const(0)
    * EnableActuation = const(1)
  When simulate for 20ms in Normal mode
  Then outputs
    * NoRegenFL: RegenFL == 0
    * NoRegenFR: RegenFR == 0
    * NoNegativeMotorFL: TorqueFL == 0
    * NoNegativeMotorFR: TorqueFR == 0
    * FrictionFL: BrakeFL > 0 when t > 10ms
    * FrictionFR: BrakeFR > 0 when t > 10ms
    * RegenOff: RegenActive == 0

Scenario: Invalid wheel speed selects degraded drive inhibition
  Given inputs
    * Force = const(1000)
    * YawMoment = const(0)
    * VehicleSpeed = const(10)
    * WheelSpeedFL = const(50)
    * WheelSpeedFR = const(50)
    * WheelSpeedRL = const(50)
    * WheelSpeedRR = const(50)
    * MotorSpeedFL = const(637.68)
    * MotorSpeedFR = const(637.68)
    * MotorSpeedRL = const(637.68)
    * MotorSpeedRR = const(637.68)
    * AccelX = const(0)
    * AccelY = const(0)
    * SOC = const(0.5)
    * WheelValidFL = const(0)
    * WheelValidFR = const(1)
    * WheelValidRL = const(1)
    * WheelValidRR = const(1)
    * PowertrainValid = const(1)
    * EnableTV = const(0)
    * EnableTC = const(1)
    * EnableRegen = const(1)
    * EnableEnergy = const(0)
    * EnableActuation = const(1)
  When simulate for 20ms in Normal mode
  Then outputs
    * DegradedMode: Mode == 3 when t > 5ms
    * InvalidFLInhibited: TorqueFL <= 0 when t > 5ms
    * ValidFRAvailable: TorqueFR > 0 when t > 10ms
    * ValidityFlag: Flags > 0 when t > 5ms

Scenario: Disabled actuation is immediate safe zero
  Given inputs
    * Force = const(100000)
    * YawMoment = const(500)
    * VehicleSpeed = const(20)
    * WheelSpeedFL = const(100)
    * WheelSpeedFR = const(100)
    * WheelSpeedRL = const(100)
    * WheelSpeedRR = const(100)
    * MotorSpeedFL = const(1000)
    * MotorSpeedFR = const(1000)
    * MotorSpeedRL = const(1000)
    * MotorSpeedRR = const(1000)
    * AccelX = const(5)
    * AccelY = const(5)
    * SOC = const(0.5)
    * WheelValidFL = const(1)
    * WheelValidFR = const(1)
    * WheelValidRL = const(1)
    * WheelValidRR = const(1)
    * PowertrainValid = const(1)
    * EnableTV = const(1)
    * EnableTC = const(1)
    * EnableRegen = const(1)
    * EnableEnergy = const(1)
    * EnableActuation = const(0)
  When simulate for 20ms in Normal mode
  Then outputs
    * DisabledFL: TorqueFL == 0
    * DisabledFR: TorqueFR == 0
    * DisabledRL: TorqueRL == 0
    * DisabledRR: TorqueRR == 0
    * DisabledBrakeFL: BrakeFL == 0
    * DisabledPower: Power == 0
    * DisabledMode: Mode == 0
