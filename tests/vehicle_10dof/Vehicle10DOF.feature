# --- front-matter:toml ---
model = "Vehicle10DOF.slx"
[inputs]
TireFxFL = "TireForceXWheel(1)"
TireFxFR = "TireForceXWheel(2)"
TireFxRL = "TireForceXWheel(3)"
TireFxRR = "TireForceXWheel(4)"
TireFyFL = "TireForceYWheel(1)"
TireFyFR = "TireForceYWheel(2)"
TireFyRL = "TireForceYWheel(3)"
TireFyRR = "TireForceYWheel(4)"
AlignMzFL = "TireAligningMoment(1)"
AlignMzFR = "TireAligningMoment(2)"
AlignMzRL = "TireAligningMoment(3)"
AlignMzRR = "TireAligningMoment(4)"
SteerFL = "WheelSteerAngle(1)"
SteerFR = "WheelSteerAngle(2)"
SteerRL = "WheelSteerAngle(3)"
SteerRR = "WheelSteerAngle(4)"
TorqueFL = "WheelAppliedTorque(1)"
TorqueFR = "WheelAppliedTorque(2)"
TorqueRL = "WheelAppliedTorque(3)"
TorqueRR = "WheelAppliedTorque(4)"
ExternalFx = "ExternalForceXBody"
ExternalFy = "ExternalForceYBody"
ExternalMz = "ExternalYawMoment"
DownforceF = "DownforceFront"
DownforceR = "DownforceRear"
RoadHeightFL = "RoadHeight(1)"
RoadHeightFR = "RoadHeight(2)"
RoadHeightRL = "RoadHeight(3)"
RoadHeightRR = "RoadHeight(4)"
RoadVelocityFL = "RoadVelocity(1)"
RoadVelocityFR = "RoadVelocity(2)"
RoadVelocityRL = "RoadVelocity(3)"
RoadVelocityRR = "RoadVelocity(4)"
[outputs]
Roll = "RollAngle"
Pitch = "PitchAngle"
Heave = "VerticalPosition"
VerticalAcceleration = "Az"
# --- end front-matter ---

Feature: Vehicle10DOF Vehicle10DOF suspension dynamics
  The three added sprung-body DOFs must preserve the E41 static balance.

Scenario: Static equilibrium preserves four-corner load distribution
  At zero road and force input, the initialized vehicle remains in equilibrium.
  Given inputs
    * TireFxFL = const(0)
    * TireFxFR = const(0)
    * TireFxRL = const(0)
    * TireFxRR = const(0)
    * TireFyFL = const(0)
    * TireFyFR = const(0)
    * TireFyRL = const(0)
    * TireFyRR = const(0)
    * AlignMzFL = const(0)
    * AlignMzFR = const(0)
    * AlignMzRL = const(0)
    * AlignMzRR = const(0)
    * SteerFL = const(0)
    * SteerFR = const(0)
    * SteerRL = const(0)
    * SteerRR = const(0)
    * TorqueFL = const(0)
    * TorqueFR = const(0)
    * TorqueRL = const(0)
    * TorqueRR = const(0)
    * ExternalFx = const(0)
    * ExternalFy = const(0)
    * ExternalMz = const(0)
    * DownforceF = const(0)
    * DownforceR = const(0)
    * RoadHeightFL = const(0)
    * RoadHeightFR = const(0)
    * RoadHeightRL = const(0)
    * RoadHeightRR = const(0)
    * RoadVelocityFL = const(0)
    * RoadVelocityFR = const(0)
    * RoadVelocityRL = const(0)
    * RoadVelocityRR = const(0)
  When simulate for 200ms in Normal mode
  Then outputs
    * ZeroRoll: Roll == [-1e-10 .. 1e-10]
    * ZeroPitch: Pitch == [-1e-10 .. 1e-10]
    * ZeroHeave: Heave == [-1e-10 .. 1e-10]
    * ZeroVerticalAcceleration: VerticalAcceleration == [-1e-10 .. 1e-10]
