function result = solveSteadyState(config, vehicleSpeed, targetAcceleration, options)
%SOLVESTEADYSTATE Solve one quasi-steady four-wheel force allocation.
%   RESULT = SOLVESTEADYSTATE(CONFIG, SPEED, [AX AY]) enforces body-force
%   balance, yaw-moment balance, quasi-static wheel loads, a TTC/MF tire
%   force envelope, motor torque and battery power limits. The solver is an
%   offline GGV primitive; it is not a replacement for the PathTracking time-domain
%   VehiclePlant. Front-wheel steering is a fixed optional study input and
%   defaults to zero; target lateral acceleration never implies a turn radius.

arguments
    config (1, 1) struct
    vehicleSpeed (1, 1) double {mustBeNonnegative, mustBeFinite}
    targetAcceleration (1, 2) double {mustBeFinite}
    options.SteeringAngle (1, 1) double {mustBeFinite} = 0.0
    options.TargetYawMoment (1, 1) double = 0.0
    options.CamberAngle (4, 1) double = zeros(4, 1)
    options.RoadGripScale (1, :) double = []
    options.RoadMuLimit (1, :) double = []
    options.TireEnvelopeLookup (1, 1) struct = struct()
    options.AssumeTireEnvelopeLookupValidated (1, 1) logical = false
end

ensureQuasiStaticTirePath();
speed = double(vehicleSpeed);
ax = double(targetAcceleration(1));
ay = double(targetAcceleration(2));
if isempty(options.RoadGripScale)
    roadGripScale = expandFour(config.Environment.RoadGripScale);
else
    roadGripScale = expandFour(options.RoadGripScale);
end
if isempty(options.RoadMuLimit)
    roadMuLimit = expandFour(config.Environment.RoadMuLimit);
else
    roadMuLimit = expandFour(options.RoadMuLimit);
end
camber = options.CamberAngle;

geometry = config.Vehicle;
steeringAngle = options.SteeringAngle;
steeringAngle = max(-pi / 2, min(pi / 2, steeringAngle));
wheelSteerAngle = [steeringAngle; steeringAngle; 0.0; 0.0];

aero = computeAero(config, speed);
normalLoad = computeNormalLoads(config, ax, ay, aero);
wheelPosition = [ ...
    geometry.CGToFrontAxle, geometry.TrackFront / 2; ...
    geometry.CGToFrontAxle, -geometry.TrackFront / 2; ...
    -geometry.CGToRearAxle, geometry.TrackRear / 2; ...
    -geometry.CGToRearAxle, -geometry.TrackRear / 2];

if any(normalLoad <= 0.0) || any(~isfinite(normalLoad))
    result = infeasibleResult(config, speed, [ax, ay], steeringAngle, ...
        aero, normalLoad, "Wheel lift-off or non-finite normal load.");
    return
end

if isempty(fieldnames(options.TireEnvelopeLookup))
    envelopes = evaluateTireEnvelopeMF62(config, normalLoad, ...
        roadGripScale, roadMuLimit, camber);
    tireEnvelopeSource = "Exact MF62 combined-slip evaluation";
else
    if ~options.AssumeTireEnvelopeLookupValidated
        validateTireEnvelopeLookup(config, options.TireEnvelopeLookup, ...
            roadGripScale, roadMuLimit, camber);
    end
    envelopes = interpolateTireEnvelopeLookup( ...
        options.TireEnvelopeLookup, normalLoad);
    tireEnvelopeSource = string(options.TireEnvelopeLookup.Method);
end

required = [ ...
    geometry.Mass * ax + aero.DragForce; ...
    geometry.Mass * ay - aero.SideForce; ...
    options.TargetYawMoment - aero.YawMoment];

[aIneq, bIneq, powerLimits] = buildForceConstraints( ...
    config, speed, wheelSteerAngle, envelopes);
aEq = [ ...
    1, 0, 1, 0, 1, 0, 1, 0; ...
    0, 1, 0, 1, 0, 1, 0, 1; ...
    -wheelPosition(1, 2), wheelPosition(1, 1), ...
    -wheelPosition(2, 2), wheelPosition(2, 1), ...
    -wheelPosition(3, 2), wheelPosition(3, 1), ...
    -wheelPosition(4, 2), wheelPosition(4, 1)];

if config.Solver.AllocationMode == "FAST"
    [forceVector, exitFlag, output] = fastForceAllocation( ...
        aEq, required, aIneq, bIneq, normalLoad);
else
    objective = zeros(8, 1);
    optionsLinprog = optimoptions("linprog", Display = "off");
    [forceVector, ~, exitFlag, output] = linprog( ...
        objective, aIneq, bIneq, aEq, required, [], [], optionsLinprog);
end
if isempty(forceVector)
    forceVector = zeros(8, 1);
end

bodyForce = reshape(forceVector, 2, 4)';
wheelForce = zeros(4, 2);
for wheel = 1:4
    c = cos(wheelSteerAngle(wheel));
    s = sin(wheelSteerAngle(wheel));
    wheelForce(wheel, :) = ([c, s; -s, c] * bodyForce(wheel, :)')';
end

actual = aEq * forceVector;
residual = actual - required;
utilization = zeros(4, 1);
for wheel = 1:4
    directionalRatio = envelopes(wheel).Directions * ...
        wheelForce(wheel, :)' ./ max(envelopes(wheel).Support, 1.0);
    utilization(wheel) = max(0.0, max(directionalRatio));
end

scale = max(1.0, norm(required, Inf));
equilibriumOk = max(abs(residual)) <= ...
    config.Solver.EquilibriumTolerance * scale;
feasible = exitFlag >= 1 && equilibriumOk && all(utilization <= 1.0 + 1.0e-5);
constraintActive = activeConstraints(config, speed, ax, required(1), ...
    utilization, powerLimits, feasible);

result = struct( ...
    "Feasible", logical(feasible), ...
    "ExitFlag", double(exitFlag), ...
    "SolverOutput", output, ...
    "VehicleSpeed", speed, ...
    "TargetAcceleration", [ax, ay], ...
    "RequiredBodyForce", required(1:2), ...
    "ActualBodyForce", actual(1:2), ...
    "RequiredYawMoment", required(3), ...
    "ActualYawMoment", actual(3), ...
    "EquilibriumResidual", residual, ...
    "SteeringAngle", steeringAngle, ...
    "WheelSteerAngle", wheelSteerAngle, ...
    "WheelPosition", wheelPosition, ...
    "NormalLoad", normalLoad, ...
    "BodyForce", bodyForce, ...
    "WheelForce", wheelForce, ...
    "TireFxMax", reshape([envelopes.FxMax], [], 1), ...
    "TireFyMax", reshape([envelopes.FyMax], [], 1), ...
    "TireEnvelopeMethod", "MF62 combined-slip directional support", ...
    "TireEnvelopeSource", tireEnvelopeSource, ...
    "TireUtilization", utilization, ...
    "Aero", aero, ...
    "PowertrainLimits", powerLimits, ...
    "ConstraintActive", constraintActive, ...
    "FailureReason", failureReason(feasible, exitFlag, equilibriumOk, ...
        normalLoad, output));
end

function [aIneq, bIneq, limits] = buildForceConstraints( ...
        config, speed, wheelSteerAngle, envelopes)
aIneq = zeros(0, 8);
bIneq = zeros(0, 1);
for wheel = 1:4
    c = cos(wheelSteerAngle(wheel));
    s = sin(wheelSteerAngle(wheel));
    bodyFromWheel = [c, s; -s, c];
    localA = envelopes(wheel).Directions;
    wheelRows = zeros(size(localA, 1), 8);
    wheelRows(:, 2 * wheel - 1:2 * wheel) = ...
        localA * bodyFromWheel;
    aIneq = [aIneq; wheelRows]; %#ok<AGROW>
    bIneq = [bIneq; envelopes(wheel).Support]; %#ok<AGROW>
end

limits = computePowertrainLimits(config, speed);
totalFxRow = repmat([1, 0], 1, 4);
aIneq = [aIneq; totalFxRow; -totalFxRow];
bIneq = [bIneq; limits.MaxDriveForce; limits.MinBrakeForce];
end

function [forceVector, exitFlag, output] = fastForceAllocation( ...
        aEq, required, aIneq, bIneq, normalLoad)
%FASTFORCEALLOCATION Weighted closed-form allocation for dense GGV scans.
%   The full LP remains available for SteadyStateBalance point studies. The fast mode uses
%   the same tire and power inequalities but a normal-load-weighted force
%   allocation, which is deterministic and substantially cheaper in a scan.
weight = diag(repelem(max(normalLoad, 1.0), 2));
normalMatrix = aEq * weight * aEq';
if rcond(normalMatrix) < 1.0e-12
    forceVector = zeros(8, 1);
    exitFlag = -2;
    output = struct("message", "Weighted equilibrium matrix is singular.");
    return
end
forceVector = weight * aEq' / normalMatrix * required;
constraintResidual = aIneq * forceVector - bIneq;
if all(constraintResidual <= 1.0e-8)
    exitFlag = 1;
    output = struct("message", "Fast normal-load-weighted allocation.");
else
    exitFlag = -2;
    output = struct("message", "Fast allocation violates a tire or power constraint.");
end
end

function limits = computePowertrainLimits(config, speed)
powertrain = config.Powertrain;
battery = config.Battery;
radius = config.Tire.EffectiveRadius;
motorSpeed = abs(speed) / radius * powertrain.GearRatio;
speedAvailable = double(motorSpeed <= powertrain.MotorSpeedLimit);
wheelForceByTorque = 4.0 * powertrain.MotorTorqueLimit * ...
    powertrain.GearRatio * powertrain.GearEfficiency / radius * speedAvailable;
powerEfficiency = powertrain.GearEfficiency * powertrain.InverterEfficiency;
powerSpeed = max(abs(speed), config.Solver.PowerSpeedEpsilon);
wheelForceByDrivePower = battery.PowerLimitDrive * powerEfficiency / powerSpeed;
% Map the battery-side charge limit back to the wheel side. Regen power
% crosses the gearbox and inverter in the opposite direction to drive power.
wheelForceByRegenPower = battery.PowerLimitRegen / ...
    (powerEfficiency * powerSpeed);
maxBrakeForce = config.Brake.MaxTotalForce;
limits = struct( ...
    "MotorSpeed", motorSpeed, ...
    "MotorSpeedAvailable", logical(speedAvailable), ...
    "MotorForceLimit", wheelForceByTorque, ...
    "DrivePowerForceLimit", wheelForceByDrivePower, ...
    "RegenPowerForceLimit", wheelForceByRegenPower, ...
    "FrictionBrakeForceLimit", maxBrakeForce, ...
    "MaxDriveForce", min(wheelForceByTorque, wheelForceByDrivePower), ...
    "MinBrakeForce", min(wheelForceByTorque + maxBrakeForce, ...
        wheelForceByRegenPower + maxBrakeForce));
end

function aero = computeAero(config, speed)
wind = config.Environment.WindVelocityBody;
relative = [speed; 0.0] - wind;
relativeSpeed = norm(relative);
q = 0.5 * config.Environment.AirDensity * relativeSpeed^2;
beta = atan2(relative(2), max(abs(relative(1)), config.Solver.PowerSpeedEpsilon));
yawScale = max(0.0, 1.0 + config.Aero.YawCorrection * beta^2);
yawDegrees = rad2deg(abs(beta));
yawDegrees = min(max(yawDegrees, config.Aero.YawAngleGrid(1)), ...
    config.Aero.YawAngleGrid(end));
downforceYawScale = interp1(config.Aero.YawAngleGrid, ...
    config.Aero.YawDownforceScale, yawDegrees, "linear");
aero = struct( ...
    "RelativeAirVelocityBody", relative, ...
    "RelativeAirSpeed", relativeSpeed, ...
    "DynamicPressure", q, ...
    "DragForce", q * config.Aero.CdA * yawScale, ...
    "SideForce", 0.0, ...
    "DownforceFront", q * config.Aero.ClAFront * downforceYawScale, ...
    "DownforceRear", q * config.Aero.ClARear * downforceYawScale, ...
    "YawMoment", 0.0);
end

function normalLoad = computeNormalLoads(config, ax, ay, aero)
vehicle = config.Vehicle;
if isfield(vehicle, "DynamicsModel") && vehicle.DynamicsModel == "10DOF"
    normalLoad = compute10DOFStaticNormalLoads(config, ax, ay, aero);
    return
end
mass = vehicle.Mass;
gravity = config.Solver.Gravity;
front = mass * gravity * vehicle.CGToRearAxle / vehicle.Wheelbase - ...
    mass * ax * vehicle.CGHeight / vehicle.Wheelbase + aero.DownforceFront;
rear = mass * gravity * vehicle.CGToFrontAxle / vehicle.Wheelbase + ...
    mass * ax * vehicle.CGHeight / vehicle.Wheelbase + aero.DownforceRear;
frontTransfer = vehicle.RollStiffnessDistributionFront * mass * ay * ...
    vehicle.CGHeight / vehicle.TrackFront;
rearTransfer = (1.0 - vehicle.RollStiffnessDistributionFront) * mass * ay * ...
    vehicle.CGHeight / vehicle.TrackRear;
normalLoad = [ ...
    front / 2.0 - frontTransfer; ...
    front / 2.0 + frontTransfer; ...
    rear / 2.0 - rearTransfer; ...
    rear / 2.0 + rearTransfer];
end

function normalLoad = compute10DOFStaticNormalLoads(config, ax, ay, aero)
% Solve the zero-velocity equilibrium of the Vehicle10DOF heave/roll/pitch states.
% The nonlinear damper force is zero in a quasi-static operating point.
vehicle = config.Vehicle;
suspension = config.Suspension;
distanceFront = vehicle.CGToFrontAxle;
distanceRear = vehicle.CGToRearAxle;
xCorner = [distanceFront; distanceFront; -distanceRear; -distanceRear];
yCorner = [vehicle.TrackFront / 2.0; -vehicle.TrackFront / 2.0; ...
    vehicle.TrackRear / 2.0; -vehicle.TrackRear / 2.0];
geometry = [ones(4, 1), yCorner, -xCorner];

forcePerState = -diag(suspension.WheelRate) * geometry;
forcePerState(:, 2) = forcePerState(:, 2) + [ ...
    -suspension.AdditionalRollStiffness(1) / vehicle.TrackFront; ...
    suspension.AdditionalRollStiffness(1) / vehicle.TrackFront; ...
    -suspension.AdditionalRollStiffness(2) / vehicle.TrackRear; ...
    suspension.AdditionalRollStiffness(2) / vehicle.TrackRear];
equilibriumMatrix = geometry.' * forcePerState;
assert(rcond(equilibriumMatrix) > 1.0e-12, ...
    "FSAE:QuasiStatic:Singular10DOFSuspension", ...
    "The 10DOF static suspension equilibrium matrix is singular.");

mass = vehicle.Mass;
gravity = config.Solver.Gravity;
generalizedLoadTarget = [ ...
    mass * gravity + aero.DownforceFront + aero.DownforceRear; ...
    -mass * ay * vehicle.CGHeight; ...
    mass * ax * vehicle.CGHeight - distanceFront * aero.DownforceFront + ...
        distanceRear * aero.DownforceRear];
staticGeneralizedLoad = geometry.' * suspension.StaticNormalLoad;
bodyState = equilibriumMatrix \ ...
    (generalizedLoadTarget - staticGeneralizedLoad);
normalLoad = suspension.StaticNormalLoad + forcePerState * bodyState;
end

function active = activeConstraints(~, speed, ax, requiredFx, utilization, limits, feasible)
tolerance = 1.0e-3;
active = struct( ...
    "Tire", any(utilization >= 1.0 - tolerance), ...
    "MotorTorque", abs(requiredFx) >= limits.MotorForceLimit * (1.0 - tolerance), ...
    "DrivePower", ax >= 0.0 && requiredFx >= limits.DrivePowerForceLimit * (1.0 - tolerance), ...
    "RegenPower", ax < 0.0 && -requiredFx >= limits.RegenPowerForceLimit * (1.0 - tolerance), ...
    "FrictionBrake", ax < 0.0 && -requiredFx > limits.RegenPowerForceLimit, ...
    "MotorSpeed", ~limits.MotorSpeedAvailable);
labels = strings(0, 1);
names = string(fieldnames(active));
for index = 1:numel(names)
    if active.(names(index))
        labels(end + 1, 1) = names(index); %#ok<AGROW>
    end
end
active.Labels = labels;
active.EquilibriumSatisfied = logical(feasible);
active.LongitudinalAcceleration = ax;
active.VehicleSpeed = speed;
end

function result = infeasibleResult(~, speed, target, steeringAngle, aero, normalLoad, reason)
result = struct( ...
    "Feasible", false, "ExitFlag", -1, "SolverOutput", struct(), ...
    "VehicleSpeed", speed, "TargetAcceleration", target, ...
    "RequiredBodyForce", [NaN, NaN], "ActualBodyForce", [NaN, NaN], ...
    "RequiredYawMoment", NaN, "ActualYawMoment", NaN, ...
    "EquilibriumResidual", [NaN; NaN; NaN], ...
    "SteeringAngle", steeringAngle, ...
    "WheelSteerAngle", [steeringAngle; steeringAngle; 0; 0], ...
    "WheelPosition", zeros(4, 2), "NormalLoad", normalLoad, ...
    "BodyForce", NaN(4, 2), "WheelForce", NaN(4, 2), ...
    "TireFxMax", NaN(4, 1), "TireFyMax", NaN(4, 1), ...
    "TireUtilization", NaN(4, 1), "Aero", aero, ...
    "PowertrainLimits", struct(), "ConstraintActive", struct(Labels = strings(0, 1)), ...
    "FailureReason", string(reason));
end

function ensureQuasiStaticTirePath()
persistent tirePathInitialized
if ~isempty(tirePathInitialized) && tirePathInitialized
    return
end
if exist("evaluateTireMF62", "file") == 2 && ...
        exist("applyRoadGripAndLimit", "file") == 2
    tirePathInitialized = true;
    return
end
projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
addpath(fullfile(projectRoot, "scripts", "tire"));
tirePathInitialized = true;
end

function values = expandFour(values)
values = double(values(:));
if isscalar(values)
    values = repmat(values, 4, 1);
end
assert(numel(values) == 4, "FSAE:QuasiStatic:FourWheelInput", ...
    "The input must be scalar or contain four wheel values.");
end

function reason = failureReason(feasible, exitFlag, equilibriumOk, normalLoad, output)
if feasible
    reason = "";
elseif any(normalLoad <= 0.0)
    reason = "Wheel lift-off or negative normal load.";
elseif exitFlag <= 0
    if isstruct(output) && isfield(output, "message")
        reason = string(output.message);
    else
        reason = "Linear force allocation did not find a feasible point.";
    end
elseif ~equilibriumOk
    reason = "Force or yaw-moment residual exceeds the equilibrium tolerance.";
else
    reason = "Tire force envelope or powertrain constraint was violated.";
end
end
