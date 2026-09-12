function vehicle10DOFSuspensionSFunction(block)
%Vehicle10DOFSUSPENSIONSFUNCTION Three-DOF heave, roll, and pitch suspension model.
%   The planar body and four wheel-rotation DOFs remain in Vehicle7DOF.
%   This Level-2 MATLAB S-function adds the three sprung-body DOFs required
%   by the Vehicle10DOF 10DOF plant and evaluates the nonlinear TTX25 MkII damper map.

setup(block);
end

function setup(block)
block.NumDialogPrms = 1;
block.NumInputPorts = 6;
block.NumOutputPorts = 12;
block.NumContStates = 6;
block.SampleTimes = [0 0];
block.SimStateCompliance = "DefaultSimState";

inputDimensions = {1, 1, 1, 1, 4, 4};
for index = 1:block.NumInputPorts
    block.InputPort(index).Dimensions = inputDimensions{index};
    block.InputPort(index).DatatypeID = 0;
    block.InputPort(index).Complexity = "Real";
    block.InputPort(index).DirectFeedthrough = true;
end

outputDimensions = {1, 1, 1, 1, 4, 4, 4, 4, 4, 1, 1, 1};
for index = 1:block.NumOutputPorts
    block.OutputPort(index).Dimensions = outputDimensions{index};
    block.OutputPort(index).DatatypeID = 0;
    block.OutputPort(index).Complexity = "Real";
end

block.RegBlockMethod("CheckParameters", @checkParameters);
block.RegBlockMethod("InitializeConditions", @initializeConditions);
block.RegBlockMethod("Outputs", @outputs);
block.RegBlockMethod("Derivatives", @derivatives);
end

function checkParameters(block)
parameters = block.DialogPrm(1).Data;
if isempty(parameters)
    return
end
requiredFields = ["Mass", "Gravity", "InertiaRoll", "InertiaPitch", ...
    "CGHeight", "CGToFrontAxle", "Wheelbase", "TrackFront", ...
    "TrackRear", "WheelRate", "MotionRatio", "StaticNormalLoad", ...
    "DamperVelocityBreakpoints", "DamperCompressionForce", ...
    "DamperReboundForce", "AdditionalRollStiffness", "InitialState"];
for field = requiredFields
    assert(isfield(parameters, field), "FSAE:Vehicle10DOF:MissingParameter", ...
        "Vehicle10DOFSuspension.%s is required.", field);
end
assert(all(diff(parameters.DamperVelocityBreakpoints) > 0), ...
    "FSAE:Vehicle10DOF:DamperBreakpoints", ...
    "Damper velocity breakpoints must be strictly increasing.");
assert(numel(parameters.InitialState) == 6, "FSAE:Vehicle10DOF:InitialState", ...
    "Vehicle10DOF suspension initial state must contain six elements.");
end

function initializeConditions(block)
parameters = block.DialogPrm(1).Data;
block.ContStates.Data = parameters.InitialState(:);
end

function outputs(block)
[stateDerivative, normalLoad, camberAngle, suspensionDeflection, ...
    damperVelocity, suspensionForce] = calculateDynamics(block);
state = block.ContStates.Data;

block.OutputPort(1).Data = state(2);
block.OutputPort(2).Data = state(3);
block.OutputPort(3).Data = state(1);
block.OutputPort(4).Data = stateDerivative(4);
block.OutputPort(5).Data = normalLoad;
block.OutputPort(6).Data = camberAngle;
block.OutputPort(7).Data = suspensionDeflection;
block.OutputPort(8).Data = damperVelocity;
block.OutputPort(9).Data = suspensionForce;
block.OutputPort(10).Data = state(5);
block.OutputPort(11).Data = state(6);
block.OutputPort(12).Data = state(4);
end

function derivatives(block)
[stateDerivative, ~, ~, ~, ~, ~] = calculateDynamics(block);
block.Derivatives.Data = stateDerivative;
end

function [stateDerivative, normalLoad, camberAngle, suspensionDeflection, ...
        damperVelocity, suspensionForce] = calculateDynamics(block)
parameters = block.DialogPrm(1).Data;
state = block.ContStates.Data;

ax = block.InputPort(1).Data;
ay = block.InputPort(2).Data;
downforceFront = block.InputPort(3).Data;
downforceRear = block.InputPort(4).Data;
roadHeight = block.InputPort(5).Data(:);
roadVelocity = block.InputPort(6).Data(:);

distanceFront = parameters.CGToFrontAxle;
distanceRear = parameters.Wheelbase - distanceFront;
xCorner = [distanceFront; distanceFront; -distanceRear; -distanceRear];
yCorner = [parameters.TrackFront / 2; -parameters.TrackFront / 2; ...
    parameters.TrackRear / 2; -parameters.TrackRear / 2];

bodyCornerPosition = state(1) + yCorner .* state(2) - xCorner .* state(3);
bodyCornerVelocity = state(4) + yCorner .* state(5) - xCorner .* state(6);
suspensionDeflection = roadHeight - bodyCornerPosition;
wheelRelativeVelocity = roadVelocity - bodyCornerVelocity;
damperVelocity = wheelRelativeVelocity ./ parameters.MotionRatio;

springForce = parameters.WheelRate .* suspensionDeflection;
damperForce = evaluateDamperForce(damperVelocity, parameters) ./ ...
    parameters.MotionRatio;

rollAngle = state(2);
frontRollForce = parameters.AdditionalRollStiffness(1) * rollAngle / ...
    parameters.TrackFront;
rearRollForce = parameters.AdditionalRollStiffness(2) * rollAngle / ...
    parameters.TrackRear;
antiRollBarForce = [-frontRollForce; frontRollForce; ...
    -rearRollForce; rearRollForce];

suspensionForce = springForce + damperForce + antiRollBarForce;
normalLoad = max(parameters.NormalLoadFloor, ...
    parameters.StaticNormalLoad + suspensionForce);

verticalAcceleration = (sum(normalLoad) - parameters.Mass * ...
    parameters.Gravity - downforceFront - downforceRear) / parameters.Mass;
rollAcceleration = (dot(yCorner, normalLoad) + parameters.Mass * ...
    ay * parameters.CGHeight) / parameters.InertiaRoll;
pitchMoment = -dot(xCorner, normalLoad) + distanceFront * ...
    downforceFront - distanceRear * downforceRear - parameters.Mass * ...
    ax * parameters.CGHeight;
pitchAcceleration = pitchMoment / parameters.InertiaPitch;

stateDerivative = [state(4); state(5); state(6); verticalAcceleration; ...
    rollAcceleration; pitchAcceleration];

% K&C camber curves exist in the source document but were not supplied as
% numerical data. Keep the interface active without inventing a gain.
camberAngle = zeros(4, 1);
end

function force = evaluateDamperForce(velocity, parameters)
speed = min(abs(velocity), parameters.DamperVelocityBreakpoints(end));
compressionMagnitude = interp1(parameters.DamperVelocityBreakpoints, ...
    parameters.DamperCompressionForce, speed, "linear");
reboundMagnitude = interp1(parameters.DamperVelocityBreakpoints, ...
    parameters.DamperReboundForce, speed, "linear");

force = zeros(4, 1);
isCompression = velocity >= 0;
force(isCompression) = compressionMagnitude(isCompression);
force(~isCompression) = -reboundMagnitude(~isCompression);
end
