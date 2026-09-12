function [referenceYawRate, yawRateError, desiredYawMoment, ...
    tvActive, controllerSaturated, integratorNext] = ...
    torqueVectoringYawControlStep(steeringAngle, longitudinalSpeed, yawRate, ...
    enableTV, imuValid, integratorState, sampleTime, wheelbase, ...
    understeerGradient, minimumControlSpeed, frictionEstimate, gravity, ...
    lateralAccelerationLimit, yawRateLimit, proportionalGain, ...
    integralGain, antiWindupGain, yawMomentLimit)
%TorqueVectoringYAWCONTROLSTEP Discrete TV1 reference-yaw and PI feedback controller.
%   Angles are rad, speed is m/s, yaw rates are rad/s, and the requested
%   yaw moment is N*m. Positive steering, yaw rate, and yaw moment all
%   correspond to a left turn under the project SAE coordinate convention.

referenceYawRate = 0.0;
yawRateError = 0.0;
desiredYawMoment = 0.0;
tvActive = false;
controllerSaturated = false;
integratorNext = 0.0;

if ~imuValid
    return
end

speedMagnitude = abs(longitudinalSpeed);
protectedSpeed = max(speedMagnitude, minimumControlSpeed);
referenceDenominator = max( ...
    wheelbase + understeerGradient * longitudinalSpeed^2, ...
    0.1 * wheelbase);
unboundedYawRate = longitudinalSpeed * steeringAngle / ...
    referenceDenominator;

frictionYawRateLimit = max(frictionEstimate, 0.0) * gravity / ...
    protectedSpeed;
accelerationYawRateLimit = max(lateralAccelerationLimit, 0.0) / ...
    protectedSpeed;
activeYawRateLimit = min(abs(yawRateLimit), ...
    min(frictionYawRateLimit, accelerationYawRateLimit));
referenceYawRate = saturate(unboundedYawRate, activeYawRateLimit);

% Blend to zero below the protected speed instead of dividing by a
% near-zero vehicle speed or commanding a finite stationary yaw rate.
if speedMagnitude < minimumControlSpeed
    referenceYawRate = referenceYawRate * ...
        speedMagnitude / max(minimumControlSpeed, eps);
end

yawRateError = referenceYawRate - yawRate;
if ~enableTV
    return
end

tvActive = true;
unboundedYawMoment = proportionalGain * yawRateError + integratorState;
desiredYawMoment = saturate(unboundedYawMoment, abs(yawMomentLimit));
antiWindupError = desiredYawMoment - unboundedYawMoment;
integratorCandidate = integratorState + sampleTime * ...
    (integralGain * yawRateError + antiWindupGain * antiWindupError);
integratorNext = saturate(integratorCandidate, abs(yawMomentLimit));

controllerSaturated = ...
    abs(referenceYawRate - unboundedYawRate) > 1.0e-9 || ...
    abs(desiredYawMoment - unboundedYawMoment) > 1.0e-9 || ...
    abs(integratorNext - integratorCandidate) > 1.0e-9;
end

function output = saturate(input, limit)
limit = max(limit, 0.0);
output = min(limit, max(-limit, input));
end
