function result = deriveAdaptivePerformanceMetrics(result, driverConfig)
%DERIVEADAPTIVEPERFORMANCEMETRICS Summarize usable closed-loop performance.

arguments
    result (1, 1) struct
    driverConfig (1, 1) struct
end

result.Metrics.MaximumSpeed = finiteMetric(result.Vehicle.Speed, "max");
result.Metrics.MinimumSpeed = finiteMetric(result.Vehicle.Speed, "min");
result.Metrics.P99AbsLongitudinalAcceleration = ...
    finitePercentile(abs(result.Vehicle.Ax), 0.99);
result.Metrics.P99AbsLateralAcceleration = ...
    finitePercentile(abs(result.Vehicle.Ay), 0.99);
result.Metrics.MaximumAbsSteeringRackRequest = finiteMetric( ...
    abs(result.Driver.SteeringRackAngleRequest), "max");
result.Metrics.SteeringAngleSaturationFraction = activeFraction( ...
    abs(result.Driver.SteeringRackAngleRequest) >= ...
    0.995 * driverConfig.MaximumSteeringAngle);
result.Metrics.SteeringRateSaturationFraction = ...
    steeringRateSaturationFraction(result, ...
    driverConfig.MaximumSteeringRate);
result.Metrics.ControllerSaturationFraction = activeFraction( ...
    result.Controller.ControllerSaturated);
result.Metrics.AccelerationRequestSaturationFraction = activeFraction( ...
    result.Driver.LongitudinalAccelerationRequest >= ...
    0.995 * driverConfig.MaximumAcceleration);
result.Metrics.DecelerationRequestSaturationFraction = activeFraction( ...
    result.Driver.LongitudinalAccelerationRequest <= ...
    -0.995 * driverConfig.MaximumDeceleration);
end

function value = finiteMetric(data, operation)
finiteData = double(data(isfinite(data)));
if isempty(finiteData)
    value = NaN;
elseif operation == "max"
    value = max(finiteData);
else
    value = min(finiteData);
end
end

function value = finitePercentile(data, probability)
finiteData = sort(double(data(isfinite(data))));
if isempty(finiteData)
    value = NaN;
    return
end
coordinate = 1 + probability * (numel(finiteData) - 1);
lowerIndex = floor(coordinate);
upperIndex = ceil(coordinate);
fraction = coordinate - lowerIndex;
value = finiteData(lowerIndex) + fraction * ...
    (finiteData(upperIndex) - finiteData(lowerIndex));
end

function fraction = activeFraction(active)
finite = isfinite(active);
if ~any(finite)
    fraction = NaN;
else
    fraction = mean(logical(active(finite)));
end
end

function fraction = steeringRateSaturationFraction(result, rateLimit)
steering = double(result.Driver.SteeringRackAngleRequest(:));
time = double(result.Time(:));
if numel(steering) ~= numel(time) || numel(time) < 2
    fraction = NaN;
    return
end
deltaTime = diff(time);
valid = isfinite(deltaTime) & deltaTime > 0 & ...
    isfinite(steering(1:end - 1)) & isfinite(steering(2:end));
if ~any(valid)
    fraction = NaN;
    return
end
deltaSteering = diff(steering);
rate = abs(deltaSteering(valid)) ./ deltaTime(valid);
fraction = mean(rate >= 0.995 * rateLimit);
end
