function pedals = deriveRaceReplayDriverRequests(driver, config)
%DERIVERACEREPLAYDRIVERREQUESTS Recorded or equivalent driver pedal requests.
arguments
    driver (1, 1) struct
    config (1, 1) struct
end
pedals = struct("Accelerator", NaN, "Brake", NaN, ...
    "Source", "请求满量程不可用", "Equivalent", true);
accelerator = raceReplayValue(driver, "AcceleratorPedalRequest", NaN);
brake = raceReplayValue(driver, "BrakePedalRequest", NaN);
if isfinite(accelerator) || isfinite(brake)
    pedals.Accelerator = percent(accelerator, 1);
    pedals.Brake = percent(brake, 1);
    pedals.Source = "原始踏板请求";
    pedals.Equivalent = false;
    return
end
mode = lower(string(raceReplayValue(config, "Model", "")));
acceleration = raceReplayValue(driver, "AccelerationRequest", NaN);
if any(mode == ["adaptive_autocross", "reference_speed"])
    positive = NaN;
    negative = NaN;
    if isfinite(acceleration)
        positive = max(acceleration, 0);
        negative = max(-acceleration, 0);
    end
    pedals.Accelerator = percent(positive, ...
        raceReplayValue(config, "MaximumAcceleration", NaN));
    pedals.Brake = percent(negative, ...
        raceReplayValue(config, "MaximumDeceleration", NaN));
    pedals.Source = "纵向加速度请求 / 驾驶员满量程（等效）";
else
    pedals.Accelerator = percent(raceReplayValue(driver, "DriveTorqueRequest", NaN), ...
        raceReplayValue(config, "MaximumDriveTorque", NaN));
    pedals.Brake = percent(raceReplayValue(driver, "BrakePressureRequest", NaN), ...
        raceReplayValue(config, "MaximumBrakePressure", NaN));
    pedals.Source = "转矩 / 压力请求归一化（等效）";
end
end

function value = percent(request, maximum)
value = NaN;
if isscalar(request) && isscalar(maximum) && isfinite(request) && ...
        isfinite(maximum) && maximum > 0
    value = 100 * min(1, max(0, request / maximum));
end
end
