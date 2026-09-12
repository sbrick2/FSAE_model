function used = lapResultUsesReferenceSpeed(result)
%LAPRESULTUSESREFERENCESPEED Identify the longitudinal-control contract.
%   AdaptiveAutocrossDriver keeps Track.ReferenceSpeed as a zero-valued bus
%   compatibility field, so plots must use explicit metadata/configuration
%   instead of inferring the mode from the numeric signal.

arguments
    result (1, 1) struct
end

if isfield(result, "Meta") && ...
        isfield(result.Meta, "ReferenceSpeedUsed")
    candidate = result.Meta.ReferenceSpeedUsed;
    if isscalar(candidate)
        used = logical(candidate);
        return
    end
end
if isfield(result, "Config") && isfield(result.Config, "Driver") && ...
        isfield(result.Config.Driver, "Model")
    used = lower(string(result.Config.Driver.Model)) ~= ...
        "adaptive_autocross";
    return
end
used = true;
end
