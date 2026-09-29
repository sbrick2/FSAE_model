function value = raceReplayValue(data, path, fallback)
%RACEREPLAYVALUE Read a nested field or a parameter-record Value.
arguments
    data
    path (1, 1) string
    fallback = []
end
value = data;
for part = split(path, ".").'
    if ~isstruct(value) || ~isscalar(value) || ~isfield(value, part)
        value = fallback;
        return
    end
    value = value.(part);
end
if isa(value, "Simulink.Parameter")
    value = value.Value;
elseif isstruct(value) && isscalar(value) && isfield(value, "Value")
    value = value.Value;
end
if isempty(value)
    value = fallback;
end
end
