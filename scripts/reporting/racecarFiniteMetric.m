function value = racecarFiniteMetric(data, operation)
%RACECARFINITEMETRIC 忽略非有限值计算标量指标，不虚构缺失数据。

arguments
    data
    operation (1, 1) string
end

values = double(data(:));
values = values(isfinite(values));
if isempty(values)
    value = NaN;
    return
end
switch lower(operation)
    case "max"
        value = max(values);
    case "min"
        value = min(values);
    case "maxabs"
        value = max(abs(values));
    case "rms"
        value = sqrt(mean(values .^ 2));
    case "mean"
        value = mean(values);
    otherwise
        error("FSAE:Analysis:MetricOperation", ...
            "Unsupported metric operation: %s.", operation);
end
end
