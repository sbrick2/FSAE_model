function catalog = listQuasiStaticSweepParameters(projectRoot)
%LISTQUASISTATICSWEEPPARAMETERS List every scalar dictionary parameter used by QuasiStatic.
%   CATALOG is derived from the current VehicleData.sldd values. Vector
%   parameters are intentionally excluded because the sweep runner changes
%   one real numeric scalar per axis.

arguments
    projectRoot (1, 1) string = defaultProjectRoot()
end

definitions = supportedParameterDefinitions();
dictionaryPath = fullfile(projectRoot, "data", "VehicleData.sldd");
assert(isfile(dictionaryPath), "FSAE:QuasiStatic:MissingDictionary", ...
    "VehicleData.sldd was not found: %s", dictionaryPath);
dictionary = Simulink.data.dictionary.open(dictionaryPath);
cleanup = onCleanup(@() close(dictionary));
designData = getSection(dictionary, "Design Data");

labels = strings(0, 1);
paths = strings(0, 1);
units = strings(0, 1);
currentValues = zeros(0, 1);
startValues = zeros(0, 1);
stopValues = zeros(0, 1);
stepValues = zeros(0, 1);
for index = 1:height(definitions)
    path = definitions.Path(index);
    parts = split(path, ".");
    group = getValue(getEntry(designData, char(parts(1))));
    fieldName = char(parts(2));
    if ~isstruct(group) || ~isfield(group, fieldName)
        continue
    end
    record = group.(fieldName);
    [value, unit] = unwrapDictionaryParameter( ...
        record, definitions.Unit(index));
    if ~(isnumeric(value) && isreal(value) && isscalar(value) && ...
            isfinite(value))
        continue
    end
    [startValue, stopValue, stepValue] = defaultSweepRange( ...
        double(value), definitions.LowerBound(index), ...
        definitions.UpperBound(index));
    labels(end + 1, 1) = definitions.Name(index) + " | " + path; %#ok<AGROW>
    paths(end + 1, 1) = path; %#ok<AGROW>
    units(end + 1, 1) = unit; %#ok<AGROW>
    currentValues(end + 1, 1) = double(value); %#ok<AGROW>
    startValues(end + 1, 1) = startValue; %#ok<AGROW>
    stopValues(end + 1, 1) = stopValue; %#ok<AGROW>
    stepValues(end + 1, 1) = stepValue; %#ok<AGROW>
end

catalog = table(labels, paths, units, currentValues, startValues, ...
    stopValues, stepValues, 'VariableNames', ...
    {'Label', 'Path', 'Unit', 'CurrentValue', 'Start', 'Stop', 'Step'});
assert(~isempty(catalog), "FSAE:QuasiStatic:EmptySweepCatalog", ...
    "No finite scalar QuasiStatic parameters were found in VehicleData.sldd.");
end

function definitions = supportedParameterDefinitions()
names = [ ...
    "整车质量"; "传动比"; "轴距"; "前轮距"; "后轮距"; ...
    "质心高度"; "质心至前轴距离"; "前轴侧倾刚度分配"; ...
    "轮胎有效半径"; "轮胎低速正则阈值"; ...
    "阻力面积"; "前轴下压力面积"; "后轴下压力面积"; ...
    "气动偏航修正"; "传动效率"; "电机扭矩上限"; ...
    "电机转速上限"; "逆变器效率"; ...
    "电池驱动功率上限"; "电池再生功率上限"; "最大总制动力"];
paths = [ ...
    "Vehicle.Mass"; "Powertrain.GearRatio"; "Vehicle.Wheelbase"; ...
    "Vehicle.TrackFront"; "Vehicle.TrackRear"; "Vehicle.CGHeight"; ...
    "Vehicle.CGToFrontAxle"; ...
    "Vehicle.RollStiffnessDistributionFront"; ...
    "Tire.EffectiveRadius"; "Tire.LowSpeedEpsilon"; ...
    "Aero.CdA"; "Aero.ClAFront"; "Aero.ClARear"; ...
    "Aero.YawCorrection"; "Powertrain.GearEfficiency"; ...
    "Powertrain.MotorTorqueLimit"; "Powertrain.MotorSpeedLimit"; ...
    "Powertrain.InverterEfficiency"; "Battery.PowerLimitDrive"; ...
    "Battery.PowerLimitRegen"; "Brake.MaxTotalForce"];
units = [ ...
    "kg"; "1"; "m"; "m"; "m"; "m"; "m"; "1"; ...
    "m"; "m/s"; "m^2"; "m^2"; "m^2"; "1"; "1"; ...
    "N*m"; "rad/s"; "1"; "W"; "W"; "N"];
lowerBounds = [ ...
    0; 0; 0; 0; 0; 0; 0; 0; 0; 0; ...
    0; 0; 0; -Inf; 0; 0; 0; 0; 0; 0; 0];
upperBounds = [ ...
    Inf; Inf; Inf; Inf; Inf; Inf; Inf; 1; Inf; Inf; ...
    Inf; Inf; Inf; Inf; 1; Inf; Inf; 1; Inf; Inf; Inf];
definitions = table(names, paths, units, lowerBounds, upperBounds, ...
    'VariableNames', {'Name', 'Path', 'Unit', 'LowerBound', 'UpperBound'});
end

function [value, unit] = unwrapDictionaryParameter(record, fallbackUnit)
value = record;
unit = fallbackUnit;
if isstruct(record) && isfield(record, "Value")
    value = record.Value;
    if isfield(record, "Unit") && strlength(string(record.Unit)) > 0
        unit = string(record.Unit);
    end
end
end

function [startValue, stopValue, stepValue] = defaultSweepRange( ...
        currentValue, lowerBound, upperBound)
if currentValue == 0.0
    span = 0.1;
else
    span = 0.1 * abs(currentValue);
end
startValue = max(lowerBound, currentValue - span);
stopValue = min(upperBound, currentValue + span);
if stopValue <= startValue
    stopValue = min(upperBound, startValue + max(span, 1.0e-6));
end
assert(stopValue > startValue, "FSAE:QuasiStatic:SweepDefaultRange", ...
    "Could not create a default sweep range around %.6g.", currentValue);
stepValue = (stopValue - startValue) / 4.0;
end

function root = defaultProjectRoot()
root = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
end
