function sweep = createQuasiStaticParameterSweep(options)
%CREATEQuasiStaticPARAMETERSWEEP Create a traceable provisional QuasiStatic scan table.
%   Values are intentionally labeled as provisional design ranges. They are
%   not vehicle measurements and must be replaced before performance use.

arguments
    options.CdA (1, :) double = [0.50, 0.70, 0.90]
    options.ClAFront (1, :) double = [0.00, 0.05, 0.10]
    options.ClARear (1, :) double = [0.00, 0.05, 0.10]
    options.RollStiffnessDistributionFront (1, :) double = [0.40, 0.50, 0.60]
    options.GearEfficiency (1, :) double = [0.90, 0.95, 0.98]
end

names = [ ...
    "CdA"; "ClAFront"; "ClARear"; ...
    "RollStiffnessDistributionFront"; "GearEfficiency"];
paths = [ ...
    "Aero.CdA"; "Aero.ClAFront"; "Aero.ClARear"; ...
    "Vehicle.RollStiffnessDistributionFront"; "Powertrain.GearEfficiency"];
values = {options.CdA; options.ClAFront; options.ClARear; ...
    options.RollStiffnessDistributionFront; options.GearEfficiency};
units = ["m^2"; "m^2"; "m^2"; "1"; "1"];
source = repmat("QuasiStatic provisional scan range; not measured", numel(names), 1);
sweep = table(names, paths, values, units, source, ...
    VariableNames = ["Name", "Path", "Values", "Unit", "Source"]);
end
