function parameters = loadTorqueVectoringVehicleParameters(options)
%LOADTorqueVectoringVEHICLEPARAMETERS Read shared physical parameters for TorqueVectoring verification.

arguments
    options.DictionaryPath (1, 1) string = defaultDictionaryPath()
end

assert(isfile(options.DictionaryPath), "FSAE:TorqueVectoring:MissingDictionary", ...
    "Vehicle data dictionary was not found: %s", options.DictionaryPath);
dictionary = Simulink.data.dictionary.open(options.DictionaryPath);
cleanup = onCleanup(@() close(dictionary));
designData = getSection(dictionary, "Design Data");

groupNames = ["Vehicle", "Tire", "Aero", "Powertrain", "Battery", "Brake"];
parameters = struct;
for groupName = groupNames
    parameters.(char(groupName)) = getValue( ...
        getEntry(designData, char(groupName)));
end
end

function path = defaultDictionaryPath()
projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
path = fullfile(projectRoot, "data", "VehicleData.sldd");
end
