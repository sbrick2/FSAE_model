function receipt = updateVehicleParameters(changes, options)
%UPDATEVEHICLEPARAMETERS Validate and persist scalar vehicle parameter changes.
%   CHANGES is a table with Path and Value variables. The data dictionary is
%   backed up before its Design Data records are updated.

arguments
    changes table
    options.ProjectRoot (1, 1) string = defaultProjectRoot()
    options.DictionaryPath (1, 1) string = ""
    options.BackupFolder (1, 1) string = ""
    options.CreateBackup (1, 1) logical = true
end

requiredVariables = ["Path", "Value"];
assert(all(ismember(requiredVariables, ...
    string(changes.Properties.VariableNames))), ...
    "FSAE:VehicleParameters:InvalidChanges", ...
    "Changes must contain Path and Value table variables.");

paths = string(changes.Path);
values = double(changes.Value);
assert(numel(paths) == height(changes) && iscolumn(paths) && ...
    iscolumn(values), "FSAE:VehicleParameters:InvalidChanges", ...
    "Path and Value must contain one scalar entry per table row.");
assert(numel(unique(paths)) == numel(paths), ...
    "FSAE:VehicleParameters:DuplicatePath", ...
    "Each vehicle parameter path may appear only once.");

dictionaryPath = options.DictionaryPath;
if strlength(dictionaryPath) == 0
    dictionaryPath = fullfile(options.ProjectRoot, ...
        "data", "VehicleData.sldd");
end
assert(isfile(dictionaryPath), "FSAE:VehicleParameters:MissingDictionary", ...
    "VehicleData.sldd was not found: %s", dictionaryPath);

receipt = struct( ...
    "ChangedCount", height(changes), ...
    "DictionaryPath", dictionaryPath, ...
    "BackupPath", "");
if isempty(changes)
    return
end

if options.CreateBackup
    backupFolder = options.BackupFolder;
    if strlength(backupFolder) == 0
        timestamp = string(datetime("now", ...
            "Format", "yyyyMMdd_HHmmss_SSS"));
        backupFolder = fullfile(options.ProjectRoot, "results", ...
            "vehicle_parameter_backups", timestamp);
    end
    if ~isfolder(backupFolder)
        mkdir(backupFolder);
    end
    backupPath = fullfile(backupFolder, "VehicleData.sldd");
    [copied, message] = copyfile(dictionaryPath, backupPath);
    assert(copied, "FSAE:VehicleParameters:BackupFailed", ...
        "Could not back up VehicleData.sldd: %s", message);
    receipt.BackupPath = string(backupPath);
end

dictionary = Simulink.data.dictionary.open(dictionaryPath);
cleanup = onCleanup(@() close(dictionary));
designData = getSection(dictionary, "Design Data");

try
    groupNames = extractBefore(paths, ".");
    fieldNames = extractAfter(paths, ".");
    assert(all(strlength(groupNames) > 0 & strlength(fieldNames) > 0 & ...
        ~contains(fieldNames, ".")), ...
        "FSAE:VehicleParameters:InvalidPath", ...
        "Parameter paths must use the Group.Field form.");

    allowedGroups = ["Vehicle", "Tire", "Aero", "Powertrain", ...
        "Battery", "Brake", "Simulation"];
    assert(all(ismember(groupNames, allowedGroups)), ...
        "FSAE:VehicleParameters:InvalidGroup", ...
        "Only physical vehicle and simulation parameter groups may be edited.");

    uniqueGroups = unique(groupNames, "stable");
    for groupIndex = 1:numel(uniqueGroups)
        groupName = uniqueGroups(groupIndex);
        groupEntry = getEntry(designData, char(groupName));
        groupValue = getValue(groupEntry);
        selectedRows = find(groupNames == groupName);
        for selectedIndex = 1:numel(selectedRows)
            row = selectedRows(selectedIndex);
            fieldName = char(fieldNames(row));
            assert(isstruct(groupValue) && isfield(groupValue, fieldName), ...
                "FSAE:VehicleParameters:MissingParameter", ...
                "Parameter does not exist in VehicleData.sldd: %s", paths(row));
            record = groupValue.(fieldName);
            validateRecordAndValue(record, values(row), paths(row));
            record.Value = values(row);
            if isfield(record, "LastUpdated")
                record.LastUpdated = char(datetime("today", ...
                    "Format", "yyyy-MM-dd"));
            end
            groupValue.(fieldName) = record;
        end
        setValue(groupEntry, groupValue);
    end
    saveChanges(dictionary);
catch exception
    discardChanges(dictionary);
    rethrow(exception);
end
end

function validateRecordAndValue(record, value, path)
assert(isstruct(record) && isscalar(record) && isfield(record, "Value") && ...
    isnumeric(record.Value) && isreal(record.Value) && ...
    isscalar(record.Value), "FSAE:VehicleParameters:UnsupportedParameter", ...
    "Parameter is not an editable real numeric scalar: %s", path);
assert(isfinite(value), "FSAE:VehicleParameters:NonFiniteValue", ...
    "Parameter must be a finite scalar: %s", path);

lowerBound = metadataBound(record, "LowerBound");
upperBound = metadataBound(record, "UpperBound");
assert(~isfinite(lowerBound) || value >= lowerBound, ...
    "FSAE:VehicleParameters:BelowLowerBound", ...
    "%s must be greater than or equal to %.6g.", path, lowerBound);
assert(~isfinite(upperBound) || value <= upperBound, ...
    "FSAE:VehicleParameters:AboveUpperBound", ...
    "%s must be less than or equal to %.6g.", path, upperBound);
end

function value = metadataBound(record, fieldName)
value = NaN;
if isfield(record, fieldName)
    candidate = record.(fieldName);
    if isnumeric(candidate) && isreal(candidate) && isscalar(candidate)
        value = double(candidate);
    end
end
end

function root = defaultProjectRoot()
root = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
end
