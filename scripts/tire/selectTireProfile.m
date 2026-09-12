function selection = selectTireProfile(profileName, modelMode, options)
%SELECTTireModelTIREPROFILE Select an installed 43075 profile and tire model mode.

arguments
    profileName (1, 1) string
    modelMode (1, 1) double {mustBeInteger, mustBeBetween(modelMode, 0, 2)}
    options.ConfirmVehicleSelection (1, 1) logical = false
    options.DictionaryPath (1, 1) string = defaultDictionaryPath()
end

dictionary = Simulink.data.dictionary.open(options.DictionaryPath);
cleanup = onCleanup(@() close(dictionary));
designData = getSection(dictionary, "Design Data");
database = getValue(getEntry(designData, "TireDatabase"));
names = string({database.Profiles.Name});
index = find(names == profileName, 1);
assert(~isempty(index), "FSAE:TireModel:UnknownProfile", ...
    "Unknown TireModel tire profile '%s'. Available profiles: %s", ...
    profileName, strjoin(names, ", "));
profile = database.Profiles(index);
selection = struct( ...
    ProfileIndex = uint16(index), ...
    ProfileName = profile.Name, ...
    TireID = profile.TireID, ...
    ModelMode = uint8(modelMode), ...
    ModelModeMeaning = "0=TireSimple, 1=TireMF62, 2=TireTTCMap", ...
    VehicleSelectionConfirmed = options.ConfirmVehicleSelection, ...
    Notes = selectionNotes(options.ConfirmVehicleSelection));
setValue(getEntry(designData, "TireSelectedProfile"), profile);
setValue(getEntry(designData, "TireSelection"), selection);
setTireModelRuntimeData(designData, profile);
saveChanges(dictionary);
clear getSelectedTireProfile evaluateTireMF62Vector evaluateTTCMapVector
end

function notes = selectionNotes(confirmed)
if confirmed
    notes = "Compound and rim selection explicitly confirmed by user.";
else
    notes = "43075 size selected; compound and rim are not confirmed.";
end
end

function path = defaultDictionaryPath()
root = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
path = fullfile(root, "data", "VehicleData.sldd");
end
