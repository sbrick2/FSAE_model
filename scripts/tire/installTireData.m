function selection = installTireData(database, dictionaryPath)
%INSTALLTireModelTIREDATA Store fitted TireModel profiles and explicit selection metadata.

arguments
    database (1, 1) struct
    dictionaryPath (1, 1) string
end

assert(isfile(dictionaryPath), "FSAE:TireModel:DictionaryMissing", ...
    "Data dictionary does not exist: %s", dictionaryPath);
assert(~isempty(database.Profiles), "FSAE:TireModel:NoProfiles", ...
    "TireModel database has no fitted profiles.");

selectedIndex = double(database.DefaultProfileIndex);
selectedProfile = database.Profiles(selectedIndex);
selection = struct( ...
    ProfileIndex = uint16(selectedIndex), ...
    ProfileName = selectedProfile.Name, ...
    TireID = selectedProfile.TireID, ...
    ModelMode = uint8(1), ...
    ModelModeMeaning = "0=TireSimple, 1=TireMF62, 2=TireTTCMap", ...
    VehicleSelectionConfirmed = true, ...
    Notes = "User selected Hoosier 43075 and a 7 inch rim; R20 is the Round 9 model basis.");

dictionary = Simulink.data.dictionary.open(dictionaryPath);
cleanup = onCleanup(@() close(dictionary));
designData = getSection(dictionary, "Design Data");
setEntry(designData, "TireDatabase", database);
setEntry(designData, "TireSelectedProfile", selectedProfile);
setEntry(designData, "TireSelection", selection);
setTireModelRuntimeData(designData, selectedProfile);
saveChanges(dictionary);
end

function setEntry(section, name, value)
try
    entry = getEntry(section, name);
    setValue(entry, value);
catch exception
    if contains(exception.identifier, "EntryNotFound") || ...
            contains(exception.message, "does not exist", ...
            IgnoreCase = true)
        addEntry(section, name, value);
    else
        rethrow(exception);
    end
end
end
