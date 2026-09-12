function profile = getSelectedTireProfile()
%GETSELECTEDTireModelTIREPROFILE Read the selected profile from VehicleData.sldd.
%   A persistent cache avoids opening the dictionary at every model step.
%   Run CLEAR GETSELECTEDTireModelTIREPROFILE after changing profile selection.

persistent cachedProfile
if isempty(cachedProfile)
    projectRoot = string(fileparts(fileparts(fileparts( ...
        mfilename("fullpath")))));
    dictionaryPath = fullfile(projectRoot, "data", "VehicleData.sldd");
    dictionary = Simulink.data.dictionary.open(dictionaryPath);
    cleanup = onCleanup(@() close(dictionary));
    designData = getSection(dictionary, "Design Data");
    cachedProfile = getValue(getEntry( ...
        designData, "TireSelectedProfile"));
end
profile = cachedProfile;
end
