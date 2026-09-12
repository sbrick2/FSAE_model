function setTireModelRuntimeData(designData, profile)
%SETTireModelRUNTIMEDATA Install numeric parameters used by MATLAB Function blocks.

setEntry(designData, "TireMFParameters", ...
    serializeTireModelMFParameters(profile));
setEntry(designData, "TireMapPoints", profile.Map.Points);
setEntry(designData, "TireMapForces", profile.Map.Forces);
setEntry(designData, "TireMapScale", profile.Map.Scale);
setEntry(designData, "TireMapLowerBound", profile.Map.LowerBound);
setEntry(designData, "TireMapUpperBound", profile.Map.UpperBound);
setEntry(designData, "TireInputPressurePa", profile.InputPressurePa);
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
