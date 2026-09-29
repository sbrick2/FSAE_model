classdef RaceReplayVideoResources < handle
    %RACEREPLAYVIDEORESOURCES Keep cleanup state alive during exception unwinding.
    properties
        Figure = []
        Writer = []
        WriterOpen (1, 1) logical = false
        TemporaryFile (1, 1) string = ""
    end
    methods
        function release(resources)
            if resources.WriterOpen
                try
                    close(resources.Writer);
                catch
                    % Continue cleanup without replacing the original failure.
                end
                resources.WriterOpen = false;
            end
            if ~isempty(resources.Figure) && isgraphics(resources.Figure)
                delete(resources.Figure);
            end
            if isfile(resources.TemporaryFile)
                delete(resources.TemporaryFile);
            end
        end
    end
end
