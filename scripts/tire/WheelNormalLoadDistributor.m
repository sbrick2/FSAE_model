classdef WheelNormalLoadDistributor < matlab.System
    %WHEELNORMALLOADDISTRIBUTOR Simulink adapter for static wheel contact loads.

    methods (Access = protected)
        function [normalLoad, frontLoad, rearLoad] = stepImpl(~, ...
                frontAxleLoad, rearAxleLoad, frontTransfer, rearTransfer, ...
                trackFront, trackRear)
            [normalLoad, frontLoad, rearLoad] = distributeWheelNormalLoads( ...
                frontAxleLoad, rearAxleLoad, frontTransfer, rearTransfer, ...
                trackFront, trackRear);
        end

        function [loadSize, frontSize, rearSize] = getOutputSizeImpl(~)
            loadSize = [4, 1];
            frontSize = [1, 1];
            rearSize = [1, 1];
        end

        function [loadType, frontType, rearType] = getOutputDataTypeImpl(~)
            loadType = "double";
            frontType = "double";
            rearType = "double";
        end

        function [loadComplex, frontComplex, rearComplex] = isOutputComplexImpl(~)
            loadComplex = false;
            frontComplex = false;
            rearComplex = false;
        end

        function [loadFixed, frontFixed, rearFixed] = isOutputFixedSizeImpl(~)
            loadFixed = true;
            frontFixed = true;
            rearFixed = true;
        end
    end
end
