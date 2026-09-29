function [normalLoad, frontLoad, rearLoad] = distributeWheelNormalLoads( ...
    frontAxleLoad, rearAxleLoad, frontTransfer, rearTransfer, ...
    trackFront, trackRear)
%DISTRIBUTEWHEELNORMALLOADS Enforce unilateral wheel contact in static loads.
%   Transfers are the loads moved from left to right, in N. When a wheel
%   lifts, redistribute roll support to the other axle, retaining total
%   force and pitch/roll moments whenever a static contact solution exists.
%   Beyond the static support limit, both inside wheels are unloaded. A
%   planar model cannot represent the subsequent rollover or pitch motion.
%   Inputs must be finite, with positive track widths. Order: [FL; FR; RL; RR].

totalLoad = max(0.0, frontAxleLoad + rearAxleLoad);
frontLoad = min(totalLoad, max(0.0, frontAxleLoad));
rearLoad = totalLoad - frontLoad;
frontMomentLimit = 0.5 * frontLoad * trackFront;
rearMomentLimit = 0.5 * rearLoad * trackRear;
requestedRollMoment = frontTransfer * trackFront + ...
    rearTransfer * trackRear;
supportMomentLimit = frontMomentLimit + rearMomentLimit;
rollMoment = min(supportMomentLimit, ...
    max(-supportMomentLimit, requestedRollMoment));

% The contact constraint replaces the elastic roll-share assumption once
% it would require a tire to pull downward on the road.
frontMomentLower = max(-frontMomentLimit, rollMoment - rearMomentLimit);
frontMomentUpper = min(frontMomentLimit, rollMoment + rearMomentLimit);
frontMoment = min(frontMomentUpper, ...
    max(frontMomentLower, frontTransfer * trackFront));
rearMoment = rollMoment - frontMoment;
frontTransfer = min(0.5 * frontLoad, ...
    max(-0.5 * frontLoad, frontMoment / trackFront));
rearTransfer = min(0.5 * rearLoad, ...
    max(-0.5 * rearLoad, rearMoment / trackRear));

normalLoad = [0.5 * frontLoad - frontTransfer; ...
    0.5 * frontLoad + frontTransfer; ...
    0.5 * rearLoad - rearTransfer; ...
    0.5 * rearLoad + rearTransfer];
end
