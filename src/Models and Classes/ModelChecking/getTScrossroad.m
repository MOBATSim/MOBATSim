function [S, Act, Tr, I, AP, L] = getTScrossroad(vehDirs)
%GETCROSSROADTS Generate a transition model of a crossroad with the
%directions of the different vehicles.
%   Detailed explanation goes here

arguments
    vehDirs                 (1,:) string    % directions of different vehicles on crossroad
end

% Get nr of vehicles
nrVeh = length(vehDirs);

% Generate Transition Systems for first vehicle
[S, Act, Tr, I, AP, L] = getTSvehicle(vehDirs(1));
% finished when only one vehicle
if nrVeh == 1
    return;
end
% Generate Transition Systems for the rest
for i = 2:nrVeh
    % generate TS for vehicle
    [Sv, Actv, Trv, Iv, APv, Lv] = getTSvehicle(vehDirs(i));
    
    [S, Act, Tr, I, AP, L] = TSSynthesis(S, Act, Tr, I, AP, L, Sv, Actv, Trv, Iv, APv, Lv);
end

end

