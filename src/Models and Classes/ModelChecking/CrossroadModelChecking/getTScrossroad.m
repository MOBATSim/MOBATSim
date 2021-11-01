function TSp = getTScrossroad(vehDirs)
%GETCROSSROADTS Generate a transition model of a crossroad with the
%directions of the different vehicles.
%   Detailed explanation goes here

arguments
    vehDirs                 (1,:) string    % directions of different vehicles on crossroad
end

% Generate Transition Systems for every vehicle
for i = 1:length(vehDirs)
    % generate states and so on for the current vehicle
    TS(i) = getTSvehicle(vehDirs(i));    
end

TSp = TransitionSystem.parallelize(TS);
end

