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
%[S, Act, Tr, I, AP, L] = getTSvehicle(vehDirs(1));
% finished when only one vehicle
if nrVeh == 1
    return;
end
% Generate Transition Systems for the rest
for i = 1:nrVeh
    % generate TS for vehicle
    [Sv, Actv, Trv, Iv, APv, Lv] = getTSvehicle(vehDirs(i));
    
    % Test: TransitionSystem
    TS(i) = TransitionSystem(Sv, Actv, Trv, Iv, APv, Lv);
    %     TS2 = TransitionSystem(Sv, Actv, Trv, Iv, APv, Lv);
    %     TSp = TransitionSystem.parallelize([TS1 TS2]);
    %
    %     S = TSp.states;
    %     Act = TSp.actions;
    %     Tr = TSp.transitions;
    %     I = TSp.initialStates;
    %     AP = TSp.atomicProps;
    %     L = TSp.labels;
    
end

TSp = TransitionSystem.parallelize(TS);
S = TSp.states;
Act = TSp.actions;
Tr = TSp.transitions;
I = TSp.initialStates;
AP = TSp.atomicProps;
L = TSp.labels;
end

