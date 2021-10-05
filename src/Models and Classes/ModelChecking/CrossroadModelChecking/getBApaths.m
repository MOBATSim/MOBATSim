function [Q, Sig, delta, Q0, F] = getBApaths(crossingPath)
%GETBAPATHS Return a Büchi automata with the crossing paths condition
%   Detailed explanation goes here

arguments
    crossingPath          (1,2) string    % two paths that are crossing each other
end

% States
Q = ["q0" "q1"];

    
% Sigma (transition conditions)
% make a safety function that could be accessed to check a transition
% condition
SF = @(allAPs, eqNr) checkSafetyCondition(crossingPath(1),crossingPath(2), allAPs, eqNr);

Sig = {@(allAPs) (SF(allAPs, 1)) ...
       @(allAPs) (SF(allAPs, 2)) ...
       @(allAPs) (SF(allAPs, 3))}; % the eqations are stored in  function safety condition

% Transitions
delta = {"q0" "q0" Sig{1}; ...
         "q0" "q1" Sig{2}; ... 
         "q1" "q1" Sig{3}};
    
% Initial states
Q0 = ["q0"];

% Final states
F = ["q1"];
end

