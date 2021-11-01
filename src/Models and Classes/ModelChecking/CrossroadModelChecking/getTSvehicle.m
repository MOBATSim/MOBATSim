function TS = getTSvehicle(dir)
%GETVEHICLEMODEL Generate the transition system of a vehicle on a crossroad
%driving in direction dir.
%   Detailed explanation goes here
% States
S = ["i" "p" "o"];

% Actions
Act = ["g"+dir];

% Transitions
% source | target | action
Tr = [S(1) S(2) Act(1);
      S(2) S(3) Act(1);
      S(3) S(3) Act(1)];

% Initial states
I = [S(1)];

% Atomic propositions
AP = [S(1)+dir S(2)+dir S(3)+dir];

% Labels
L = [S' AP'];

% generate transition system object
TS = TransitionSystem(S, Act, Tr, I, AP, L);
end

