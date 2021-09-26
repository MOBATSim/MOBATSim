
% Directions of vehicles
% 1 vehicle
%dirs = ["ew"];
% 2 vehicle
%dirs = ["ew","wn"];
% 3 vehicle
dirs = ["ew","wn","nw"];
% 4 vehicle
%dirs = ["ew","wn","nw","se"];

% Generate Transition System for the crossroad
[S, Act, Tr, I, AP, L] = getTScrossroad(dirs);

% Plot the crossroad TS
plotTS(S, Act, Tr, I, AP, L);

% array with paths that are crossing in each line        
crossingPaths = ["pne", "pew";
                 "pne", "pwn";
                 "pne", "pes";
                 "pne", "psn";
                 "pne", "pwe";
                 "pne", "pse";
                 "pns", "pew";
                 "pns", "psw";
                 "pns", "pwn";
                 "pns", "pwe";
                 "pns", "pws";
                 "pns", "pes";
                 "pnw", "pew";
                 "pnw", "psw";
                 "pes", "psn";
                 "pes", "psw";
                 "pes", "pwe";
                 "pes", "pws";
                 "pew", "psn";
                 "pew", "pwn";
                 "pew", "psw";
                 "pen", "psn";
                 "pen", "pwn";
                 "psw", "pwe";
                 "psw", "pwn";
                 "psn", "pwe";
                 "psn", "pwn";
                 "pse", "pwe"];          

% Generate a verified Transition System with Büchi Automata
[S, Act, Tr, I, AP, L] = getVerifiedTS(S, Act, Tr, I, AP, L, crossingPaths);

% Plot the verified TS
plotTS(S, Act, Tr, I, AP, L);
