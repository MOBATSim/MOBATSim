% Pairs of directions that are crossing at the crossroad ("ne" means
% Nort-East)
crossingPaths = ["ne", "ew";
                 "ne", "wn";
                 "ne", "es";
                 "ne", "sn";
                 "ne", "we";
                 "ne", "se";
                 "ns", "ew";
                 "ns", "sw";
                 "ns", "wn";
                 "ns", "we";
                 "ns", "ws";
                 "ns", "es";
                 "nw", "ew";
                 "nw", "sw";
                 "es", "sn";
                 "es", "sw";
                 "es", "we";
                 "es", "ws";
                 "ew", "sn";
                 "ew", "wn";
                 "ew", "sw";
                 "en", "sn";
                 "en", "wn";
                 "sw", "we";
                 "sw", "wn";
                 "sn", "we";
                 "sn", "wn";
                 "se", "we"];

% Directions of vehicles on crossroad
% 1 vehicle
%dirs = ["ew"];
% 2 vehicle
dirs = ["ew","wn"];
% 3 vehicle
%dirs = ["ew","wn","nw"];
% 4 vehicle
%dirs = ["ew","wn","nw","se"];

% Generate Transition System for the crossroad
TS = TScrossroad(dirs); 

% Synthesize a model that does not contain any states with vehicles
% colliding on crossing paths
TS.synthesizeWithSPs(crossingPaths);
% Plot the synthesized TS
TS.plotTS();
