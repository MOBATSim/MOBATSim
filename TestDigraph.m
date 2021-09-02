% all states must have the same numbers of string in their set

% States
S1 = ["Ie" "Ow"];
S2 = ["Iw" "On"];
S3 = ["In" "Ow"];
S4 = ["Is" "Oe"];

% Actions
Act1 = ["G1"];
Act2 = ["G2"];
Act3 = ["G3"];
Act4 = ["G4"];

% Transitions
% source | target | action
Tr1 = ["Ie" "Ow" "G1"];
Tr2 = ["Iw" "On" "G2"];
Tr3 = ["In" "Ow" "G3"];
Tr4 = ["Is" "Oe" "G4"];

% Initial states
I1 = ["Ie"];
I2 = ["Iw"];
I3 = ["In"];
I4 = ["Is"];

% Atomic propositions
AP1 = ["P1"];
AP2 = ["P2"];
AP3 = ["P3"];
AP4 = ["P4"];

% Labels
L1 = ["" "P1"];
L2 = ["" "P2"];
L3 = ["" "P3"];
L4 = ["" "P4"];

% 2 vehicles
[S, Act, Tr, I, AP, L] = TSSynthesis(S1, Act1, Tr1, I1, AP1, L1, S2, Act2, Tr2, I2, AP2, L2);
% 3 vehicles
[S, Act, Tr, I, AP, L] = TSSynthesis(S, Act, Tr, I, AP, L, S3, Act3, Tr3, I3, AP3, L3);
% 4 vehicles
[S, Act, Tr, I, AP, L] = TSSynthesis(S, Act, Tr, I, AP, L, S4, Act4, Tr4, I4, AP4, L4);

% Digraph
% table with all edges (transitions) and labled with actions
EdgeTable = table([Tr(:,1) Tr(:,2)], Tr(:,3), 'VariableNames',{'EndNodes' 'Code'});
% build digraph with it
graph = digraph(EdgeTable);

plot(graph,'EdgeLabel', graph.Edges.Code)
