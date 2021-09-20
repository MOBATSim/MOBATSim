% all states must have the same numbers of string in their set
  
%   % States
S1 = ["i" "p" "o"];
S2 = ["i" "p" "o"];
S3 = ["i" "p" "o"];
S4 = ["i" "p" "o"];

% Actions
Act1 = ["g1"];
Act2 = ["g2"];
Act3 = ["g3"];
Act4 = ["g4"];

% Transitions
% source | target | action
Tr1 = ["i" "p" "g1";
       "p" "o" "g1";
       "o" "o" "g1"];
Tr2 = ["i" "p" "g2";
       "p" "o" "g2";
       "o" "o" "g2"];
Tr3 = ["i" "p" "g3";
       "p" "o" "g3";
       "o" "o" "g3"];
Tr4 = ["i" "p" "g4";
       "p" "o" "g4";
       "o" "o" "g4"];

% Initial states
I1 = ["i"];
I2 = ["i"];
I3 = ["i"];
I4 = ["i"];

% Atomic propositions
AP1 = ["iew" "pew" "oew"];
AP2 = ["iwn" "pwn" "own"]; 
AP3 = ["inw" "pnw" "onw"];
AP4 = ["ise" "pse" "ose"];

% Labels
L1 = [S1' AP1'];
L2 = [S2' AP2'];
L3 = [S3' AP3'];
L4 = [S4' AP4'];

% 2 vehicles
[S, Act, Tr, I, AP, L] = TSSynthesis(S1, Act1, Tr1, I1, AP1, L1, S2, Act2, Tr2, I2, AP2, L2);
% 3 vehicles
[S, Act, Tr, I, AP, L] = TSSynthesis(S, Act, Tr, I, AP, L, S3, Act3, Tr3, I3, AP3, L3);
% 4 vehicles
[S, Act, Tr, I, AP, L] = TSSynthesis(S, Act, Tr, I, AP, L, S4, Act4, Tr4, I4, AP4, L4);

% Digraph
% table with all states
NodeTable = table(S', L(:,2),'VariableNames',{'Name' 'Label'});
% table with all edges (transitions) and labled with actions
EdgeTable = table([Tr(:,1) Tr(:,2)], Tr(:,3), 'VariableNames',{'EndNodes' 'Code'});
% build digraph with it
graph = digraph(EdgeTable,NodeTable);
figure(2)
p = plot(graph,'EdgeLabel', graph.Edges.Code, 'NodeLabel', graph.Nodes.Label);

% Include safety properties
 

% function handel for all safety conditions containing equations for every transition condition
% all safety conditions          
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
             
 %crossingPaths = {"pew", "pwn"};
             
% Preallocate function handle array
  SFs = {@(allAPs, eqNr) checkSafetyCondition(crossingPaths(1,1),crossingPaths(1,2), allAPs, eqNr)};
for i = 1:size(crossingPaths,1)
    SFs(i) = {@(allAPs, eqNr) checkSafetyCondition(crossingPaths(i,1),crossingPaths(i,2), allAPs, eqNr)};
end

% Buchi Automat for safety property

% generate all safety properties
for i = 1:size(crossingPaths,1)
    Q(i,:) = ["q0" "q1"];
    Sig(i,:) = {@(allAPs) (SFs{i}(allAPs, 1)) ...
                @(allAPs) (SFs{i}(allAPs, 2)) ...
                @(allAPs) (SFs{i}(allAPs, 3))}; % the eqations are stored in  function safety condition
    delta(3*i:3*i+2,:) = {"q0" "q0" Sig{i,1}; ...
                          "q0" "q1" Sig{i,2}; ... 
                          "q1" "q1" Sig{i,3}};
    Q0(i) = ["q0"];
    F(i) = ["q1"];
end
tic
% Check which parts of TS are still valid with A
for i = 1:size(crossingPaths,1)
    % Check if this safety condition is relevant for this model
    if all(contains(crossingPaths(i,:), AP))
        [S, Act, Tr, I, AP, L] = TsACheck(S, Act, Tr, I, AP, L, Q(i,:), Sig(i,:), delta(3*i:3*i+2,:), Q0(i), F(i));
    end
end
toc
% Digraph
% table with all states
NodeTable = table(S', L(:,2),'VariableNames',{'Name' 'Label'});
% table with all edges (transitions) and labled with actions
EdgeTable = table([Tr(:,1) Tr(:,2)], Tr(:,3), 'VariableNames',{'EndNodes' 'Code'});
% build digraph with it
graph = digraph(EdgeTable,NodeTable);

p = plot(graph,'EdgeLabel', graph.Edges.Code, 'NodeLabel', graph.Nodes.Label);
