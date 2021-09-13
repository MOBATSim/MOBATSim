% all states must have the same numbers of string in their set

% States
S1 = ["i" "o"];
S2 = ["i" "o"];
S3 = ["i" "o"];
S4 = ["i" "o"];

% Actions
Act1 = ["g1"];
Act2 = ["g2"];
Act3 = ["g3"];
Act4 = ["g4"];

% Transitions
% source | target | action
Tr1 = ["i" "o" "g1"];
Tr2 = ["i" "o" "g2"];
Tr3 = ["i" "o" "g3"];
Tr4 = ["i" "o" "g4"];

% Initial states
I1 = ["i"];
I2 = ["i"];
I3 = ["i"];
I4 = ["i"];

% Atomic propositions
AP1 = ["ie" "ow"];
AP2 = ["iw" "on"]; 
AP3 = ["in" "ow"];
AP4 = ["is" "oe"];

% Labels
L1 = ["i" "ie";
      "o" "ow"];
L2 = ["i" "iw";
      "o" "on"];
L3 = ["i" "in";
      "o" "ow"];
L4 = ["i" "is";
      "o" "oe"];

% 2 vehicles
[S, Act, Tr, I, AP, L] = TSSynthesis(S1, Act1, Tr1, I1, AP1, L1, S2, Act2, Tr2, I2, AP2, L2);
% 3 vehicles
[S, Act, Tr, I, AP, L] = TSSynthesis(S, Act, Tr, I, AP, L, S3, Act3, Tr3, I3, AP3, L3);
% 4 vehicles
[S, Act, Tr, I, AP, L] = TSSynthesis(S, Act, Tr, I, AP, L, S4, Act4, Tr4, I4, AP4, L4);



% Include safety properties
 

% function handel for all safety conditions containing equations for every transition condition
% all safety conditions
forbiddenPaths = {{'in','oe'}, {'ie','ow'};
                 {'in','oe'}, {'iw','on'};
                 {'in','oe'}, {'ie','os'};
                 {'in','oe'}, {'is','on'};
                 {'in','oe'}, {'iw','oe'};
                 {'in','oe'}, {'is','oe'};
                 {'in','os'}, {'ie','ow'};
                 {'in','os'}, {'is','ow'};
                 {'in','os'}, {'iw','on'};
                 {'in','os'}, {'iw','oe'};
                 {'in','os'}, {'iw','os'};
                 {'in','os'}, {'ie','os'};
                 {'in','ow'}, {'ie','ow'};
                 {'in','ow'}, {'is','ow'};
                 {'ie','os'}, {'is','on'};
                 {'ie','os'}, {'is','ow'};
                 {'ie','os'}, {'iw','oe'};
                 {'ie','os'}, {'iw','os'};
                 {'ie','ow'}, {'is','on'};
                 {'ie','ow'}, {'iw','on'};
                 {'ie','ow'}, {'is','ow'};
                 {'ie','on'}, {'is','on'};
                 {'ie','on'}, {'iw','on'};
                 {'is','ow'}, {'iw','oe'};
                 {'is','ow'}, {'iw','on'};
                 {'is','on'}, {'iw','oe'};
                 {'is','on'}, {'iw','on'};
                 {'is','oe'}, {'iw','oe'}};
             
% Preallocate function handle array
  SFs = {@(APs1, APs2, eqNr) SafetyCondition(forbiddenPaths{1,1},forbiddenPaths{1,2}, APs1, APs2, eqNr)};
for i = 1:size(forbiddenPaths,1)
    SFs(i) = {@(APs1, APs2, eqNr) SafetyCondition(forbiddenPaths{i,1},forbiddenPaths{i,2}, APs1, APs2, eqNr)};
end

%SF1 = @(APs1, APs2, eqNr) SafetyCondition({'ie','ow'}, {'iw','on'}, APs1, APs2, eqNr);

% Buchi Automat for safety property

% generate all safety properties
for i = 1:size(forbiddenPaths,1)
    Q(i,:) = ["q0" "q1"];
    Sig(i,:) = {@(APs1, APs2) (SFs{i}(APs1, APs2, 1)) ...
              @(APs1, APs2) (SFs{i}(APs1, APs2, 2)) ...
              @(APs1, APs2) (SFs{i}(APs1, APs2, 3))}; % the eqations are stored in  function safety condition
    delta(3*i:3*i+2,:) = {"q0" "q0" @(APs1, APs2) (SFs{i}(APs1, APs2, 1)); ... % TODO: fix this kind of saving
                "q0" "q1" @(APs1, APs2) (SFs{i}(APs1, APs2, 2)); ... 
                "q1" "q1" @(APs1, APs2) (SFs{i}(APs1, APs2, 3))};
    Q0(i) = ["q0"];
    F(i) = ["q1"];
end
tic
% Check which parts of TS are still valid with A
for i = 1:size(forbiddenPaths,1)
    [S, Act, Tr, I, AP, L] = TsACheck(S, Act, Tr, I, AP, L, Q(i,:), Sig(i,:), delta(3*i:3*i+2,:), Q0(i), F(i));
end
toc
% Digraph
% table with all states
NodeTable = table(S', 'VariableNames',{'Name'});
% table with all edges (transitions) and labled with actions
EdgeTable = table([Tr(:,1) Tr(:,2)], Tr(:,3), 'VariableNames',{'EndNodes' 'Code'});
% build digraph with it
graph = digraph(EdgeTable,NodeTable);

p = plot(graph,'EdgeLabel', graph.Edges.Code);
