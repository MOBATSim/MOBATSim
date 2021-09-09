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
allAPs = struct("in", 0, "ie", 0, "is", 0, "iw", 0, "on", 0, "oe", 0, "os", 0, "ow", 0);
%symAllAP = str2sym(allAP);
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
%[S, Act, Tr, I, AP, L] = TSSynthesis(S, Act, Tr, I, AP, L, S3, Act3, Tr3, I3, AP3, L3);
% 4 vehicles
%[S, Act, Tr, I, AP, L] = TSSynthesis(S, Act, Tr, I, AP, L, S4, Act4, Tr4, I4, AP4, L4);

% % Digraph
% % table with all edges (transitions) and labled with actions
% EdgeTable = table([Tr(:,1) Tr(:,2)], Tr(:,3), 'VariableNames',{'EndNodes' 'Code'});
% % build digraph with it
% graph = digraph(EdgeTable);
% 
% plot(graph,'EdgeLabel', graph.Edges.Code)



% Include safety properties
 
% Buchi Automat for safety property
Q = ["q0" "q1"];
% function handels for every transition condition
a = @(APs) (APs.ie & APs.iw & ~APs.on)|(APs.ie & APs.iw & ~APs.ow);
b = @(APs) (~APs.ie&~APs.on)|(~APs.ie&~APs.ow)|(~APs.iw&~APs.on)|(~APs.iw&~APs.ow);
c = @(APs) 1;

Sig = {a b c}; % Sig stands for sigma
delta = {"q0" "q0" a; ...
         "q0" "q1" b; ... 
         "q1" "q1" c};
Q0 = ["q0"];
F = ["q1"];

[S, Act, Tr, I, AP, L] = TsACheck(allAPs, S, Act, Tr, I, AP, L, Q, Sig, delta, Q0, F);

% % merge TS and BA
% % States
% Sm = S' + Q;
% % rearrange to row vector
% Sm = Sm(:)';
% 
% % Transitions
% Trm = strings(0,0);
% % for every transition from TS
% for i = 1:size(Tr,1)
%     transition = Tr(i,:); 
%     labelTarget = L(transition(2)==L(:,1),2); % get the label from the transition target
%     currentAPs = setAPs(allAPs, labelTarget); % get current active atomic propositions from label
%     % try transitions in buchi with propositions
%     for j= 1:size(delta,1)
%         result = delta{j,3}(currentAPs); % check which transition becomes true
%         if result == true %
%             % Build transition in TS x A
%             Trm(end+1,:) = [Tr(i,1)+delta{j,1} Tr(i,2)+delta{j,2} Tr(i,3)];
%         end
%     end
% end
% 
% % Initial states
% Im = strings(0,0);
% for i=1:length(I) % check every TS initial state
%     labelInitState = L(I(i)==L(:,1),2);
%     activeAPs = setAPs(allAPs,labelInitState); % get active APs from this init state
%     for j=1:size(delta,1)
%         fromBASourceState = any(delta{j,1} == Q0); % this BA transition starts at a source state
%         withTSInitLabel = delta{j,3}(activeAPs); % BA transition action works with TS initial state label
%         if fromBASourceState && withTSInitLabel
%             % this is a valid source state in TS x A
%             Im(end+1) = I(i)+delta{j,2}; % the new state is the TS init state + target of delta transition
%         end
%     end
% end
% 
% % Atomic propositions
% APm = Q;
% 
% % Labels
% % Combine to new state-label combination
% Lm(1:length(Sm),1) = Sm'; % label states
% Lm(:,2) = repelem(Q,length(S))'; % matching labels
% 
% % Digraph
% % table with all states
% NodeTable = table(Sm', 'VariableNames',{'Name'});
% % table with all edges (transitions) and labled with actions
% EdgeTable = table([Trm(:,1) Trm(:,2)], Trm(:,3), 'VariableNames',{'EndNodes' 'Code'});
% % build digraph with it
% graph = digraph(EdgeTable,NodeTable);
% 
% p = plot(graph,'EdgeLabel', graph.Edges.Code);
% 
% [paths,edgepaths] = allpaths(graph,"iiq0","ooq1"); % TODO: find initial node and end node
% % Highlight all valid paths
% for i=1:length(edgepaths)
%     highlight(p,'Edges',edgepaths{i},'EdgeColor','r','LineWidth',1.5,'NodeColor','r','MarkerSize',6)
% end