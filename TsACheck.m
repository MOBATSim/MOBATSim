function [S, Act, Tr, I, AP, L] = TsACheck(allAPs, S, Act, Tr, I, AP, L, Q, Sig, delta, Q0, F)
%TSASYNTHESIS Check transition system TS with buchi automata BA
%   Detailed explanation goes here

    arguments
        allAPs  (1,1) struct    % structure containing all possible atomic propositions
        S       (1,:) string    % states for TS
        Act     (1,:) string    % actions for TS
        Tr      (:,3) string    % transitions for TS
        I       (1,:) string    % initial states for TS
        AP      (1,:) string    % atomic propositions for TS
        L       (:,2) string    % labels for TS ( state | label )
        Q       (1,:) string    % states for BA
        Sig     (1,:) cell      % actions for BA
        delta   (:,3) cell      % transitions for BA
        Q0      (1,:) string    % initial states for BA
        F       (1,:) string    % final states for BA
    end
%% merge TS and BA
% States
Sm = S' + Q;
% rearrange to row vector
Sm = Sm(:)';

% Transitions
Trm = strings(0,0);
% for every transition from TS
for i = 1:size(Tr,1)
    transition = Tr(i,:); 
    labelTarget = L(transition(2)==L(:,1),2); % get the label from the transition target
    currentAPs = setAPs(allAPs, labelTarget); % get current active atomic propositions from label
    % try transitions in buchi with propositions
    for j= 1:size(delta,1)
        result = delta{j,3}(currentAPs); % check which transition becomes true
        if result == true %
            % Build transition in TS x A
            Trm(end+1,:) = [Tr(i,1)+delta{j,1} Tr(i,2)+delta{j,2} Tr(i,3)];
        end
    end
end

% Initial states
Im = strings(0,0);
for i=1:length(I) % check every TS initial state
    labelInitState = L(I(i)==L(:,1),2);
    activeAPs = setAPs(allAPs,labelInitState); % get active APs from this init state
    for j=1:size(delta,1)
        fromBASourceState = any(delta{j,1} == Q0); % this BA transition starts at a source state
        withTSInitLabel = delta{j,3}(activeAPs); % BA transition action works with TS initial state label
        if fromBASourceState && withTSInitLabel
            % this is a valid source state in TS x A
            Im(end+1) = I(i)+delta{j,2}; % the new state is the TS init state + target of delta transition
        end
    end
end

% Atomic propositions
APm = Q;

% Labels
% Combine to new state-label combination
Lm(1:length(Sm),1) = Sm'; % label states
Lm(:,2) = repelem(Q,length(S))'; % matching labels

%% delete states and transitions from TS with merged TSm

% delete dead sources
% remove states that are not target of a transition and not initial states
% from TSm
for state = Sm
    if ~any(state == Im) && ~any(state == Trm(:,2))
        Trm(state == Trm(:,1),:) = []; % remove all transitions from this state
        Sm(state == Sm) = []; % remove state
    end
end

% delete dead targets
% find all states that is no transition leaving and they are not a final
% state
for state = Sm
    if ~any(state == Trm(:,1))
        
        %TODO: resume from here and use transclosure(graph)
        Trm(state == Trm(:,1),:) = []; % remove all transitions from this state
        Sm(state == Sm) = []; % remove state
    end
end

% delete deat loops
% TODO: this is a difficult task without
% "allpaths(graph,startNode,endNode)" (only in 2021a) :/


% Digraph
% table with all states
NodeTable = table(Sm', 'VariableNames',{'Name'});
% table with all edges (transitions) and labled with actions
EdgeTable = table([Trm(:,1) Trm(:,2)], Trm(:,3), 'VariableNames',{'EndNodes' 'Code'});
% build digraph with it
graph = digraph(EdgeTable,NodeTable);

%p = plot(graph,'EdgeLabel', graph.Edges.Code);

% [paths,edgepaths] = allpaths(graph,"iiq0","ooq1"); % TODO: find initial node and end node
% % Highlight all valid paths
% for i=1:length(edgepaths)
%     highlight(p,'Edges',edgepaths{i},'EdgeColor','r','LineWidth',1.5,'NodeColor','r','MarkerSize',6)
% end

