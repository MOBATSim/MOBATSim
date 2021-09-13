function [S, Act, Tr, I, AP, L] = TsACheck(S, Act, Tr, I, AP, L, Q, Sig, delta, Q0, F)
%TSASYNTHESIS Check transition system TS with buchi automata BA
%   Detailed explanation goes here

    arguments
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
Sm = S' +" "+ Q;
% rearrange to row vector
Sm = Sm(:)';

% Transitions
Trm = strings(0,0);
% for every transition from TS
for i = 1:size(Tr,1)
    transition = Tr(i,:); 
    labelTarget = L(transition(2)==L(:,1),2); % get the label from the transition target
    labelSource = L(transition(1)==L(:,1),2); % get the label from the transition target
    currentAPs = stringToAPs(labelTarget, labelSource); % get current active atomic propositions from label
    % TODO: check transition between every vehicle combination: 1 vehicle -> 1 mal 
                                                               %2 vehicle -> 1 mal
                                                               %3 vehicle -> 3 mal
                                                               %4 vehicle -> 6 mal, but stop as soon as it didnt work
    partialActs = split(Tr(i,3)); % all partial actions in this action
    if length(partialActs) >= 3
        test = 1;
    end
    % try transitions in buchi with propositions
    for j=1:size(delta,1)
        % test BA transitions with every partial action pairs
        % solution for only one partial action
        if length(partialActs) == 1
            result = true; % there is only one car taht could not collide with an other 
        else
            % solution for more then one partial action
            results = [];
            for k=1:length(partialActs)-1
                for m=k+1:length(partialActs)
                    results(end+1) = delta{j,3}(currentAPs{k}, currentAPs{m}); % check which transition becomes true
                end
            end
%             if length(partialActs) == 2
%                 result = partialResult;
%             elseif length(partialActs) == 3
%                 result = partialResult(1,1)&&partialResult(1,2)&&partialResult(2,1);
%             else
%                 result = false;
%             end
            %result = istril(flip(partialResult)); % TODO: fix this!!!
            result = all(results);
        end        
        if result == true %
            % Build transition in TS x A
            Trm(end+1,:) = [join([Tr(i,1) delta{j,1}]) join([Tr(i,2) delta{j,2}]) Tr(i,3)];
        end
    end
end


% Initial states
Im = strings(0,0);
for i=1:length(I) % check every TS initial state
    labelInitState = L(I(i)==L(:,1),2);
    stringAPsInit = split(labelInitState);
    % get all non init state APs
    stringAPsNotInit = strings(0,0);
    for stringAP = AP
        if stringAP ~= stringAPsInit
            stringAPsNotInit(end+1) = stringAP;
        end
    end
    % get all input 
    [~, activeAPs] = stringToAPs(stringAPsInit,stringAPsNotInit); % get active APs from this init state
    for j=1:size(delta,1)
        fromBASourceState = any(delta{j,1} == Q0); % this BA transition starts at a source state
        % TODO: check this when more safety conditions are used, could
        % maybe not work.
        initStateReachable = delta{j,3}(activeAPs,activeAPs); % BA transition action works with TS initial state label
        if fromBASourceState && initStateReachable
            % this is a valid source state in TS x A
            Im(end+1) = I(i) +" "+delta{j,2}; % the new state is the TS init state + target of delta transition
        end
    end
end
% TODO: fix this
Im = I(1) +" "+ delta{1,2};

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

% find all final states in Sm

FSm = strings(0,0);
% get all states that are also final state of A
for i=1:length(F)
    numFinalStateChars = strlength(F(i)); % get number of characters in final state of A
    posInString = strfind(Sm,F(i),'ForceCellOutput',true); % check position of the final state of A in state string
    % check for every state if it is containing the final state of A at correct
    % position
    for j=1:length(Sm) 
        if any(posInString{j} == (strlength(Sm(j)) - (numFinalStateChars - 1))) % final state of A in correct position
            FSm(end+1) = Sm(j);% add to list of final states
        end
    end
end
% Digraph
% table with all states
NodeTable = table(Sm', 'VariableNames',{'Name'});
% table with all edges (transitions) and labled with actions
EdgeTable = table([Trm(:,1) Trm(:,2)], Trm(:,3), 'VariableNames',{'EndNodes' 'Code'});
% build digraph with it
graph = digraph(EdgeTable,NodeTable);
% graph with connections to every reachable state from graph
graphReachableStates = transclosure(graph);
% adjacency matrix of graph reachable states
adjMatrixReachable = full(adjacency(graphReachableStates));

% remove all states not reachable from init state
for i=length(FSm):-1:1
    % check if state can be reached by any init state
    allPreStates = predecessors(graphReachableStates,FSm(i));
    if ~any(Im == allPreStates)
        FSm(i) = []; % remove because could not be reached by any init state
    end
end
% remove all not stable states (not final infinetly often)
for i=length(FSm):-1:1
    allPreStates = predecessors(graphReachableStates,FSm(i)); % get all states leading to this one
    postStates = successors(graph,FSm(i));
    % check if there are post states and if the post states could not reach this final state   
    if ~isempty(postStates)
        % check if the post states are also prepre...States from our state, when not delete them
        validFinal = false;
        for postState = postStates'
            if any(postState == allPreStates)
                % post state is also pre state, all fine
                validFinal = true;
            end            
        end
        % delete from final list, because there is not one post state that
        % is also prepre... state from this state 
        if  ~validFinal
            FSm(i) = [];
        end
    end
end

% delete dead targets and dead loops
% find all states that is no transition leaving and they are not a final
% state and also unconnected loops
for state = Sm
    % must be init state or final state or could be reached from a init
    % state and lead to final state
    if all(state ~= Im) && all(state ~= FSm)
        % not from init reachable or not reaches a final state
        if ~any(Im == Sm(logical(adjMatrixReachable(:,Sm == state)))) ... % from a init state not reachable
            || ~any(FSm == Sm(logical(adjMatrixReachable(Sm == state,:)))) % not leading to a final state
            Sm(state == Sm) = []; % remove state
            Trm(any(state == Trm(:,[1 2]),2),:) = []; % remove all transitions to and from this state            
        end
    end
end

% delete all states, transitions, etc, from TS when they are not in TSm
% States
for state = S
     if isempty(strfind(Sm,state,'ForceCellOutput',true)) % search if state is in Sm
         % if the state is not in Sm, delete it
         S(state == S) = [];
     end
end

% Actions
for action = Act
    if ~any(action == Trm(:,3))
        % delete all actions that are not used in TSm
        Act(action == Act) = [];
    end
end

% Transitions
% delete atomic proposition of A from merged transition
TrmClean = split(Trm(:,1:2));
TrmClean = [TrmClean(:,:,1) Trm(:,3)];
for i=size(Tr,1):-1:1
    % check if there is an action in Trm that has the same source, target
    % and action
    if ~any(all(Tr(i,:) == TrmClean,2))
        Tr(i,:) = []; % delete transition when not in merged
    end
end

% Initial states
% get TS inital states out of Im
ImClean = split(Im," ",3);
ImClean = ImClean(:,:,1);
for initState = I
    if ~any(initState == ImClean)
        I(initState == I) = []; % delete init state when not in merged
    end
end

% Labels
% check if state still exists
L(all(L(:,1) ~= S,2),:) = [];


