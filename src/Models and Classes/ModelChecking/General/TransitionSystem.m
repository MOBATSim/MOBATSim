classdef TransitionSystem < handle
    %TRANSITIONSYSTEM State system with states, actions, transitions,
    %inital states, atomic propositions and labels.
    %   Detailed explanation goes here
    
    % Contributors: Johannes Pintscher
    
    properties
        states                  (1,:) string    % states S
        actions                 (1,:) string    % actions Act
        transitions             (:,3) string    % transitions Tr
        initialStates           (1,:) string    % initial states I
        atomicProps             (1,:) string    % atomic propositions AP
        labels                  (:,2) string    % labels ( state | label ) L
    end
    
    methods
        function obj = TransitionSystem(states, actions, transitions, initialStates, atomicProps, labels)
            %TRANSITIONSYSTEM Construct an instance of this class
            %   Detailed explanation goes here
            obj.states = states;
            obj.actions = actions;
            obj.transitions = transitions;
            obj.initialStates = initialStates;
            obj.atomicProps = atomicProps;
            obj.labels = labels;
        end
        
        function plotedDigraph = plotTS(obj, figureNr)
            %PLOTEDDIGRAPH Plot a transition system as digraph.
            %   Detailed explanation goes here
            arguments
                obj
                figureNr (1,1) double = 0 % the figure number the digraph should be plotted in
            end
            
            % create figure
            if figureNr ~= 0
                figure(figureNr)
            else
                figure
            end
            
            % Digraph
            % table with all states
            NodeTable = table(obj.states', obj.labels(:,2),'VariableNames',{'Name' 'Label'});
            % table with all edges (transitions) and labled with actions
            EdgeTable = table([obj.transitions(:,1) obj.transitions(:,2)], obj.transitions(:,3), 'VariableNames',{'EndNodes' 'Code'});
            % build digraph with it
            graph = digraph(EdgeTable,NodeTable);

            plotedDigraph = plot(graph,'EdgeLabel', graph.Edges.Code, 'NodeLabel', graph.Nodes.Label);
        end
        
        function APStruct = labelToAPStruct(obj, label)
            % LABELTOAPSTRUCT Convert the atomic propositions contained in
            % a label to positive logical values in a structure containing
            % all atomic propositions
            arguments
                obj     (1,1) TransitionSystem 
                label   (1,1) string   % containing labels separeted by space
            end
            
            % generate structure with all APs
            for atomicProp = obj.atomicProps
                APStruct.(atomicProp) = 0;
            end
            % set atomic propositions from label to true            
            activeAPs = split(label)'; % get all active atomic propositions from label (delimiter is " ")
            for atomicProp = activeAPs
                APStruct.(atomicProp) = 1;
            end
        end
        
        function synthesizeWithBA(obj, BA, loopDetectionActive)
            % SYNTHESIZEWITHBA  Synthesize with Büchi Automata to meet the 
            %conditions from this automata
            arguments
                obj                     (1,1) TransitionSystem % the transition system to check
                BA                      (1,1) BuchiAutomata   % the automat to check the transition system with
                loopDetectionActive     (1,1) logical = false; % detection for loops in BA deactivated for performance reasons
            end
            
            %% merge TS and BA
            % States
            Sm = obj.states' +" "+ BA.states;
            % rearrange to row vector
            Sm = Sm(:)';
            
            % Transitions
            Trm = strings(0,0);
            % for every transition from TS
            for i = 1:size(obj.transitions,1)
                % get label from transition target
                labelTarget = obj.labels(obj.transitions(i,2)==obj.labels(:,1),2);
                % get structure with target label atomic props true
                positiveAPs = obj.labelToAPStruct(labelTarget);
                % test which transitions are possible to target with buchi automata
                for j=1:size(BA.transitions,1)
                    if BA.transitions{j,3}(positiveAPs) % check which transition becomes true
                        % Build transition in TS x A
                        Trm(end+1,:) = [join([obj.transitions(i,1) BA.transitions{j,1}]) ...
                                        join([obj.transitions(i,2) BA.transitions{j,2}]) ...
                                        obj.transitions(i,3)];
                    end
                end
            end
            
            
            % Initial states
            Im = strings(0,0);
            % for every TS initial state
            for i=1:length(obj.initialStates)
                % get the label from the initial state
                labelInitState = obj.labels(obj.initialStates(i)==obj.labels(:,1),2);
                % get structure with initial state label atomic props true
                activeAPs = obj.labelToAPStruct(labelInitState);
                % test which initial states are possible with buchi automata
                for j=1:size(BA.transitions,1)
                    % this BA transition starts at a initial state
                    fromBAInitState = any(BA.transitions{j,1} == BA.initialStates);
                    % BA transition is fulfilled by TS initial state label
                    initStateReachable = BA.transitions{j,3}(activeAPs); 
                    if fromBAInitState && initStateReachable
                        % the new state is a TS init state + target of BA transition
                        Im(end+1) = obj.initialStates(i) +" "+ BA.transitions{j,2}; 
                    end
                end
            end
            
            % Atomic propositions
            APm = BA.states;
            
            % Labels
            % Combine to new state-label combination
            Lm(1:length(Sm),1) = Sm'; % label states
            Lm(:,2) = repelem(BA.states,length(obj.states))'; % matching labels
         
            
            
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
            % get all states that are also final state of BA
            for i=1:length(BA.finalStates)
                % get number of characters in final state of BA
                numFinalStateChars = strlength(BA.finalStates(i)); 
                % check position of the final state of BA in state string
                posInString = strfind(Sm,BA.finalStates(i),'ForceCellOutput',true);
                % check for every state if it is containing the final state of BA at correct
                % position
                for j=1:length(Sm)
                    % final state of A in correct position
                    if any(posInString{j} == (strlength(Sm(j)) - (numFinalStateChars - 1)))
                        % add to list of final states
                        FSm(end+1) = Sm(j);
                    end
                end
            end
            
            % find all dead targets
            % remove not final states without any further state
            for state = Sm
                if ~any(state == FSm) && ~any(state == Trm(:,1))
                    Trm(state == Trm(:,2),:) = []; % remove all transitions to this state
                    Sm(state == Sm) = []; % remove state
                end
            end
            
            if ~loopDetectionActive
                % find all not stable final states
                % remove final states that have a successor
                % ATTENTION: this does not work with loops, final state is stable when a
                % loop lead from this state back the state
                for finalState = FSm
                    if any(finalState == Trm(:,1))
                        FSm(finalState == FSm) = [];
                    end
                end
            else
                % find all dead loops
                % delete all loops that are not connected to a init and final state
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
            end
            
            %% delete all states, transitions, etc, from verfiedTS when they are not in TSm
            
            % States
            for state = obj.states
                if ~any(contains(Sm,state)) % search if state is in Sm
                    % if the state is not in Sm, delete it
                    obj.states(state == obj.states) = [];
                end
            end
            
            % Actions
            for action = obj.actions
                if ~any(action == Trm(:,3))
                    % delete all actions that are not used in TSm
                    obj.actions(action == obj.actions) = [];
                end
            end
            
            % Transitions
            % clear atomic proposition of BA from merged transition
            TrmClean = split(Trm(:,1:2));
            TrmClean = [TrmClean(:,:,1) Trm(:,3)];
            for i=size(obj.transitions,1):-1:1
                % check if there is an action in Trm that has the same source, target
                % and action
                if ~any(all(obj.transitions(i,:) == TrmClean,2))
                    obj.transitions(i,:) = []; % delete transition when not in merged
                end
            end
            
            % Initial states
            % get TS inital states out of Im
            ImClean = split(Im," ",3);
            ImClean = ImClean(:,:,1);
            for initState = obj.initialStates
                if ~any(initState == ImClean)
                    % delete init state when not in merged
                    obj.initialStates(initState == obj.initialStates) = [];
                end
            end
            
            % Labels
            % delete label when the state does not exists
            obj.labels(all(obj.labels(:,1) ~= obj.states,2),:) = [];
            
        end
        
    end
    
    methods(Static)
        function parallelizedTS = parallelize(transitionSystems)
            %PARALLELIZE Parallelise two transition systems that are active at
            %the same time.
            %   Detailed explanation goes here
            arguments
                transitionSystems    (1,:) TransitionSystem % the transition systems to parallelize
            end
            
            % if only one transition system is in transition systems take
            % this one
            if length(transitionSystems) == 1
                parallelizedTS = transitionSystems(1);
                return
            end
            
            % Parallelize all transition systems from array
            TSa = transitionSystems(1); % first for combination
            for k = 2:length(transitionSystems)
                
                % transition system to add
                TSb = transitionSystems(k); % second for combination
                
                
                %% States
                % Combine to new states (S1 x S2)
                S = TSa.states'+ TSb.states;
                % rearrange to row vector
                S = S(:)';
                
                %% Actions
                % Combine and add actions
                Act = [TSa.actions, TSb.actions];
                
                
                %% Transitions
                % source | target | action
                Tr = strings(0);
                
                % Combine transitions
                % Get a list where the states from TS1 and TS2 are in S
                S1inS = extractBetween(S,1,strlength(TSa.states(1)));
                S2inS = extractBetween(S,strlength(TSa.states(1))+1,strlength(S));
                
                % transitions from system 1 ( <s1,s2> -a-> <s1',s2> )
                for  transition = TSa.transitions'
                    % check every state from S if it contains the source state
                    for i=1:length(S1inS)
                        if transition(1) == S1inS(i)
                            % make a transition to every matching target state if S2 didnt
                            % change!
                            for j=1:length(S1inS)
                                if (transition(2) == S1inS(j)) && (S2inS(i) == S2inS(j))
                                    % make a transition
                                    Tr(end+1,1:3) = [S(i) S(j) transition(3)]; %#ok<AGROW>
                                end
                            end
                        end
                    end
                end
                
                % transitions from system 2 ( <s1,s2> -b-> <s1,s2'> )
                for  transition = TSb.transitions'
                    % check every state from S if it contains the source state
                    for i=1:length(S2inS)
                        if transition(1) == S2inS(i)
                            % make a transition to every matching target state if S1 didnt
                            % change!
                            for j=1:length(S2inS)
                                if (transition(2) == S2inS(j)) && (S1inS(i) == S1inS(j))
                                    % make a transition
                                    Tr(end+1,1:3) = [S(i) S(j) transition(3)];  %#ok<AGROW>
                                end
                            end
                        end
                    end
                end
                
                %% Initial states
                % combine intial states ( I = I1 x I2 )
                I = TSa.initialStates'+ TSb.initialStates;
                % rearrange to row vector
                I = I(:)';
                
                
                %% Atomic propositions
                % Add atomic propositons
                AP = [TSa.atomicProps, TSb.atomicProps];
                
                
                %% Labels
                % Combine to new state-label combination
                L = strings(0);
                L(:,1) = S'; % label states
                labels = TSa.labels(:,2) +" "+ TSb.labels(:,2)'; % combine the labels like the states
                L(:,2) = labels(:);
                
                % new first transition system for combination
                TSa = TransitionSystem(S, Act, Tr, I, AP, L);
                
            end
            
            % end parallelization when last TS added
            parallelizedTS = TSa;
        end
    end
end

