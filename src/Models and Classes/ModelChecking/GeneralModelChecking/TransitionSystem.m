classdef TransitionSystem
    %TRANSITIONSYSTEM State system with states, actions, transitions,
    %inital states, atomic propositions and labels.
    %   Detailed explanation goes here
    
    properties
        states                  (1,:) string    % states
        actions                 (1,:) string    % actions
        transitions             (:,3) string    % transitions
        initialStates           (1,:) string    % initial states
        atomicProps             (1,:) string    % atomic propositions
        labels                  (:,2) string    % labels ( state | label )
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
            %PLOTTS Plot a transition system as digraph.
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

