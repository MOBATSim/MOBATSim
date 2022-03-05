classdef TScrossroad < TransitionSystem
    %TSCROSSROAD Transition sytem of the whole crossroad with all vehicle
    %transition systems.
    %   Detailed explanation goes here
    
    methods
        function obj = TScrossroad(vehDirs)
            %TSCROSSROAD Directions from the vehicles that are crossing the
            %crossroad.
            %   Detailed explanation goes here
            arguments
                vehDirs                 (1,:) string    % directions of different vehicles on crossroad
            end
            
            % generate a transition system for every vehicle
            for i = 1:length(vehDirs)
                TSvehicles(i) = TSvehicle(vehDirs(i));
            end
            
            % Parallelize all this vehicle transition systems to one crossroad
            % transition system
            TSp = TransitionSystem.parallelize(TSvehicles);
            % Construct transition system
            obj@TransitionSystem(TSp.states, TSp.actions, TSp.transitions, TSp.initialStates, TSp.atomicProps, TSp.labels);
        end
        
        function synthesizeWithSPs(obj, crossingDirections)
            %SYNTHESISWITHSPS Synthesize with safety property to exclude
            %states that does not meet the safety property. The safety
            %property is generated from the crossing directions.
            arguments
                obj                 (1,1) TransitionSystem % the transition system to check and modify
                crossingDirections  (:,2) string % contains string pairs that show crossing paths at crossroad
            end
            
            % Make a safety property for every crossing directions pair and
            % synthesize them with the transition system
            for i = 1:size(crossingDirections,1)
                % Synthesize only directions that affect the transition system
                if all(contains(crossingDirections(i,:), extractAfter(obj.atomicProps,1)))
                    % Generate the Safety Property as a Büchi Automata
                    safetyProperty = SafetyPropertyBA(crossingDirections(i,:));
                    % Verify the TS with BA
                    obj.synthesizeWithBA(safetyProperty);
                end
            end
            
        end
    end
    
    methods(Static)
        function verified = verifyModel(atInput, atPassing,crossingPaths)
            %VERIFYMODEL Verify the selected directions with model checking
            %   Detailed explanation goes here
            arguments
                atInput         (1,:) string % contains directions from vehicles at input nodes
                atPassing       (1,:) string % contains directions from vehicles passing the crossroad
                crossingPaths   (:,2) string % contains string pairs that show crossing paths at crossroad
            end
            
            if isempty(atInput) && isempty(atPassing)
                % no vehicle on crossroad
                verified = true;
                return;
            end
            % Get current state and directions from atInput and atPassing
            directions = strings(0,0);
            currentState = strings(1);
            if ~isempty(atInput)
                directions = atInput;
                currentState = join(repmat("i",1,length(atInput)),"");
            end
            if ~isempty(atPassing)
                directions(end+1:end+length(atPassing)) = atPassing;
                currentState = currentState + join(repmat("p",1,length(atPassing)),"");
            end
            
            % Generate Transition System for the crossroad
            TS = TScrossroad(directions);
            
            % Generate a verified Transition System with Büchi Automata
            TS.synthesizeWithSPs(crossingPaths);
            
            % check if state exists
            if any(currentState == TS.states)
                % get all actions that should be done
                currentActions = TS.actions(contains(TS.actions,directions(1,:)));
                % check all actions
                for i=1:length(currentActions)
                    if any(all([currentState currentActions(i)] == TS.transitions(:,[1 3]),2))
                        % action exists!! :)
                        % go to state corresponding to this action
                        currentState = TS.transitions(all([currentState currentActions(i)] == TS.transitions(:,[1 3]),2),2);
                    else
                        verified = false;
                        return
                    end
                end
                % all actions are possible
                verified = true;
            else
                verified = false;
            end  
            
        end
    end
end

