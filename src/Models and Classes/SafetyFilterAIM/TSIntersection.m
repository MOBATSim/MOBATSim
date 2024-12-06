classdef TSIntersection < TransitionSystem
    %TSCROSSROAD Transition system of the intersection with all vehicles
    %modeled as transition systems according to their planned paths.
    
    % Contributors: Johannes Pintscher
    % Based on: https://github.com/johannes-pi/TSModelCheckerMATLAB
    % License: https://github.com/johannes-pi/TSModelCheckerMATLAB/blob/main/LICENSE
    
    methods
        function obj = TSIntersection(vehDirs)
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
        function [verified, safeActions] = verifyPlannedAIMActions(atInput, atPassing,crossingPaths, plannedActionsByAIM)
            %VERIFYMODEL Verify the selected directions with model checking
            %   Detailed explanation goes here
            arguments
                atInput         (1,:) string % contains directions from vehicles at input nodes
                atPassing       (1,:) string % contains directions from vehicles passing the crossroad
                crossingPaths   (:,2) string % contains string pairs that show crossing paths at crossroad
                plannedActionsByAIM (1,:) string % contains planned actions by AIM to be verified
            end
            
            if isempty(atInput) && isempty(atPassing)
                % no vehicle on crossroad
                verified = true;
                return;
            else
                % Remove same directions (two vehicles starting and heading
                % the same way) as they shouldn't affect the passing order.
                atInput = unique(atInput);
                atPassing = unique(atPassing);
            end
            
            %%
            
            % If both atInput and atPassing are not empty check for the same directions
            if ~(isempty(atInput) || isempty(atPassing))
                % Find the indices of strings that exist in atPassing in atInput
                idx = contains(atInput,intersect(atInput,atPassing));
                if any(idx)
                    % Remove the same direction from the atInput as it should have a pass
                    atInput(idx)=[];
                end
            end
            
            % Get current state and directions from atInput and atPassing
            directions = strings(0,0);
            initialState = strings(1);
            if ~isempty(atInput)
                directions = atInput;
                initialState = join(repmat("i",1,length(atInput)),"");
            end
            if ~isempty(atPassing)
                directions(end+1:end+length(atPassing)) = atPassing;
                initialState = initialState + join(repmat("p",1,length(atPassing)),"");
            end
            
            % Generate Transition System for the intersection
            TS = TSIntersection(directions);
            
            % Generate a verified Transition System with Büchi Automata
            TS.synthesizeWithSPs(crossingPaths);
            
            % Start checking from the initial state as the current state
            currentState = initialState;
            
            % check if state exists
            if any(currentState == TS.states)
                % Get all actions that are safe to be applied in the current state (i.e., not discarded during the synthesis)
                currentActions = TS.actions(contains(TS.actions,directions(1,:)));
                
                
                %% Safety verification (only verify if the planned AIM actions exist in TS_C)
                
                % Format the Planned AIM decisions
                if ~isempty(plannedActionsByAIM)
                    AIMdecisions = split(plannedActionsByAIM,"-");
                    AIMdecisions = unique(append('g',lower(AIMdecisions)))';
                    
                    % Check if the new actions planned by AIM are in the current actions that lead to the safe states
                    if any(ismember(AIMdecisions,currentActions))
                        
                        safeActions = [];
                        
                        for i=1:length(AIMdecisions)
                            if any(all([currentState AIMdecisions(i)] == TS.transitions(:,[1 3]),2))
                                % action exists!! :)
                                % go to state corresponding to this action
                                currentState = TS.transitions(all([currentState AIMdecisions(i)] == TS.transitions(:,[1 3]),2),2);
                                safeActions = [safeActions AIMdecisions(i)];
                            else
                                % Unsafe action/decision by AIM -> Find safe alternatives
                                safeActions = [];
                                verified = false;
                                % Find the only safe actions that exist in TS from the current state and output them
                                allTransitionsFromCurrentState = TS.transitions(:,[1 3]);
                                allActions = allTransitionsFromCurrentState(currentState==allTransitionsFromCurrentState(:,1),2)';
                                
                                for j=1:length(allActions)
                                    if any(all([currentState allActions(j)] == TS.transitions(:,[1 3]),2))
                                        currentState = TS.transitions(all([currentState allActions(j)] == TS.transitions(:,[1 3]),2),2);
                                        safeActions = [safeActions allActions(j)];
                                    end
                                end
                                
                                return % Output verified = false, and safeActions
                            end
                        end
                        
                        verified = true;
                        return
                        
                    else
                        % Unsafe action/decision by AIM -> Find safe alternatives
                        safeActions = [];
                        verified = false;
                        % Find the only safe actions that exist in TS from the current state and output them
                        allTransitionsFromCurrentState = TS.transitions(:,[1 3]);
                        allActions = allTransitionsFromCurrentState(currentState==allTransitionsFromCurrentState(:,1),2)';
                        
                        for j=1:length(allActions)
                            if any(all([currentState allActions(j)] == TS.transitions(:,[1 3]),2))
                                currentState = TS.transitions(all([currentState allActions(j)] == TS.transitions(:,[1 3]),2),2);
                                safeActions = [safeActions allActions(j)];
                            end
                        end
                        
                        return % Output verified = false, and safeActions
                    end
                    
                    
                end
                
                
                
            end
            
        end
    end
end

