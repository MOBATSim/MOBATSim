classdef TSIntersection < TransitionSystem
    %TSCROSSROAD Transition system of the intersection with all vehicles
    %modeled as transition systems according to their planned paths.
    
    % Contributors: Johannes Pintscher, Mustafa Saraoglu
    % Based on: https://github.com/johannes-pi/TSModelCheckerMATLAB
    % License: https://github.com/johannes-pi/TSModelCheckerMATLAB/blob/main/LICENSE
    
    %%
    % MIT License
    % 
    % Copyright (c) 2021 johannes-pi
    % 
    % Permission is hereby granted, free of charge, to any person obtaining a copy
    % of this software and associated documentation files (the "Software"), to deal
    % in the Software without restriction, including without limitation the rights
    % to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    % copies of the Software, and to permit persons to whom the Software is
    % furnished to do so, subject to the following conditions:
    % 
    % The above copyright notice and this permission notice shall be included in all
    % copies or substantial portions of the Software.
    % 
    % THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    % IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    % FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    % AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    % LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    % OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    % SOFTWARE.
    %%
    
    methods
        function obj = TSIntersection(vehDirs)
            %TSINTERSECTION Directions from the vehicles that are crossing the intersection.
            arguments
                vehDirs                 (1,:) string    % directions of different vehicles on the intersection
            end
            
            % generate a transition system for every vehicle
            for i = 1:length(vehDirs)
                TSvehicles(i) = TSvehicle(vehDirs(i));
            end
            
            % Parallelize all vehicle transition systems and 
            % generate an interleaved transition system of the intersection
            TSp = TransitionSystem.parallelize(TSvehicles);
            % Construct transition system
            obj@TransitionSystem(TSp.states, TSp.actions, TSp.transitions, TSp.initialStates, TSp.atomicProps, TSp.labels);
        end
        
        function TS_C = synthesizeWithSPs(obj, crossingDirections)
            %SYNTHESISWITHSPS Synthesize with safety property to exclude
            %states that does not meet the safety property. The safety
            %property is generated from the crossing directions.
            arguments
                obj                 (1,1) TransitionSystem % the transition system to check and modify
                crossingDirections  (:,2) string % contains string pairs that show crossing paths at the intersection
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
            
            TS_C = obj; % Assign the verified TS as a new variable TS_C
            
        end
    end
    
    methods(Static)
        function [verified, safeActions] = verifyPlannedAIMActions(atInput, atPassing,crossingPaths, plannedActionsByAIM)
            %VERIFYMODEL Verify the selected directions with model checking
            %   Detailed explanation goes here
            arguments
                atInput         (1,:) string % contains directions from vehicles at input nodes
                atPassing       (1,:) string % contains directions from vehicles passing the intersection
                crossingPaths   (:,2) string % contains string pairs that show crossing paths at the intersection
                plannedActionsByAIM (1,:) string % contains planned actions by AIM to be verified
            end
            
            if isempty(atInput) && isempty(atPassing)
                % no vehicle on the intersection
                verified = true;
                return;
            else
                % Remove same directions (two vehicles starting and heading
                % the same way) as they shouldn't affect the passing order.
                atInput = unique(atInput);
                atPassing = unique(atPassing);
            end
            
            %% If both atInput and atPassing are not empty check for the same directions
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
            
            %% Generate Transition System for the intersection
            TS = TSIntersection(directions);
            
            %% Generate a verified Transition System with Büchi Automata
            TS_C = TS.synthesizeWithSPs(crossingPaths);
            
            %% Alternatively,
            % The generated TS_C (verified transition system according to the safety properties) 
            % can be saved in the current folder. This would allow for the reuse of the safety filter 
            % for the same input at the intersection.
            % This feature is not implemented in this version to avoid overcrowding the folder with 
            % files for all possible combinations of incoming and outgoing vehicles at the intersection.
            % Instead, a new TS_C is generated each time it is required.
            
            %% Start checking from the initial state as the current state
            currentState = initialState;
            
            % check if state exists
            if any(currentState == TS_C.states)
                % Get all actions that are safe to be applied in the current state (i.e., not discarded during the synthesis)
                currentActions = TS_C.actions(contains(TS_C.actions,directions(1,:)));
                
                
                %% Safety verification (only verify if the planned AIM actions exist in TS_C)
                
                % Format the Planned AIM decisions
                if ~isempty(plannedActionsByAIM)
                    AIMdecisions = split(plannedActionsByAIM,"-");
                    AIMdecisions = unique(append('g',lower(AIMdecisions)))';
                    
                    % Check if the new actions planned by AIM are in the current actions that lead to the safe states
                    if any(ismember(AIMdecisions,currentActions))
                        
                        safeActions = [];
                        
                        for i=1:length(AIMdecisions)
                            if any(all([currentState AIMdecisions(i)] == TS_C.transitions(:,[1 3]),2))
                                % action exists!! :)
                                % go to state corresponding to this action
                                currentState = TS_C.transitions(all([currentState AIMdecisions(i)] == TS_C.transitions(:,[1 3]),2),2);
                                safeActions = [safeActions AIMdecisions(i)];
                            else
                                % Unsafe action/decision by AIM -> Find safe alternatives
                                safeActions = [];
                                verified = false;
                                % Find the only safe actions that exist in TS_C from the current state and output them
                                allTransitionsFromCurrentState = TS_C.transitions(:,[1 3]);
                                allActions = allTransitionsFromCurrentState(currentState==allTransitionsFromCurrentState(:,1),2)';
                                
                                for j=1:length(allActions)
                                    if any(all([currentState allActions(j)] == TS_C.transitions(:,[1 3]),2))
                                        currentState = TS_C.transitions(all([currentState allActions(j)] == TS_C.transitions(:,[1 3]),2),2);
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
                        % Find the only safe actions that exist in TS_C from the current state and output them
                        allTransitionsFromCurrentState = TS_C.transitions(:,[1 3]);
                        allActions = allTransitionsFromCurrentState(currentState==allTransitionsFromCurrentState(:,1),2)';
                        
                        for j=1:length(allActions)
                            if any(all([currentState allActions(j)] == TS_C.transitions(:,[1 3]),2))
                                currentState = TS_C.transitions(all([currentState allActions(j)] == TS_C.transitions(:,[1 3]),2),2);
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

