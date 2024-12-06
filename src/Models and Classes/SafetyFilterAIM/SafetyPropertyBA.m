classdef SafetyPropertyBA < BuchiAutomata
    %SAFETYPROPERTYBA A safety property that prohibits two vehicles coming from directions that overlap on the intersection.
    %   This is a specific Büchi Automata.
    
    % Contributors: Johannes Pintscher
    % Based on: https://github.com/johannes-pi/TSModelCheckerMATLAB
    % License: https://github.com/johannes-pi/TSModelCheckerMATLAB/blob/main/LICENSE
    
    methods
        function obj = SafetyPropertyBA(crossingDirections)
            %SAFETYPROPERTYBA  A property that prohibits two directions are driven at the
            %same time.

            arguments
                crossingDirections  (1,2) string % two directions that are crossing each other
            end
            
            % States Q
            states = ["q0" "q1"];
            
            % Symbols Sigma (transition conditions)
            % conditions for the transition between states
            
            dir1 = crossingDirections(1);
            dir2 = crossingDirections(2);
            
            symbols = {@(allAPs) (~(allAPs.("p"+dir1) && allAPs.("p"+dir2)) && ~allAPs.("o"+dir1) && ~allAPs.("o"+dir2)) ...
                       @(allAPs) (allAPs.("o"+dir1) || allAPs.("o"+dir2)) ...
                       @(allAPs) (1)};
            
            % Transitions delta
            transitions = {"q0" "q0" symbols{1}; ...
                           "q0" "q1" symbols{2}; ...
                           "q1" "q1" symbols{3}};
            
            % Initial states
            initialStates = "q0";
            
            % Final states
            finalStates = "q1";
            
            % Construct Buchi Automata
            obj@BuchiAutomata(states, symbols, transitions, initialStates, finalStates);
        end
    end
end

