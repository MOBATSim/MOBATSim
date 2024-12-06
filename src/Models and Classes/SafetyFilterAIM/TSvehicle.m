classdef TSvehicle < TransitionSystem
    % TSVEHICLE Transition system of a vehicle on an intersection according to its planned path.
    
    % Contributors: Johannes Pintscher
    % Based on: https://github.com/johannes-pi/TSModelCheckerMATLAB
    % License: https://github.com/johannes-pi/TSModelCheckerMATLAB/blob/main/LICENSE
    
    methods
        function obj = TSvehicle(direction)
            %TSVEHICLE Construct an instance of this class

            arguments
                direction   (1,1) string    % direction of the vehicles passing the intersection
            end
            
            % States
            S = ["i" "p" "o"];
            
            % Actions
            Act = "g"+direction;
            
            % Transitions
            % source | target | action
            Tr = [S(1) S(2) Act(1);
                S(2) S(3) Act(1);
                S(3) S(3) Act(1)];
            
            % Initial states
            I = S(1);
            
            % Atomic propositions
            AP = [S(1)+direction S(2)+direction S(3)+direction];
            
            % Labels
            L = [S' AP'];
            
            % generate transition system object
            obj@TransitionSystem(S, Act, Tr, I, AP, L);
        end
        
    end
end

