classdef SafetyPropertyBA < BuchiAutomata
    %SAFETYPROPERTYBA A safety property that prohibits two vehicles coming from directions that overlap on the intersection.
    %   This is a specific Büchi Automata.
    
    % Contributors: Johannes Pintscher
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

