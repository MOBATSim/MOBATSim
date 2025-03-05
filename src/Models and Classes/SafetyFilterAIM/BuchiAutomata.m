classdef BuchiAutomata < handle
    % BUCHI AUTOMATA: Automata with states, symbols, transitions, initial and final states.
    
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
    
     properties
        states                  (1,:) string    % states Q
        symbols                 (1,:) cell      % symbols Sig
        transitions             (:,3) cell      % transitions delta
        initialStates           (1,:) string    % initial states Q0
        finalStates             (1,:) string    % final states F
    end
    
    methods
        function obj = BuchiAutomata(states, symbols, transitions, initialStates, finalStates)
            %BUCHIAUTOMATA Construct an instance of this class
            obj.states = states;
            obj.symbols = symbols;
            obj.transitions = transitions;
            obj.initialStates = initialStates;
            obj.finalStates = finalStates;
        end      
    end
end

