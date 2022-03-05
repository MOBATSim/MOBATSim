classdef BuchiAutomata < handle
    %BUCHIAUTOMATA Automata with states, symbols, transitions, initial and
    %final states.
    %   Detailed explanation goes here
    
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
            %   Detailed explanation goes here
            obj.states = states;
            obj.symbols = symbols;
            obj.transitions = transitions;
            obj.initialStates = initialStates;
            obj.finalStates = finalStates;
        end      
    end
end

