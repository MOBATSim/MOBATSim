function [newTrm] = DFS_fromInitialState(Trm,initialState)
%   DFS Algorithm to get all reachable states from the initial state

% Contributors: Mustafa Saraoglu
% Based on: https://github.com/MOBATSim/TSModelCheckerMATLAB which is
% forked from: https://github.com/johannes-pi/TSModelCheckerMATLAB

notVisitedStates = unique([Trm(:,1);Trm(:,2)]); % Gather all the unique states
notVisitedStates(1,:) = []; % Remove the initial State

allVisitedStates = initialState; % Start with the initial State
PushedStates = Trm(Trm(:,1) == initialState,:); % Push the states around the initial state

while ~isempty(PushedStates) % Continue until no more states to push
    
    % Visit state
    State2Visit = PushedStates(1,2);
    
    if any(contains(notVisitedStates,State2Visit))
        allVisitedStates = [allVisitedStates; State2Visit];
        notVisitedStates(contains(notVisitedStates,State2Visit)) = [];
        PushedStates(1,:) = [];
        newPushedStates = Trm(Trm(:,1) == State2Visit,:);
        PushedStates = [PushedStates; newPushedStates];
    else
        PushedStates(1,:) = []; % Remove from the stack
        
    end
    
end

newTrm = Trm(ismember(Trm(:,1),allVisitedStates),:);
end