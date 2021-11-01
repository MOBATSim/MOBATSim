function verified = verifyWithModel(atInput, atPassing,crossingPaths)
%UNTITLED Verify the selected directions with model checking
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
TS = getTScrossroad(directions);

% Generate a verified Transition System with Büchi Automata
[S, Act, Tr, I, AP, L] = getVerifiedTS(TS.states, TS.actions, TS.transitions, TS.initialStates, TS.atomicProps, TS.labels, crossingPaths);


% check if state exists
if any(currentState == S)
    % get all actions that should be done
    currentActions = Act(contains(Act,directions(1,:)));
    % check all actions
    for i=1:length(currentActions)
        if any(all([currentState currentActions(i)] == Tr(:,[1 3]),2))
            % action exists!! :)
            % go to state corresponding to this action
            currentState = Tr(all([currentState currentActions(i)] == Tr(:,[1 3]),2),2);
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

