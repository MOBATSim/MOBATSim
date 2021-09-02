function [S, Act, Tr, I, AP, L] = TSSynthesis(S1, Act1, Tr1, I1, AP1, L1, S2, Act2, Tr2, I2, AP2, L2)
%TSSYNTHESIS Synthesise two transition systems
%   Detailed explanation goes here

%% States
% Combine to new states (S1 x S2)
S = S1'+ S2;
% rearrange to row vector
S = S(:)';

%% Actions
% Combine and add actions
Act = [Act1, Act2, Act1 + Act2];


%% Transitions
% source | target | action
Tr = strings(0);

% Combine transitions
% Get a list where the states from S1 and S2 are in S
S1inS = extractBetween(S,1,strlength(S1(1)));
S2inS = extractBetween(S,strlength(S1(1))+1,strlength(S));

% transitions from system 1 ( <s1,s2> -a-> <s1',s2> )
for  transition = Tr1'
    % check every state from S if it contains the source state
    for i=1:length(S1inS)
        if transition(1) == S1inS(i)
            % make a transition to every matching target state if S2 didnt
            % change!
            for j=1:length(S1inS)
                if (transition(2) == S1inS(j)) && (S2inS(i) == S2inS(j))
                    % make a transition
                    Tr(end+1,1:3) = [S(i) S(j) transition(3)]; %#ok<AGROW>
                end
            end
        end
    end
end

% transitions from system 2 ( <s1,s2> -b-> <s1,s2'> )
for  transition = Tr2'
    % check every state from S if it contains the source state
    for i=1:length(S2inS)
        if transition(1) == S2inS(i)
            % make a transition to every matching target state if S1 didnt
            % change!
            for j=1:length(S2inS)
                if (transition(2) == S2inS(j)) && (S1inS(i) == S1inS(j))
                    % make a transition
                    Tr(end+1,1:3) = [S(i) S(j) transition(3)];  %#ok<AGROW>
                end
            end
        end
    end
end

% transitions from system 1 and system 2 ( <s1,s2> -b-> <s1',s2'> )
for  transition = Tr1'
    % check every state from S if it contains the source state
    for i=1:length(S1inS)
        if transition(1) == S1inS(i)
            % make a transition to every matching target state if both
            % states changed!
            for j=1:length(S1inS)
                if (transition(2) == S1inS(j)) && (S2inS(i) ~= S2inS(j))
                    % get actions
                    actionS1 = transition(3);
                    actionS2 = Tr2( all(Tr2(:,1:2) == [S2inS(i) S2inS(j)],2), 3);
                    % if there is no transition in system 2, dont make a
                    % combined action
                    if isempty(actionS2)
                        break;
                    end
                    % make a transition
                    Tr(end+1,1:3) = [S(i) S(j) actionS1+actionS2]; %#ok<AGROW> % TODO: not nice, better try to use matching action
                end
            end
        end
    end
end

%% Initial states
% combine intial states ( I = I1 x I2 )
I = I1'+ I2;
% rearrange to row vector
I = I(:)';


%% Atomic propositions
% Add
AP = [AP1, AP2];


%% Labels
% Combine to new labels like the states
L = L1' + L2;
% rearrange to row vector
L = L(:)';


end

