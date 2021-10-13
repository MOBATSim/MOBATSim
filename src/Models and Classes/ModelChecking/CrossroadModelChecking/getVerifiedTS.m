function [S, Act, Tr, I, AP, L] = getVerifiedTS(S, Act, Tr, I, AP, L, crossingPaths)
%CHECKALLSP Check all safety properties at transition system and remove
%invalid states.
%   Detailed explanation goes here

% Check which parts of TS are still valid with A
for i = 1:size(crossingPaths,1)
    % Generate the Büchi Automata for the current safety property
    [Q, Sig, delta, Q0, F] = getBApaths(crossingPaths(i,:));
    % Check if this safety property is relevant for this model
    if all(contains(crossingPaths(i,:), AP))
        % Verify the TS with BA
        [S, Act, Tr, I, AP, L] = TsACheck(S, Act, Tr, I, AP, L, Q, Sig, delta, Q0, F);
    end
end

end

