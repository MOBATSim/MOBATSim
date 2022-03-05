function invalidNodes = checkNodesValid(occupiedNodes,forbiddenNodes)
%CHECKNODESVALID Test of all occupied nodes are not a forbidden node
%   Detailed explanation goes here
    invalidNodes = [];
    % Check all elements
    for i=1:numel(occupiedNodes)
        % check if a forbidden node
        if any(occupiedNodes(i) == forbiddenNodes,'all')
            invalidNodes(end+1) = occupiedNodes(i); %#ok<AGROW>
        end
    end
end

