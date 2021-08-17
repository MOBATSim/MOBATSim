function valid = checkNodesValid(occupiedNodes,forbiddenNodes)
%CHECKNODESVALID Test of all occupied nodes are not a forbidden node
%   Detailed explanation goes here

    % Check all elements
    for i=1:numel(occupiedNodes)
        % check if a forbidden node
        if any(occupiedNodes(i) == forbiddenNodes,'all')
            valid = false;
            return;
        end
    end
    
    % No invalid element
    valid = true;
end

