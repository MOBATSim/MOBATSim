function allActiveAPs = stringToStructAPs(stringAllAPs, stringActiveAPs)
%SETAPS Generate a structure with all active APs set to true
%   Detailed explanation goes here
    
arguments
        stringAllAPs        (1,:) string        % all atomic proposition as string array
        stringActiveAPs     (1,:) string        % string with atomic propostions that are active
end

% generate structure with every AP
for stringAP = stringAllAPs
    allActiveAPs.(stringAP) = 0;
end
% set all active APs to true
% get all active atomic propositions from string (delimiter is " ")
activeAPsTarget = split(stringActiveAPs)';
for positiveAP = activeAPsTarget
    allActiveAPs.(positiveAP) = 1;
end

end

    