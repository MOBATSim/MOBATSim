function result = checkSafetyCondition(pathA, pathB, allAPs, equationNr)
%SAFETYCONDITION check if the safety condition holds the selected equation
%   The result is positive if the equation is true

if ~all(isfield(allAPs, [pathA pathB]))
    % Safety condition not relevant for this set of atomic propositions
    error("Wrong safety condition used for the model. Use only one where all atomic propositions contained in model.");
end

% directions of crossing vehicles
dir = extractAfter([pathA pathB],1);

% equations
if equationNr == 1
    result = ~(allAPs.(pathA) && allAPs.(pathB)) && ~allAPs.("o"+dir(1)) && ~allAPs.("o"+dir(2));
elseif equationNr == 2
    result = allAPs.("o"+dir(1)) || allAPs.("o"+dir(2));
elseif equationNr == 3
    result = 1;
else
    error("Equation does not exist. Wrong equation number used.");
end

end

