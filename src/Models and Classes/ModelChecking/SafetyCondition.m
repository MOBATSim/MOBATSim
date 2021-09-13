function result = SafetyCondition(allowedFieldsA, allowedFieldsB, APsA, APsB,equationNr)
%SAFETYCONDITION General structure of the two vehicles crossing safety condition, return the
%result of the equation, after checking the allowed atomic propositions
%   The result is positive if the equation is true or if the wrong atomic propositions are used

% Check if the right APs are used
if ~all(isfield(APsA, allowedFieldsA)) || ~all(isfield(APsB, allowedFieldsB))
    % try to swap atomic propositions 
    if all(isfield(APsA, allowedFieldsB)) && all(isfield(APsB, allowedFieldsA))
        APsTemp = APsA;
        APsA = APsB;
        APsB = APsTemp;
    else % this safety condtion could not be used, dont influence the result
        result = 1;
        return
    end
end
    

% atomic propositions from two vehicles
a1 = allowedFieldsA{1};
a2 = allowedFieldsA{2};
b1 = allowedFieldsB{1};
b2 = allowedFieldsB{2};

% equations
if equationNr == 1
    result = (APsA.(a1) && APsB.(b1) && ~APsB.(b2))||(APsA.(a1) && APsB.(b1) && ~APsA.(a2));
elseif equationNr == 2
    result = (~APsA.(a1) && ~APsB.(b2))||(~APsA.(a1) && ~APsA.(a2))||(~APsB.(b1) && ~APsB.(b2))||(~APsB.(b1) && ~APsA.(a2)); 
elseif equationNr == 3
    result = 1;
else
    error("Equation does not exist. Wrong equation number used.");
end

end

