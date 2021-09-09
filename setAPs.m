function [APsStruct] = setAPs(APsStruct,APsString)
%SETAPS Set all atomic propostions from string in the struct to true
%   Detailed explanation goes here

    % get all atomic propositions from string (delimiter is " ")
    positiveAPs = split(APsString)'; 
    % set all contained atomic propositions in struct to true
    for AP = positiveAPs
        APsStruct.(AP) = 1;
    end
end

    