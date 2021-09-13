function [APsStructs,APsStruct] = stringToAPs(APsStringTrue, APsStringFalse)
%SETAPS Generate atomic proposition pairs from input. The first AP is true, the other one false. 
%   Detailed explanation goes here
    
arguments
        APsStringTrue   (1,:) string        % string with atomic propostions that should be true
        APsStringFalse  (1,:) string = [];  % string with atomic propostions that should be false
    end
    % get all atomic propositions from string (delimiter is " ")
    positiveAPsTarget = split(APsStringTrue)';
    % get all atomic propositions from string (delimiter is " ")
    positiveAPsSource = split(APsStringFalse)';
    % for APsStructs only return changes TODO: document and clean up this
    % method
    changedElements = (positiveAPsTarget ~= positiveAPsSource);

    % 1. output: all APs in one struct
    APsStructs = [];
    %APStruct = allAPs;
    % set all contained atomic propositions in one struct
    for i=1:length(positiveAPsTarget)
        APsStruct.(positiveAPsTarget(i)) = 1;
    end
    for i=1:length(positiveAPsSource)
        APsStruct.(positiveAPsSource(i)) = 0;
    end
    
    % 2. output: every AP pairs in one struct
    j = 1;
    for i=1:length(changedElements)
        if changedElements(i)
            APs = [];
            if positiveAPsSource(i) ~= ""
                APs.(positiveAPsSource(i)) = 0; % set corresponding input AP to false, because passed crossroad
            end
            APs.(positiveAPsTarget(i)) = 1; % set corresponding output AP to true, because passed crossroad
            
            APsStructs{end+1} = APs;
            j = j+1;
        end
    end
end

    