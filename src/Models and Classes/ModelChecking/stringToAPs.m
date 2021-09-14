function [APsStructs,APsStruct] = stringToAPs(APsStringTrue, APsStringFalse)
%SETAPS Generate atomic proposition pairs from input. The first AP is true, the other one false. 
%   Detailed explanation goes here
    
arguments
        APsStringTrue   (1,:) string        % string with atomic propostions that should be true
        APsStringFalse  (1,:) string = [];  % string with atomic propostions that should be false
end

allAPs.changed = 0;
allAPs.in = 0;
allAPs.ie = 0;
allAPs.is = 0;
allAPs.iw = 0;
allAPs.on = 0;
allAPs.oe = 0;
allAPs.os = 0;
allAPs.ow = 0;


    % get all atomic propositions from string (delimiter is " ")
    positiveAPsTarget = split(APsStringTrue)';
    % get all atomic propositions from string (delimiter is " ")
    positiveAPsSource = split(APsStringFalse)';
    % for APsStructs only return changes TODO: document and clean up this
    % method
    APschanged = (positiveAPsTarget ~= positiveAPsSource);

    % 1. output: all APs in one struct
    APsStructs = [];
    %APStruct = allAPs;
    % set all contained atomic propositions in one struct
%     for i=1:length(positiveAPsSource)
%         APsStruct.(positiveAPsSource(i)) = 0;
%     end
    nrAPsPairs = length(positiveAPsTarget);
    % set all APsStructs to pre state
    for i = 1:nrAPsPairs
        allAPs.(positiveAPsSource(i)) = 1;
    end
    APsStruct = repmat(allAPs,nrAPsPairs,1);
    % for every partial action change corresponding APs
    for i = 1:nrAPsPairs
        if APschanged(i) == true
            APsStruct(i).(positiveAPsSource(i)) = 0;
            APsStruct(i).(positiveAPsTarget(i)) = 1;
            APsStruct(i).changed = 1;
        end
    end
    
    % 2. output: every AP pairs in one struct
    j = 1;
    for i=1:length(APschanged)
        if APschanged(i)
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

    