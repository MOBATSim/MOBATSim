function blkStruct = slblocks
% This function specifies that the library 'MOBATSim_Library'
% should appear in the Library Browser with the 
% name 'MOBATSim'

% Contributors: Laura Slabon

    Browser.Library = 'MOBATSim_Library';
    % 'MOBATSim_Library' is the name of the library

    Browser.Name = 'MOBATSim';
    % 'MOBATSim' is the library name that appears
    % in the Library Browser

    blkStruct.Browser = Browser;