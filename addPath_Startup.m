% Add all the folders and subfolders to path (In case if the Project startup add to path doesn't work properly)
%addpath(genpath('img'));
%addpath(genpath('src'));

if isempty(matlab.project.rootProject)
    uiopen('\MOBATSim.prj',1);
end