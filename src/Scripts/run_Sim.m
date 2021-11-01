% This function is to make sure that the "To Workspace" blocks store the data in the base workspace when simulation
% is triggered from the GUI
sim(evalin('base','configs.modelName'));