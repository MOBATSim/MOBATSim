% Contributors: Johannes Pintscher, Mustafa Saraoglu

%% Starts the simulation
% Before running this script, make sure that the necessary simulation data is loaded (prepare_simulator.m)

% Turns off the automatic MPC-related messages in the command window
mpcverbosity off;

% This function is to make sure that the "ToWorkspace" blocks save the simulation data in the base workspace 
% when the simulation is started from the GUI
sim(evalin('base','configs.modelName'));