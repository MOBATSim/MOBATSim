% this script is for the Bird eye view of the 2D simulation
% update the map information file after every map update
% make sure every vehicle will not stop during simulation time
 save ('.\src\Misc\BirdEyeView_Visualization\vehicleTrajectories.mat');%save the data in workspace to the file
 run DrivingScenarioDesigner.m 
 %sim('ScenarioAnimation.slx')
 open('ScenarioAnimation.slx')