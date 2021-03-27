% this script is for the Bird eye view of the 2D simulation
 save ('.\src\Misc\BirdEyeView_Visualization\test.mat');%save the data in workspace to the file
 run DrivingScenarioDesigner.m 
 %sim('ScenarioAnimation.slx')
 open('ScenarioAnimation.slx')