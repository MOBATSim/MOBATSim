mapName = 'Mobatkent';
%load the new mobatkent map from drivingScenarioDesigner
run new_map_for_mobatsim.m

startingNodes = [56    72    73    65;
    51     0     0    44];

breakingNodes = [ 70    71    74    75;
    78     0     0    76];

stoppingNodes = [68    62    49    66;
    77     0     0    43];

leavingNodes = [69    50    67    63;
    0     0    42     0];

assignin('base','mapName',mapName);
assignin('base','waypoints',waypoints);
assignin('base','connections_circle',connections_circle);
assignin('base','connections_translation',connections_translation);

assignin('base','startingNodes',startingNodes);
assignin('base','breakingNodes',breakingNodes);
assignin('base','stoppingNodes',stoppingNodes);
assignin('base','leavingNodes',leavingNodes);