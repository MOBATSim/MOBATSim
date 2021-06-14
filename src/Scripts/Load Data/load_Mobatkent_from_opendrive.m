function [Route_LaneNumber, mapName, waypoints, connections_translation, connections_circle, ...
          startingNodes, breakingNodes, stoppingNodes, leavingNodes] = load_Mobatkent_from_opendrive(convert)
      arguments
          convert (1,1) logical = false
      end
      
    %load the new mobatkent map from drivingScenarioDesigner
    mapName = 'Mobatkent';
    
    % Run new_map_for_mobatsim.m % this one test the transformation script
    if convert
        [Route_LaneNumber, waypoints, connections_translation, connections_circle] = map_extend_v1();
    else
        load('Mobatkent_converted.mat','Route_LaneNumber','waypoints','connections_translation','connections_circle');
    end
     

    %% Intersection Management Nodes (Waypoints)
    startingNodes = [56    72    73    65;
        51     0     0    44];

    breakingNodes = [ 70    71    74    75;
        78     0     0    76];

    stoppingNodes = [68    62    49    66;
        77     0     0    43];

    leavingNodes = [69    50    67    63;
        0     0    42     0];

end