function [Route_LaneNumber, waypoints, connections_translation, connections_circle, ...
          startingNodes, brakingNodes, stoppingNodes, leavingNodes] = load_Mobatkent_from_opendrive(convert)
      %% Map Data for Mobatkent Map
      
      arguments
          convert (1,1) logical = false
      end
    
    % Run new_map_for_mobatsim.m % this one test the transformation script of Mobatkent Map from drivingScenarioDesigner
    if convert
        [Route_LaneNumber, waypoints, connections_translation, connections_circle] = map_extend_v1();
    else
        load('Mobatkent_converted.mat','Route_LaneNumber','waypoints','connections_translation','connections_circle');
    end
     

    %% Intersection Management Nodes (Waypoints)
    startingNodes = [56    72    73    65;
        51     0     0    44];

    brakingNodes = [ 70    71    74    75;
        78     0     0    76];

    stoppingNodes = [68    62    49    66;
        77     0     0    43];

    leavingNodes = [69    50    67    63;
        0     0    42     0];

end