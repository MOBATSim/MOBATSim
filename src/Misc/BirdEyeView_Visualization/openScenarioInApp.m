%% Run this script for the Cuboid World or Unreal Engine 4 Simulation of the 2D MOBATSim simulation
% update the map information file after every map update by running this
% function which creates the drivingScenario object that is to be used by 
% the DrivingScenarioDesigner APP

%% Add all road segments
MOBATKent_Scenario = scenario_map_v1(); % export from drivingScenarioDesigner

%% Load all trajectories
step_length = 1; % Sampling of WAYPOINTS and SPEEDS

% To define the Ego Vehicle, just put the code of the corresponding vehicle to the first place.
egoVehicle = vehicle(MOBATKent_Scenario, ...
    'ClassID', 1, ...
    'Position', [-V2_translation.Data(1,3) -V2_translation.Data(1,1) 0]);
    waypoints = [-V2_translation.Data((1:step_length:end),3) -V2_translation.Data((1:step_length:end),1) V2_translation.Data((1:step_length:end),2)];
    speed = V2_speed.Data(1:step_length:end);
    
    egoVehicle = defineTrajectory(egoVehicle, waypoints, speed);
    
% Add the non-ego actors
car1 = vehicle(MOBATKent_Scenario, ...
     'ClassID', 1, ...
     'Position', [-V1_translation.Data(1,3) -V1_translation.Data(1,1) 0]);
     waypoints = [-V1_translation.Data((1:step_length:end),3) -V1_translation.Data((1:step_length:end),1) V1_translation.Data((1:step_length:end),2)];
     speed = V1_speed.Data(1:step_length:end);
     
     car1 = defineTrajectory(car1, waypoints, speed);
    
car3 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 1, ...
    'Position', [-V3_translation.Data(1,3) -V3_translation.Data(1,1) 0]);
    waypoints = [-V3_translation.Data((1:step_length:end),3) -V3_translation.Data((1:step_length:end),1) V3_translation.Data((1:step_length:end),2)];
    speed = V3_speed.Data(1:step_length:end);
    
    car3 = defineTrajectory(car3, waypoints, speed);
    
car4 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 1, ...
    'Position', [-V4_translation.Data(1,3) -V4_translation.Data(1,1) 0]);
    waypoints = [-V4_translation.Data((1:step_length:end),3) -V4_translation.Data((1:step_length:end),1) V4_translation.Data((1:step_length:end),2)];
    speed = V4_speed.Data(1:step_length:end);
    
    car4 = defineTrajectory(car4, waypoints, speed);

car5 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 1, ...
    'Position', [-V5_translation.Data(1,3) -V5_translation.Data(1,1) 0]);
    waypoints = [-V5_translation.Data((1:step_length:end),3) -V5_translation.Data((1:step_length:end),1) V5_translation.Data((1:step_length:end),2)];
    speed = V5_speed.Data(1:step_length:end);
    
    car5 = defineTrajectory(car5, waypoints, speed);
    
 car6 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 1, ...
    'Position', [-V6_translation.Data(1,3) -V6_translation.Data(1,1) 0]);
    waypoints = [-V6_translation.Data((1:step_length:end),3) -V6_translation.Data((1:step_length:end),1) V6_translation.Data((1:step_length:end),2)];
    speed = V6_speed.Data(1:step_length:end);
    
    car6 = defineTrajectory(car6, waypoints, speed);
    
 car7 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 1, ...
    'Position', [-V7_translation.Data(1,3) -V7_translation.Data(1,1) 0]);
    waypoints = [-V7_translation.Data((1:step_length:end),3) -V7_translation.Data((1:step_length:end),1) V7_translation.Data((1:step_length:end),2)];
    speed = V7_speed.Data(1:step_length:end);
    
    car7 = defineTrajectory(car7, waypoints, speed);
    
 car8 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 1, ...
    'Position', [-V8_translation.Data(1,3) -V8_translation.Data(1,1) 0]);
    waypoints = [-V8_translation.Data((1:step_length:end),3) -V8_translation.Data((1:step_length:end),1) V8_translation.Data((1:step_length:end),2)];
    speed = V8_speed.Data(1:step_length:end);
    
    car8 = defineTrajectory(car8, waypoints, speed);
    
car9 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 1, ...
    'Position', [-V9_translation.Data(1,3) -V9_translation.Data(1,1) 0]);
    waypoints = [-V9_translation.Data((1:step_length:end),3) -V9_translation.Data((1:step_length:end),1) V9_translation.Data((1:step_length:end),2)];
    speed = V9_speed.Data(1:step_length:end);
    
    car9 = defineTrajectory(car9, waypoints, speed);
    
car10 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 1, ...
    'Position', [-V10_translation.Data(1,3) -V10_translation.Data(1,1) 0]);
    waypoints = [-V10_translation.Data((1:step_length:end),3) -V10_translation.Data((1:step_length:end),1) V10_translation.Data((1:step_length:end),2)];
    speed = V10_speed.Data(1:step_length:end);
    
    car10 = defineTrajectory(car10, waypoints, speed);
    
    %% Open the scenario in drivingScenarioDesigner (This way it can be also edited or simulated 3D in Unreal Engine)
    drivingScenarioDesigner(MOBATKent_Scenario)

function car = defineTrajectory(car, waypoints, speed)
try
    trajectory(car, waypoints, speed)
catch
    [~,I,~] = unique(waypoints, 'rows','stable');
    waypoints = waypoints(I,:,:);
    speed = speed(I,:,:);
    trajectory(car, waypoints, speed);
end

end


    


