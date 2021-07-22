%% Run this script for the Bird-Eye View of the 2D simulation
% update the map information file after every map update by running this
% function which creates the drivingScenario object that is to be used by
% the ScenarioReader block in ScenarioAnimation.slx

%% Add all road segments
MOBATKent_Scenario = scenario_map_v1(); % export from drivingScenarioDesigner

%% Load all trajectories
step_length = 1; % Sampling of WAYPOINTS and SPEEDS

% To define the Ego Vehicle, just put the code of the corresponding vehicle to the first place.
egoVehicle = vehicle(MOBATKent_Scenario, ...
    'ClassID', 2, ...
    'Position', [-V2_translation.Data(1,3) -V2_translation.Data(1,1) 0]);
waypoints = [-V2_translation.Data((1:step_length:end),3) -V2_translation.Data((1:step_length:end),1) V2_translation.Data((1:step_length:end),2)];
speed = V2_speed.Data(1:step_length:end);

defineTrajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
car1 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 1, ...
    'Position', [-V1_translation.Data(1,3) -V1_translation.Data(1,1) 0]);
waypoints = [-V1_translation.Data((1:step_length:end),3) -V1_translation.Data((1:step_length:end),1) V1_translation.Data((1:step_length:end),2)];
speed = V1_speed.Data(1:step_length:end);

defineTrajectory(car1, waypoints, speed);

car3 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 3, ...
    'Position', [-V3_translation.Data(1,3) -V3_translation.Data(1,1) 0]);
waypoints = [-V3_translation.Data((1:step_length:end),3) -V3_translation.Data((1:step_length:end),1) V3_translation.Data((1:step_length:end),2)];
speed = V3_speed.Data(1:step_length:end);

defineTrajectory(car3, waypoints, speed);

car4 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 4, ...
    'Position', [-V4_translation.Data(1,3) -V4_translation.Data(1,1) 0]);
waypoints = [-V4_translation.Data((1:step_length:end),3) -V4_translation.Data((1:step_length:end),1) V4_translation.Data((1:step_length:end),2)];
speed = V4_speed.Data(1:step_length:end);

defineTrajectory(car4, waypoints, speed);

car5 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 5, ...
    'Position', [-V5_translation.Data(1,3) -V5_translation.Data(1,1) 0]);
waypoints = [-V5_translation.Data((1:step_length:end),3) -V5_translation.Data((1:step_length:end),1) V5_translation.Data((1:step_length:end),2)];
speed = V5_speed.Data(1:step_length:end);

defineTrajectory(car5, waypoints, speed);

car6 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 6, ...
    'Position', [-V6_translation.Data(1,3) -V6_translation.Data(1,1) 0]);
waypoints = [-V6_translation.Data((1:step_length:end),3) -V6_translation.Data((1:step_length:end),1) V6_translation.Data((1:step_length:end),2)];
speed = V6_speed.Data(1:step_length:end);

defineTrajectory(car6, waypoints, speed);

car7 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 7, ...
    'Position', [-V7_translation.Data(1,3) -V7_translation.Data(1,1) 0]);
waypoints = [-V7_translation.Data((1:step_length:end),3) -V7_translation.Data((1:step_length:end),1) V7_translation.Data((1:step_length:end),2)];
speed = V7_speed.Data(1:step_length:end);

defineTrajectory(car7, waypoints, speed);

car8 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 8, ...
    'Position', [-V8_translation.Data(1,3) -V8_translation.Data(1,1) 0]);
waypoints = [-V8_translation.Data((1:step_length:end),3) -V8_translation.Data((1:step_length:end),1) V8_translation.Data((1:step_length:end),2)];
speed = V8_speed.Data(1:step_length:end);

defineTrajectory(car8, waypoints, speed);

car9 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 9, ...
    'Position', [-V9_translation.Data(1,3) -V9_translation.Data(1,1) 0]);
waypoints = [-V9_translation.Data((1:step_length:end),3) -V9_translation.Data((1:step_length:end),1) V9_translation.Data((1:step_length:end),2)];
speed = V9_speed.Data(1:step_length:end);

defineTrajectory(car9, waypoints, speed);

car10 = vehicle(MOBATKent_Scenario, ...
    'ClassID', 10, ...
    'Position', [-V10_translation.Data(1,3) -V10_translation.Data(1,1) 0]);
waypoints = [-V10_translation.Data((1:step_length:end),3) -V10_translation.Data((1:step_length:end),1) V10_translation.Data((1:step_length:end),2)];
speed = V10_speed.Data(1:step_length:end);

defineTrajectory(car10, waypoints, speed);

%% Open the Simulink Model for the ScenarioReader block and the Bird-Eye View visualization
open('ScenarioAnimation.slx')

function defineTrajectory(car, waypoints, speed)
try
    trajectory(car, waypoints, speed)
catch
    [~,I,~] = unique(waypoints, 'rows','stable');
    waypoints = waypoints(I,:,:);
    speed = speed(I,:,:);
    try
        trajectory(car, waypoints, speed)
    catch
        idx =(speed~=0);
        speed = (speed(idx));
        waypoints = waypoints(idx,:,:);
        try
            trajectory(car, waypoints, speed);
        catch
            % The last try to generate waypoints by skipping the first 10 waypoints
            waypoints = waypoints(10:end,:,:);
            speed = speed(10:end);
            trajectory(car, waypoints, speed);
        end
    end
end
end