% Contributors: Qianwei Yang, Mustafa Saraoglu

%% Run this script for the Cuboid World or Unreal Engine 4 Simulation of the 2D MOBATSim simulation
% This script:
% 1- should be run after finishing the main simulation on the 2D plot because the logged data is required for this script.
% 2- has to be run in order to update the trajectories after each driving scenario in the main simulation.
% 3- opens the drivingScenarioDesigner APP to simulate the vehicle trajectories in the Cuboid World or using Unreal Engine 4

%% Load all the road information 
MOBATKent_Scenario = scenario_map_v1(); % Output of a function automatically generated from the drivingScenarioDesigner after designing the roads

%% Load all vehicles and generate Trajectories
step_length = 20; % Sampling of WAYPOINTS and SPEEDS -> if this script takes too long to run, increase this number!

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

%% This function uses try-catch-try structure as a workaround the geometric errors caused by the "trajectory" function
function car = defineTrajectory(car, waypoints, speed)
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
            % Try to generate waypoints by skipping the first 10 waypoints
            waypoints = waypoints(10:end,:,:);
            speed = speed(10:end);
            try
                trajectory(car, waypoints, speed);
            catch
                % The last try: Sample each waypoint in 5
                I = 1:5:length(waypoints);
                waypoints = waypoints(I,:,:);
                speed = speed(I,:,:);
                
                % If this also doesn't work, then it simply fails, try changing the scenario.
                trajectory(car, waypoints, speed);
            end
        end
    end
end
end
