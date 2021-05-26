function [scenario, egoVehicle] = DrivingScenarioDesigner()
% Returns the drivingScenario defined in the Designer

%get scenario ans
% Command summary goes here
% prepare_simulator();
% run_Sim();
% model_name = 'MOBATSim';
% t = get_param(model_name,'StopTime');
% Construct a drivingScenario object.
% scenario = drivingScenario;

%% Add all road segments
scenario = scenario_map_v1(); % export from drivingScenarioDesigner

%% Add the ego vehicle
% ans = ans;
% load('matlab.mat')
load('vehicleTrajectories.mat');
step_length = 1; % TODO, make a try catch or better solve the trajectory error with same WAYPOINTS with different SPEEDS error


% temp1 = [ans.logsout{1}.Values.Data(1,1) -ans.logsout{1}.Values.Data(1,3) 0];
% temp2 = [ans.logsout{1}.Values.Data(:,1) -ans.logsout{1}.Values.Data(:,3) ans.logsout{1}.Values.Data(:,2)]
%if you want to change the ego vehicle, just put the code of the corresponding vehicle to the first place.
egoVehicle = vehicle(scenario, ...
    'ClassID', 2, ...
    'Position', [-V2_translation.Data(1,3) -V2_translation.Data(1,1) 0]);
    waypoints = [-V2_translation.Data((1:step_length:end),3) -V2_translation.Data((1:step_length:end),1) V2_translation.Data((1:step_length:end),2)];
    speed = V2_speed.Data(1:step_length:end);
    trajectory(egoVehicle, waypoints, speed);
% Add the non-ego actors
% car1 = vehicle(scenario, ...
%     'ClassID', 1, ...
%     'Position', [str2double(get_param(append(model_name,'/Vehicle Model 1 - Manual'),'X_init')) str2double(get_param(append(model_name,'/Vehicle Model 1 - Manual'),'Y_init')) 0]);
%     waypoints = [ans.v1_info.InertFrm.Cg.Disp.X.Data ans.v1_info.InertFrm.Cg.Disp.Y.Data ans.v1_info.InertFrm.Cg.Disp.Z.Data];
%     speed = sqrt(ans.v1_info.InertFrm.Cg.Vel.Xdot.Data.^2+ans.v1_info.InertFrm.Cg.Vel.Ydot.Data.^2);
%     trajectory(car1, waypoints, speed);
car1 = vehicle(scenario, ...
     'ClassID', 1, ...
     'Position', [-V1_translation.Data(1,3) -V1_translation.Data(1,1) 0]);
     waypoints = [-V1_translation.Data((1:step_length:end),3) -V1_translation.Data((1:step_length:end),1) V1_translation.Data((1:step_length:end),2)];
     speed = V1_speed.Data(1:step_length:end);
     trajectory(car1, waypoints, speed);
    
car3 = vehicle(scenario, ...
    'ClassID', 3, ...
    'Position', [-V3_translation.Data(1,3) -V3_translation.Data(1,1) 0]);
    waypoints = [-V3_translation.Data((1:step_length:end),3) -V3_translation.Data((1:step_length:end),1) V3_translation.Data((1:step_length:end),2)];
    speed = V3_speed.Data(1:step_length:end);
    trajectory(car3, waypoints, speed);
    
car4 = vehicle(scenario, ...
    'ClassID', 4, ...
    'Position', [-V4_translation.Data(1,3) -V4_translation.Data(1,1) 0]);
    waypoints = [-V4_translation.Data((1:step_length:end),3) -V4_translation.Data((1:step_length:end),1) V4_translation.Data((1:step_length:end),2)];
    speed = V4_speed.Data(1:step_length:end);
    trajectory(car4, waypoints, speed);
    
car5 = vehicle(scenario, ...
    'ClassID', 5, ...
    'Position', [-V5_translation.Data(1,3) -V5_translation.Data(1,1) 0]);
    waypoints = [-V5_translation.Data((1:step_length:end),3) -V5_translation.Data((1:step_length:end),1) V5_translation.Data((1:step_length:end),2)];
    speed = V5_speed.Data(1:step_length:end);
    trajectory(car5, waypoints, speed);
    
 car6 = vehicle(scenario, ...
    'ClassID', 6, ...
    'Position', [-V6_translation.Data(1,3) -V6_translation.Data(1,1) 0]);
    waypoints = [-V6_translation.Data((1:step_length:end),3) -V6_translation.Data((1:step_length:end),1) V6_translation.Data((1:step_length:end),2)];
    speed = V6_speed.Data(1:step_length:end);
    trajectory(car6, waypoints, speed);
    
 car7 = vehicle(scenario, ...
    'ClassID', 7, ...
    'Position', [-V7_translation.Data(1,3) -V7_translation.Data(1,1) 0]);
    waypoints = [-V7_translation.Data((1:step_length:end),3) -V7_translation.Data((1:step_length:end),1) V7_translation.Data((1:step_length:end),2)];
    speed = V7_speed.Data(1:step_length:end);
    trajectory(car7, waypoints, speed);
    
 car8 = vehicle(scenario, ...
    'ClassID', 8, ...
    'Position', [-V8_translation.Data(1,3) -V8_translation.Data(1,1) 0]);
    waypoints = [-V8_translation.Data((1:step_length:end),3) -V8_translation.Data((1:step_length:end),1) V8_translation.Data((1:step_length:end),2)];
    speed = V8_speed.Data(1:step_length:end);
    trajectory(car8, waypoints, speed);
    
car9 = vehicle(scenario, ...
    'ClassID', 9, ...
    'Position', [-V9_translation.Data(1,3) -V9_translation.Data(1,1) 0]);
    waypoints = [-V9_translation.Data((1:step_length:end),3) -V9_translation.Data((1:step_length:end),1) V9_translation.Data((1:step_length:end),2)];
    speed = V9_speed.Data(1:step_length:end);
    trajectory(car9, waypoints, speed);
    
car10 = vehicle(scenario, ...
    'ClassID', 10, ...
    'Position', [-V10_translation.Data(1,3) -V10_translation.Data(1,1) 0]);
    waypoints = [-V10_translation.Data((1:step_length:end),3) -V10_translation.Data((1:step_length:end),1) V10_translation.Data((1:step_length:end),2)];
    speed = V10_speed.Data(1:step_length:end);
    trajectory(car10, waypoints, speed);
% car3 = vehicle(scenario, ...
%     'ClassID', 3, ...
%     'Position', [-temp.V3_translation.Data(1,3) -temp.V3_translation.Data(1,1) 0]);
%     waypoints = [-temp.V3_translation.Data((1:step_length:end),3) -temp.V3_translation.Data((1:step_length:end),1) temp.V3_translation.Data((1:step_length:end),2)];
%     speed = temp.V3_speed.Data(1:step_length:end);
%     trajectory(car3, waypoints, speed);
% car4 = vehicle(scenario, ...
%     'ClassID', 4, ...
%     'Position', [-temp.V4_translation.Data(1,3) -temp.V4_translation.Data(1,1) 0]);
%     waypoints = [-temp.V4_translation.Data((1:step_length:end),3) -temp.V4_translation.Data((1:step_length:end),1) temp.V4_translation.Data((1:step_length:end),2)];
%     speed = temp.V4_speed.Data(1:step_length:end);
%     trajectory(car4, waypoints, speed);
% car5 = vehicle(scenario, ...
%     'ClassID', 5, ...
%     'Position', [-temp.V5_translation.Data(1,3) -temp.V5_translation.Data(1,1) 0]);
%     waypoints = [-temp.V5_translation.Data((1:step_length:end),3) -temp.V5_translation.Data((1:step_length:end),1) temp.V5_translation.Data((1:step_length:end),2)];
%     speed = temp.V5_speed.Data(1:step_length:end);
%     trajectory(car5, waypoints, speed);
% car6 = vehicle(scenario, ...
%     'ClassID', 6, ...
%     'Position', [-temp.V6_translation.Data(1,3) -temp.V6_translation.Data(1,1) 0]);
%     waypoints = [-temp.V6_translation.Data((1:step_length:end),3) -temp.V6_translation.Data((1:step_length:end),1) temp.V6_translation.Data((1:step_length:end),2)];
%     speed = temp.V6_speed.Data(1:step_length:end);
%     trajectory(car6, waypoints, speed);
% car7 = vehicle(scenario, ...
%     'ClassID', 7, ...
%     'Position', [-temp.V7_translation.Data(1,3) -temp.V7_translation.Data(1,1) 0]);
%     waypoints = [-temp.V7_translation.Data((1:step_length:end),3) -temp.V7_translation.Data((1:step_length:end),1) temp.V7_translation.Data((1:step_length:end),2)];
%     speed = temp.V7_speed.Data(1:step_length:end);
%     trajectory(car7, waypoints, speed);
% car8 = vehicle(scenario, ...
%     'ClassID', 8, ...
%     'Position', [-temp.V8_translation.Data(1,3) -temp.V8_translation.Data(1,1) 0]);
%     waypoints = [-temp.V8_translation.Data((1:step_length:end),3) -temp.V8_translation.Data((1:step_length:end),1) temp.V8_translation.Data((1:step_length:end),2)];
%     speed = temp.V8_speed.Data(1:step_length:end);
%     trajectory(car8, waypoints, speed);
% car9 = vehicle(scenario, ...
%     'ClassID', 9, ...
%     'Position', [-temp.V9_translation.Data(1,3) -temp.V9_translation.Data(1,1) 0]);
%     waypoints = [-temp.V9_translation.Data((1:step_length:end),3) -temp.V9_translation.Data((1:step_length:end),1) temp.V9_translation.Data((1:step_length:end),2)];
%     speed = temp.V9_speed.Data(1:step_length:end);
%     trajectory(car9, waypoints, speed);
% car10 = vehicle(scenario, ...
%     'ClassID', 10, ...
%     'Position', [-temp.V10_translation.Data(1,3) -temp.V10_translation.Data(1,1) 0]);
%     waypoints = [-temp.V10_translation.Data((1:step_length:end),3) -temp.V10_translation.Data((1:step_length:end),1) temp.V10_translation.Data((1:step_length:end),2)];
%     speed = temp.V10_speed.Data(1:step_length:end);
%     trajectory(car10, waypoints, speed);
