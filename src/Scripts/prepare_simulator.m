function prepare_simulator(options)
    % This function prepares the simulation
    %   After calling this method the simulation can run
    arguments
        options.Analysing   (1,1) logical   = false                 % activate the analysing functions
        options.modelName   (1,1) string    = 'MOBATSim'            % name of the simulink model
        options.simStopTime (1,1) double    = 80                    % simulation stop time in seconds
        options.simTs       (1,1) double    = 0.02                  % simulation time step: sample time of the simulation (may not be stable if changed)
        options.scenarioName(1,1) string    = 'Urban City Traffic'
        options.FI_id       (1,1) double = 1
        options.FI_value    (1,1) double = 0
        options.FI_delay    (1,1) double = 0
        options.FI_failure  (1,1) double = 0
    end
    
    %% Init file for MOBATSim
    hold off
    warning off

    %% Added for Fast Debug /Needs to be removed later to make sure that simulations can be repeated without "clear all"
    if evalin('base','exist(''Map'',''var'')') 
        evalin('base','clear all'); %TODO: needs to be off in order not to delete the variables assigned from the GUI
        evalin('base','close all'); %to avoid some problems with the deleted handles TODO: Try -> close('all', 'hidden')
    end
    

    %% MOBATSim Configurations  
      
    configs = MOBATSimConfigurations(options.modelName, ...
                                     options.simStopTime, ...
                                     options.simTs, ...
                                     options.mapName, ...
                                     options.scenarioName);


    %% Load the Map
    %[options.mapName, waypoints, connections_circle, connections_translation, ...
    %  startingNodes, brakingNodes, stoppingNodes, leavingNodes] = load_Mobatkent();
    [Route_LaneNumber, waypoints, connections_translation, connections_circle, ...
        startingNodes, brakingNodes, stoppingNodes, leavingNodes] = load_Mobatkent_from_opendrive();%load extended map

    %% Generate the 2D Map and the instance from the Map class
    Map = GridMap(options.mapName,waypoints, connections_circle,connections_translation, startingNodes, brakingNodes, stoppingNodes, leavingNodes,Route_LaneNumber);

    %% Load Scenario and Vehicles
    if (~exist('CustomScenarioGenerated','var'))&&(~exist('RandomScenarioGenerated','var')) % TODO: change this part when GUI is changed, does not check base workspace
        [startingPoints, destinationPoints, maxSpeeds] = load_scenario(options.scenarioName); % default on - for Monte Carlo experiments comment out
    end
    %uncomment line below to undo
    % [startingPoints, destinationPoints, maxSpeeds] = load_scenario(options.scenarioName); % default on - for Monte Carlo experiments comment out

    
    %% (Will be moved from here) Fault Injection Parameters
    maxSpeeds = changeMaxSpeedofAVehicle(maxSpeeds,options.FI_id,options.FI_value);
    
    delayTimeV3 = options.FI_delay; % delay time of driving mode change for V3 
    V3_FailureRate = options.FI_failure; % failure rate of the sensor of V3 
    
    %% Load Vehicles
    
    Vehicles = load_vehicles(startingPoints, destinationPoints, maxSpeeds, Map); % default on - for Monte Carlo experiments comment out

    %MonteCarlo_scenarios(); % default off - for Monte Carlo experiments uncomment

    %% Initialize Vehicles on the Map
    Map.Vehicles = Vehicles;
    Map.initCarDescriptionPlot();

    
    %create BOG
    [Map.bogMap,Map.xOffset,Map.yOffset] = Map.generateBOGrid(Map);

    
    % Open MOBATSim Simulink Model
    open_system(options.modelName)

    %% Fault Injection properties (TODO: To be implemented soon)
    FI_distance = 0;
    FI_speed = 0;
    SafeDistance =18;

    %% Initalize analysing
    
    % close vehicle analysing window
    close(findall(groot,'Type','figure','Tag','vehicleAnalysingWindow')); % close analysing window
    
    % generate analysing classes
    if options.Analysing
        vehiclePredictor = VehiclePredictor(Vehicles, 2); % part for all calculations and stuff shown on vehicle analysing window
        vehicleAnalysingWindow_Gui = VehicleAnalysingWindow_Gui(vehiclePredictor);
        assignin('base','vehiclePredictor',vehiclePredictor);
    else
        vehicleAnalysingWindow_Gui = false;
    end   
    
    
    %% Assign all needed variables to base workspace TODO: check if they all needed in base workspace
    
    assignin('base','Sim_Ts',options.simTs); % used by the model and in VehicleKinematics and V_WPGenerator_PurePursuit
    assignin('base','Sim_t',options.simStopTime); % used by Infrastructure for a test and in the model as StopTime
    assignin('base','configs',configs);
    assignin('base','Map',Map); % only used by Infrastructure.m
    assignin('base','Vehicles',Vehicles); % used by many instances
    assignin('base','vehicleAnalysingWindow_Gui',vehicleAnalysingWindow_Gui);
    assignin('base','FI_distance',FI_distance);
    assignin('base','FI_speed',FI_speed);
    assignin('base','SafeDistance',SafeDistance);
    assignin('base','delayTimeV3',delayTimeV3);
    assignin('base','V3_FailureRate',V3_FailureRate);
    
    %% Single button execution
    
    %sim(options.modelName); % Uncomment this line for a single button execution
    
end

%% TODO: Move this function to the report generator folder
function maxSpeed = changeMaxSpeedofAVehicle(maxSpeed,vehicleid,FiVal)
% Change the corresponding vehicle's maxSpeed
  maxSpeed(vehicleid) = maxSpeed(vehicleid) + FiVal;
end