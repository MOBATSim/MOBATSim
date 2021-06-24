function prepare_simulator(options)
    % This function prepares the simulation
    %   After calling this method the simulation can run
    arguments

        options.Analysing (1,1) logical = false
        options.FI_id (1,1) double = 1
        options.FI_value (1,1) double = 0
    end
    
    %% Init file for MOBATSim
    hold off
    warning off

    %% Added for Fast Debug /Needs to be removed later to make sure that simulations can be repeated without "clear all"
    if evalin('base','exist(''Map'',''var'')') 
        evalin('base','clear all'); %TODO: needs to be off in order not to delete the variables assigned from the GUI
        evalin('base','close all'); %to avoid some problems with the deleted handles TODO: Try -> close('all', 'hidden')
        MapType = MapTypes.GridMap;
    elseif ~evalin('base','exist(''MapType'',''var'')')
        MapType = MapTypes.GridMap;
    end


    %% MOBATSim Configurations
    modelName = 'MOBATSim';
    
    assignin('base','modelName',modelName);
    
    simSpeed = 1; % For scaling the simulation speed
    Sim_Ts = 0.02; % Sample time of the simulation (may not be stable if changed)
    Sim_t = 20; % set simulation time

    assignin('base','simSpeed',simSpeed);
    assignin('base','Sim_Ts',Sim_Ts);
    assignin('base','Sim_t',Sim_t);
    
    configs = MOBATSimConfigurations(modelName,simSpeed,Sim_Ts,MapType); % MapType: 'GridMap' or 'DigraphMap'


    %% GUI Scenario Config Defaults % TODO: change this part when GUI is changed, does not check base workspace
    if ~exist('mapName','var')
        mapName = 'Mobatkent'; % Default map selection
    end
    if ~exist('scenarioSelection','var')&&~exist('CustomScenarioGenerated','var')&&(~exist('RandomScenarioGenerated','var'))
        scenarioSelection = 'Urban City Traffic'; % Default scenario selection
    end

    %% Load the Map
    switch mapName  
        case 'Mobatkent'
            %[mapName, waypoints, connections_circle, connections_translation, ...
            %  startingNodes, breakingNodes, stoppingNodes, leavingNodes] = load_Mobatkent();
            [Route_LaneNumber, waypoints, connections_translation, connections_circle, ...
              startingNodes, breakingNodes, stoppingNodes, leavingNodes] = load_Mobatkent_from_opendrive();%load extended map
        case 'Highway'
            open_system('Platoon_Event')
            return

        case 'Crossmap'
            [Route_LaneNumber, waypoints, connections_circle, connections_translation, ...
              startingNodes, breakingNodes, stoppingNodes, leavingNodes] = load_Crossmap();             
    end

    %% Generate the 2D Map and the instance from the Map class
    if configs.MapType == MapTypes.GridMap
        Map = GridMap(mapName,waypoints, connections_circle,connections_translation, startingNodes, breakingNodes, stoppingNodes, leavingNodes,Route_LaneNumber);
    else
        Map = DigraphMap(mapName,waypoints, connections_circle,connections_translation, startingNodes, breakingNodes, stoppingNodes, leavingNodes,Route_LaneNumber);
    end
    
     assignin('base','Map',Map);

    %% Load Scenario and Vehicles
    if (~exist('CustomScenarioGenerated','var'))&&(~exist('RandomScenarioGenerated','var')) % TODO: change this part when GUI is changed, does not check base workspace
        [startingTimes, startingPoints, destinationPoints, maxSpeeds] = load_scenario(scenarioSelection); % default on - for Monte Carlo experiments comment out
    end
    %uncomment line below to undo
    % [startingTimes, startingPoints, destinationPoints, maxSpeeds] = load_scenario(scenarioSelection); % default on - for Monte Carlo experiments comment out

    
    %% TODO: Write here the Function to manipulate the initial maxSpeeds for experiments
    maxSpeeds = changeMaxSpeedofAVehicle(maxSpeeds,options.FI_id,options.FI_value);
    
    % Load Vehicles
    Vehicles = load_vehicles(startingPoints, destinationPoints, maxSpeeds, startingTimes, simSpeed); % default on - for Monte Carlo experiments comment out

    %MonteCarlo_scenarios(); % default off - for Monte Carlo experiments uncomment

    %% Initialize Vehicles on the Map
    Map.Vehicles = Vehicles;
    Map.initCarDescriptionPlot();

    if configs.MapType == MapTypes.GridMap
        %create BOG
        [Map.bogMap,Map.xOffset,Map.yOffset] = Map.generateBOGrid(Map);
    end
    
    % Open MOBATSim Simulink Model
    open_system(modelName)

    %% Fault Injection properties (TODO: To be implemented soon)
    FI_distance = 0;
    FI_speed = 0;
    SafeDistance =18;

    %% Assign all needed variables to base workspace TODO: check if they all needed in base workspace

    assignin('base','MapType',MapType);    
    assignin('base','configs',configs);
    assignin('base','scenarioSelection',scenarioSelection);
    assignin('base','mapName',mapName);
    assignin('base','Map',Map);
    assignin('base','Vehicles',Vehicles);  
    assignin('base','FI_distance',FI_distance);
    assignin('base','FI_speed',FI_speed);
    assignin('base','SafeDistance',SafeDistance);

    %% Initalize analysing
    % close vehicle analysing window
    close(findall(groot,'Type','figure','Tag','vehicleAnalysingWindow')); % close analysing window
    
    if options.Analysing
        vehicleAnalysingWindow_Gui = VehicleAnalysingWindow_Gui(Vehicles, 2);
    else
        vehicleAnalysingWindow_Gui = false;
    end
    
    assignin('base','vehicleAnalysingWindow_Gui',vehicleAnalysingWindow_Gui);
    
    %sim(modelName); % Uncomment this line for a single button execution

end

%% TODO: Move this function to the report generator folder
function maxSpeed = changeMaxSpeedofAVehicle(maxSpeed,vehicleid,FiVal)
% Change the corresponding vehicle's maxSpeed
  maxSpeed(vehicleid) = maxSpeed(vehicleid) + FiVal;
end