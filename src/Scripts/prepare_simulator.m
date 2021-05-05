%% Init file for MOBATSim
hold off
warning off

%% Added for Fast Debug /Needs to be removed later to make sure that simulations can be repeated without "clear all"
if exist('Map','var') 
    clear all; %TODO: needs to be off in order not to delete the variables assigned from the GUI
    close all; %to avoid some problems with the deleted handles TODO: Try -> close('all', 'hidden')
    MapType = MapTypes.GridMap;
elseif ~exist('MapType','var')
    MapType = MapTypes.GridMap;
end


%% MOBATSim Configurations
modelName = 'MOBATSim';

simSpeed = 1; % For scaling the simulation speed
Sim_Ts = 0.02; % Sample time of the simulation (may not be stable if changed)

configs = MOBATSimConfigurations(modelName,simSpeed,Sim_Ts,MapType); % MapType: 'GridMap' or 'DigraphMap'


%% GUI Scenario Config Defaults
if ~exist('mapSelection','var')
    mapSelection = 'Mobatkent'; % Default map selection
end
if ~exist('scenarioSelection','var')&&~exist('CustomScenarioGenerated','var')&&(~exist('RandomScenarioGenerated','var'))
    scenarioSelection = 'Urban City Traffic'; % Default scenario selection
end

%% Load the Map
switch mapSelection  
    case 'Mobatkent'
        %load_Mobatkent();
        load_Mobatkent_from_opendrive();%load extended map
    case 'Highway'
        open_system('Platoon_Event')
        return
    
    case 'Crossmap'
        load_Crossmap();             
end

%% Generate the 2D Map and the instance from the Map class
if configs.MapType == MapTypes.GridMap
    Map = GridMap(mapName,waypoints, connections_circle,connections_translation, startingNodes, breakingNodes, stoppingNodes, leavingNodes,Route_LaneNumber);
else
    Map = DigraphMap(mapName,waypoints, connections_circle,connections_translation, startingNodes, breakingNodes, stoppingNodes, leavingNodes,Route_LaneNumber);
end

%% Load Scenario and Vehicles
if (~exist('CustomScenarioGenerated','var'))&&(~exist('RandomScenarioGenerated','var')) % new
load_scenario(scenarioSelection); % default on - for Monte Carlo experiments comment out
end
%uncomment line below to undo
%load_scenario(scenarioSelection); % default on - for Monte Carlo experiments comment out

% Load Vehicles
load_vehicles(); % default on - for Monte Carlo experiments comment out

%MonteCarlo_scenarios(); % default off - for Monte Carlo experiments uncomment

%% Initialize Vehicles on the Map
Map.Vehicles = Vehicles;
Map.initCarDescriptionPlot();

if configs.MapType == MapTypes.GridMap
    %create BOG
    [Map.bogMap,Map.xOffset,Map.yOffset] = Map.generateBOGrid(Map);
end
% Clear the initializing variables
clear_init_variables();

% Open MOBATSim Simulink Model
open_system(modelName)

% generate vehicle analysing window TODO JP: maybe move, only here for
% testing
VehicleAnalysingWindow = VehicleAnalysingWindow([Vehicle4 Vehicle9], 9);

%% Fault Injection properties (TODO: To be implemented soon)
FI_distance = 0;
FI_speed = 0;
SafeDistance =18;

%sim(modelName); % Uncomment this line for a single button execution