%% Init file for MOBATSim
hold off

if exist('Map','var') 
clear all; %TODO: needs to be off in order not to delete the variables assigned from the GUI
close all; %to avoid some problems with the deleted handles
end

warning off

%%
modelName = 'MOBATSim';

simSpeed = 4; % For scaling the simulation speed
Sim_Ts = 0.01; % Sample time of the simulation (may not be stable if changed)

if ~exist('mapSelection','var')
    mapSelection = 'Mobatkent'; % Default map selection
end
%% new
if ~exist('scenarioSelection','var')&&~exist('CustomScenarioGenerated','var')&&(~exist('RandomScenarioGenerated','var'))
    scenarioSelection = 'Urban City Traffic'; % Default scenario selection
end
%% Replacement
% if ~exist('scenarioSelection','var')
%     scenarioSelection = 'Urban City Traffic'; % Default scenario selection
% end

%% Load the Map
switch mapSelection  
    case 'Mobatkent'
        load_Mobatkent();
    
    case 'Highway'
        open_system('Platoon_Event')
        return
    
    case 'Crossmap'
        load_Crossmap();             
end

% Generate the 2D Map and the instance from the Map class


% Map = GridMap(mapName,waypoints, connections_circle,connections_translation, startingNodes, breakingNodes, stoppingNodes, leavingNodes);

Map2 = GridMapPed(mapName,waypoints, connections_circle,connections_translation, startingNodes, breakingNodes, stoppingNodes, leavingNodes);

Map = Map(mapName,waypoints, connections_circle,connections_translation, startingNodes, breakingNodes, stoppingNodes, leavingNodes);


%% Load Scenario and Vehicles
if (~exist('CustomScenarioGenerated','var'))&&(~exist('RandomScenarioGenerated','var')) % new
load_scenario(scenarioSelection); % default on - for Monte Carlo experiments comment out
end
%uncomment line below to undo
%load_scenario(scenarioSelection); % default on - for Monte Carlo experiments comment out

% Load Vehicles
load_vehicles(); % default on - for Monte Carlo experiments comment out

%MonteCarlo_scenarios(); % default off - for Monte Carlo experiments uncomment
figure(Map.MapFig)

%% Initialize Vehicles on the Map
Map.Vehicles = Vehicles;
Map.initCarDescriptionPlot();

% Clear the initializing variables
clear_init_variables();

% Open MOBATSim Simulink Model
open_system(modelName)

%% Fault Injection properties (TODO: To be implemented soon)
FI_distance = 0;
FI_speed = 0;
SafeDistance =18;

%sim(modelName); % Uncomment this line for a single button execution

createPedestrians();