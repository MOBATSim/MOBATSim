function prepare_simulator(options)
%% Init file for MOBATSim    
    % This function prepares the simulation
    %   After calling this function the simulation can run
    
    % Contributors: Johannes Pintscher, Mustafa Saraoglu, Qianwei Yang, Erik Noack
    
    arguments
        options.Analysing           (1,1) logical   = false                 % Activate the analysing functions
        options.modelName           (1,1) string    = 'MOBATSim'            % Name of the simulink model
        options.mapName             (1,1) string    = 'Mobatkent'           % Name of the map
        options.simStopTime         (1,1) double    = 80                    % Simulation stop time in seconds
        options.simTs               (1,1) double    = 0.02                  % Simulation time step: sample time of the simulation (may not be stable if changed)
        options.scenarioName        (1,1) string    = 'Urban City Traffic'  % Scenario sets start points, destination points and maxSpeeds
        options.startingPoints      (1,:) double    = []                    % Custom starting points for vehicles
        options.destinationPoints   (1,:) double    = []                    % Custom destination points for vehicles
        options.maxSpeeds           (1,:) double    = []                    % Custom max speeds for vehicles
        options.simpleMap           (1,1) logical   = false                 % Displays the map in a simpler way
    end
       
    hold off
    warning off
    mpcverbosity off;
    %% Load Scenario
    [startingPoints, destinationPoints, maxSpeeds] = load_scenario(options.scenarioName);
    
    % Check for custom starting options
    if ~isempty(options.startingPoints)
        % Replace starting points with custom
        startingPoints = options.startingPoints;
    end
    if ~isempty(options.destinationPoints)
        % Replace destination points with custom
        destinationPoints = options.destinationPoints;
    end
    if ~isempty(options.maxSpeeds)
        % Replace max speeds with custom
        maxSpeeds = options.maxSpeeds;
    end
    

    %% Check if preparation can be skipped

    % initial conditions
    simNotStarted = isempty(evalin('base','who(''-regexp'', ''V[0-9]*_'')'));
    configExists = evalin('base','exist(''configs'',''var'')');
    
    if simNotStarted && configExists

        oldConfig = evalin('base','configs');
        differences = oldConfig.compareConfigurations(options.modelName, ...
                                                        options.mapName, ...
                                                        options.simStopTime, ...
                                                        options.simTs, ...                                     
                                                        options.scenarioName, ...
                                                        startingPoints, ...
                                                        destinationPoints, ...
                                                        maxSpeeds, ...
                                                        options.Analysing, ...
                                                        options.simpleMap);
        
        % if there are no differences, preparation is done                                        
        if ~any(differences)
            return
        end
    end
    
    %% Clear all data and release maps including invisible handles to make sure that the simulations can be repeated
    
    if evalin('base','exist(''Map'',''var'')')
        evalin('base','clear all');
        evalin('base','close all');
    end

    %% Load the Map
    [Route_LaneNumber, waypoints, connections_translation, connections_circle, ...
        startingNodes, brakingNodes, stoppingNodes, leavingNodes] = load_Mobatkent_from_file();%load extended map
        
    %% Generate the 2D Map and the instance from the Map class
    Map = GridMap(options.mapName,waypoints, connections_circle,connections_translation, startingNodes, brakingNodes, stoppingNodes, leavingNodes,Route_LaneNumber,"showLaneNumbers",~options.simpleMap,"showNodeNumbers",~options.simpleMap);

    %% Load Vehicles    
    Vehicles = load_vehicles(startingPoints, destinationPoints, maxSpeeds, Map);

    %% Initialize Vehicles on the Map
    Map.Vehicles = Vehicles;
    Map.initCarDescriptionPlot();

    % Create Binary Occupancy Grid Map
    [Map.bogMap,Map.xOffset,Map.yOffset] = Map.generateBOGrid(Map);
    
    % Check selected starting and destination points are valid
    forbiddenNodes = [brakingNodes; stoppingNodes]; % vehicles should not stop in front or on crossroad
    invalidNodes = checkNodesValid([startingPoints, destinationPoints], forbiddenNodes);
    if ~isempty(invalidNodes)
        % Construct an exception for wrong waypoints
        errID = 'PREPARATION:NodesOccupied';
        msg = "Vehicles starting waypoint or destination waypoint is on forbidden nodes: " + strjoin(string(invalidNodes));
        baseException = MException(errID,msg);
        throw(baseException);
    end
    
    % Open the MOBATSim Simulink Model
    open_system(options.modelName)
    
    %% Initalize Analysis Window
    % Close Vehicle Analysis Window
    close(findall(groot,'Type','figure','Tag','vehicleAnalysingWindow'));
    
    % Generate Analysing Classes
    if options.Analysing
        vehiclePredictor = VehiclePredictor(Vehicles, 2); % part for all calculations and stuff shown on vehicle analysing window
        vehicleAnalysingWindow_Gui = VehicleAnalysingWindow_Gui(vehiclePredictor);
        assignin('base','vehiclePredictor',vehiclePredictor);
    else
        vehicleAnalysingWindow_Gui = false;
    end   
      
    %% Make a MOBATSim Configuration
    configs = MOBATSimConfigurations(options.modelName, ...
                                     options.mapName, ...
                                     options.simStopTime, ...
                                     options.simTs, ...                                     
                                     options.scenarioName, ...
                                     startingPoints, ...
                                     destinationPoints, ...
                                     maxSpeeds, ...
                                     options.Analysing, ...
                                     options.simpleMap);
        

    %% Assign all needed workspace variables to the "base" workspace
    assignin('base','Sim_Ts',options.simTs);        % Used as the block sampling time
    assignin('base','Sim_t',options.simStopTime);   % Used by the model as the Simulation Time
    assignin('base','configs',configs);             % MOBATSim configurations
    assignin('base','Map',Map);                     % Used by the Infrastructure.m
    assignin('base','Vehicles',Vehicles);           % The instances are used by the MATLAB System Blocks
    assignin('base','vehicleAnalysingWindow_Gui',vehicleAnalysingWindow_Gui);
    
end