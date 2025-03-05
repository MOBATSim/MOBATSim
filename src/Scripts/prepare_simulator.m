function prepare_simulator(options)
%% Init file for MOBATSim    
    % This function prepares the simulation
    %   After calling this function the simulation can run
    
    % Contributors: Johannes Pintscher, Mustafa Saraoglu, Qianwei Yang, Erik Noack
    
    arguments
        options.modelName         (1,1) string    = 'MOBATSim'      % Name of the simulink model
        options.mapName           (1,1) string    = 'Mobatkent'     % Name of the map
        options.scenarioName      (1,1) string    = 'Scenario 1'    % Scenario sets start points, destination points and maxSpeeds
        options.simStopTime       (1,1) double    = 120             % Simulation stop time in seconds
        options.simTs             (1,1) double    = 0.02            % Simulation time step: sample time of the simulation (may not be stable if changed)
        options.IM_Mode           (1,1) IM_Algorithm_Enum = IM_Algorithm_Enum.IntelligentDecision % the mode in which the intersection management algorithm operates
        options.simpleMap         (1,1) logical   = false           % Displays the map without waypoint or route numbers
        options.startingPoints    (1,:) double    = []              % Custom starting points for vehicles
        options.destinationPoints (1,:) double    = []              % Custom destination points for vehicles
        options.maxSpeeds         (1,:) double    = []              % Custom max speeds for vehicles
        options.collisionReport   (1,1) double    = 0               % 0 = no report, 1= Detailed collision report, 2=Only intersection
        options.analysisWindow    (1,1) logical   = false           % Activates the vehicle analysis window

    end
    
    %% Keep the command window clear
    hold off
    warning off

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
    % Initial conditions
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
                                                        options.analysisWindow, ...
                                                        options.simpleMap, ...
                                                        options.IM_Mode);
        
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
        startingNodes, brakingNodes, stoppingNodes, leavingNodes] = load_MapSpecifications();

    %% Generate the 2D Map and the instance from the Map class
    Map = GridMap(options.mapName,waypoints, connections_circle,connections_translation, startingNodes, brakingNodes, stoppingNodes, leavingNodes,Route_LaneNumber,options.IM_Mode,"showLaneNumbers",~options.simpleMap,"showNodeNumbers",~options.simpleMap);

    %% Load Vehicles    
    Vehicles = load_vehicles(startingPoints, destinationPoints, maxSpeeds, Map, options.collisionReport);

    %% Initialize Vehicles on the Map
    Map.Vehicles = Vehicles;
    Map.initCarDescriptionPlot();

    %% Create Binary Occupancy Grid Map
    [Map.bogMap,Map.xOffset,Map.yOffset] = Map.generateBOGrid(Map);
    
    %% Check selected starting and destination points are valid
    forbiddenNodes = [brakingNodes; stoppingNodes]; % vehicles should not stop in front or in the intersection    
    occupiedNodes = [startingPoints, destinationPoints];
    
    invalidNodes = [];
    for i=1:numel(occupiedNodes)
        % check if an occupied node is actually a forbidden node
        if any(occupiedNodes(i) == forbiddenNodes,'all')
            invalidNodes(end+1) = occupiedNodes(i); %#ok<AGROW>
        end
    end
    
    if ~isempty(invalidNodes)
        % Construct an exception for wrong waypoints
        errID = 'PREPARATION:NodesOccupied';
        msg = "Vehicles starting or destination waypoint is on forbidden nodes: " + strjoin(string(invalidNodes));
        baseException = MException(errID,msg);
        throw(baseException);
    end
    
    %% Open the MOBATSim Simulink Model
    open_system(options.modelName)
    
    %% Activative the Vehicle Analysis Window (optional)
    % Close Vehicle Analysis Window
    close(findall(groot,'Type','figure','Tag','vehicleAnalysingWindow'));
    
    %% Initalize Analysis Window
    if options.analysisWindow
        vehiclePredictor = VehiclePredictor(Vehicles, 2); % part for all calculations and stuff shown on vehicle analysing window
        vehicleAnalysingWindow_Gui = VehicleAnalysingWindow_Gui(vehiclePredictor);
        assignin('base','vehiclePredictor',vehiclePredictor);
    else
        vehicleAnalysingWindow_Gui = false;
    end   
      
    %% Save configuration
    configs = MOBATSimConfigurations(options.modelName, ...
                                     options.mapName, ...
                                     options.simStopTime, ...
                                     options.simTs, ...                                     
                                     options.scenarioName, ...
                                     startingPoints, ...
                                     destinationPoints, ...
                                     maxSpeeds, ...
                                     options.analysisWindow, ...
                                     options.simpleMap, ...
                                     options.IM_Mode);
        

    %% Assign all needed workspace variables to the "base" workspace
    assignin('base','Sim_Ts',options.simTs);        % Used as the block sampling time
    assignin('base','Sim_t',options.simStopTime);   % Used by the model as the Simulation Time
    assignin('base','configs',configs);             % MOBATSim configurations
    assignin('base','Map',Map);                     % Used by the Infrastructure.m
    assignin('base','Vehicles',Vehicles);           % The instances are used by the MATLAB System Blocks
    assignin('base','vehicleAnalysingWindow_Gui',vehicleAnalysingWindow_Gui);
    
end