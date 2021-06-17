classdef Infrastructure < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.CustomIcon
    % This block initializes the vehicles on the map, draws and updates their positions on the 2D plot. When a collision happens, the MAP instance detects it and stops the colliding vehicles. Apart from that, it contains an AIM (Autonomous Intersection Manager) to avoid conflictions on intersections. The decisions are sent to the vehicles as stop, wait, and go.
    %

    % Public, tunable properties
    properties(Access = private)
        map = evalin('base','Map');
        Vehicles = evalin('base','Vehicles'); % prepare for logging test data
        Sim_t = evalin('base','Sim_t')% simulation time
    end
    
    
    % Pre-computed constants
    properties(Access = private)
        vehicleAnalysingWindow = evalin('base','vehicleAnalysingWindow_Gui');
        allTestData 
    end
    
    methods(Access = protected)

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon("logo_small.png");
        end
        
        function setupImpl(~)
            % Perform one-time calculations, such as computing constants
        end
       
        
        function mergedBrakingFlagArrays = stepImpl(obj, V2Idata)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            for i=1:size(V2Idata,1)
                if ~(V2Idata(i,1) == 0)
                    vehicle = obj.map.Vehicles(i);
                    if V2Idata(i,2) == 1
                        % Car reaches the crossroad
                        obj.map.crossroadUnits(V2Idata(i,1)).carReachesCrossroad(vehicle, vehicle.pathInfo.lastWaypoint);
                    elseif V2Idata(i,2) == 2
                        % Car reaches the braking point
                        obj.map.crossroadUnits(V2Idata(i,1)).carReachesBreakingPoint(vehicle, vehicle.pathInfo.lastWaypoint, obj.getCurrentTime);
                    elseif V2Idata(i,2) == 3
                        
                    elseif V2Idata(i,2) == 4
                        % Car leaves the crossroad
                        obj.map.crossroadUnits(V2Idata(i,1)).carLeavesCrossroad(vehicle, obj.getCurrentTime)
                    end
                end
            end
            
            mergedBrakingFlagArrays = []; % Memory preallocation is not feasible since the assignment size is variable
            for i=1:length(obj.map.crossroadUnits)
                mergedBrakingFlagArrays =  [mergedBrakingFlagArrays; obj.map.crossroadUnits(i).breakingFlagArray]; %#ok<AGROW>
            end
            

            %% 2D Traffic Plot
            obj.map.dynamicTrafficPlot();
            
            %% Path Dynamic Highlight
            if mod(obj.getCurrentTime,0.2) == 0
                obj.map.dynamicRouteHighlighting();
            end
            %% Collision detection
            if obj.getCurrentTime>0.1 % If you check collision right away, all vehicles collide at time 0.0
                allVehiclePositions = cat(1,cat(1,cat(1,obj.map.Vehicles).dynamics).position);
                allVehicleDistancesMatrix = dist(allVehiclePositions'); % Distance between all vehicles
                [row,column] = find(triu(allVehicleDistancesMatrix<20,1)); % The value 20 is the threshold for checking for collision
                if ~isempty(column) 
                    for k = 1:length(column) % If there are vehicle closer than 20 meters, check the collision for vehicle column(k) to row(k)
                        obj.map.Vehicles(column(k)).checkCollision(obj.map.Vehicles(row(k)));
                    end
                end
            end
            %% Vehicle Analysing Window TODO: check if should be called here
            if obj.vehicleAnalysingWindow ~= false % call only if window is generated
                obj.vehicleAnalysingWindow.update(obj.getCurrentTime);
            end
         %% Test data for evaluation
            enableLogData = true; % if it is able to log test data
            if enableLogData
                 TestData = obj.logTestData(); 
                 obj.allTestData = cat(3, obj.allTestData, TestData); % all test data in 3 dimension array
                 if obj.getCurrentTime >= obj.Sim_t % the last loop of simulation, "Sim_t" can be changed in "prepare_simulator.m"
                 assignin('base', 'allTestData', obj.allTestData); % all test data in workspace
                 %assignin('base', 'TestData', TestData); % only test data at stop time
                 end
            end
        end
        %% load test data for safety evaluation
         function TestData = logTestData(obj)
             TestData = zeros(7, length(obj.Vehicles));
             for i = 1:length(obj.Vehicles)
                 TestData(1, i) = obj.Vehicles(1,i).dynamics.speed;
                 TestData(2, i) = obj.Vehicles(1,i).dynamics.minDeceleration;
                 TestData(3, i) = obj.Vehicles(1,i).dynamics.position(1);
                 TestData(4, i) = obj.Vehicles(1,i).dynamics.position(3);
                 TestData(5, i) = obj.Vehicles(1,i).sensors.AEBdistance;
                 TestData(6, i) = obj.Vehicles(1,i).sensors.frontSensorRange;
                 TestData(7, i) = obj.Vehicles(1,i).sensors.ttc;
             end
         end
  %%      
        % These have to be specified because of
        % matlab.system.mixin.Propagates, to have variable size of outputs
        % and to avoid code generation which does not support the digraph
        function mergedBrakingFlagArrays = isOutputFixedSizeImpl(~)
            %Both outputs are always variable-sized
            mergedBrakingFlagArrays = false;
            
        end
    end
    
    methods(Static,Access = protected)
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
        
        
        function mergedBrakingFlagArrays = getOutputSizeImpl(~)
            % Maximum length of the output
            mergedBrakingFlagArrays = [10 20];
            
        end
        
        function mergedBrakingFlagArrays = getOutputDataTypeImpl(~)
            mergedBrakingFlagArrays = 'double'; %Linear indices are always double values
            
        end
        
        function mergedBrakingFlagArrays = isOutputComplexImpl(~)
            mergedBrakingFlagArrays = false; %Linear indices are always real values
           
        end
    end
end
