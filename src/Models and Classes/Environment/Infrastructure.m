classdef Infrastructure < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.CustomIcon
    % This block initializes the vehicles on the map, draws and updates their positions on the 2D plot. When a collision happens, the MAP instance detects it and stops the colliding vehicles. Apart from that, it contains an AIM (Autonomous Intersection Manager) to avoid conflictions on intersections. The decisions are sent to the vehicles as stop, wait, and go.
    %
    
    properties (Nontunable)
        % This property is needed to set the output size of this block       
        nrVehicles % Number of vehicles
    end
    
    % Public, tunable properties
    properties(Access = private)
        map = evalin('base','Map');
        Vehicles = evalin('base','Vehicles'); % prepare for logging test data
    end
    
    
    % Pre-computed constants
    properties(Access = private)
        vehicleAnalysingWindow = evalin('base','vehicleAnalysingWindow_Gui');
    end
    
    methods(Access = protected)
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon("MOBATSIM-Icon-Set_5- Intersection.png");
        end      
        
        function StopCommands = stepImpl(obj, V2Idata)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            %% Crossroad units           
            crossroadUnits = obj.map.crossroadUnits;
            
            for i=1:size(V2Idata,1)
                
                crossroadNr = V2Idata(i,1);     % number of crossroad the vehicle is passing
                crossroadPart = V2Idata(i,2);   % part of the crossroad the vehicle is passing
                
                if crossroadNr ~= 0
                    vehicle = obj.Vehicles(i);
                    if crossroadPart == 1
                        % Car reaches the crossroad
                        crossroadUnits(crossroadNr).carReachesCrossroad(vehicle);
                    elseif crossroadPart == 2
                        % Car reaches the braking point
                        crossroadUnits(crossroadNr).carReachesBrakingPoint(vehicle, obj.Vehicles);
                    elseif crossroadPart == 3
                        % Car reaches the start of the crossroad
                        crossroadUnits(crossroadNr).carReachesStartingPoint(vehicle);
                    elseif crossroadPart == 4
                        % Car leaves the crossroad
                        crossroadUnits(crossroadNr).carLeavesCrossroad(vehicle, obj.Vehicles);
                    end
                end
            end
            
            % Update conventional traffic systems
            for i=1:length(crossroadUnits)
                if crossroadUnits(i).mode == CrossroadModeEnum.TrafficLight
                    crossroadUnits(i).updateConventionalTrafficLightSystem(obj.getCurrentTime);
                end
            end
            
            
            % get the braking flag arrays from all crossroad units
            StopCommands = cat(1,obj.map.crossroadUnits.vehicleOrders);
            
            % Only brake when in crossroad zone 2 (between braking and stopping point)
            for i=size(StopCommands,1):-1:1
                if obj.Vehicles(StopCommands(i)).status.inCrossroad(2) ~= 2
                    % Delete when not in this zone from braking array
                    StopCommands(i,:) = [];
                end
            end
            
            %% 2D Traffic Plot + Path Dynamic Highlight
            if mod(obj.getCurrentTime,0.2) == 0
                obj.map.dynamicTrafficPlot(); % Update Vehicle locations and speeds
                
                if mod(obj.getCurrentTime,1) == 0
                    obj.map.dynamicRouteHighlighting(); % Update hightlighted paths
                end
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
            %% Vehicle Analysing Window
            if obj.vehicleAnalysingWindow ~= false % call only if window is generated
                obj.vehicleAnalysingWindow.update(obj.getCurrentTime);
            end
        end
        
        %%
        % These have to be specified because of
        % matlab.system.mixin.Propagates, to have variable size of outputs
        % and to avoid code generation which does not support the digraph
        function StopCommands = isOutputFixedSizeImpl(~)
            %Both outputs are always variable-sized
            StopCommands = false;
            
        end
    end
    
    methods(Static,Access = protected)
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
        
        
        function StopCommands = getOutputSizeImpl(obj)
            % Maximum length of the output
            StopCommands = [obj.nrVehicles 2];
            
        end
        
        function StopCommands = getOutputDataTypeImpl(~)
            StopCommands = 'double'; %Linear indices are always double values
            
        end
        
        function StopCommands = isOutputComplexImpl(~)
            StopCommands = false; %Linear indices are always real values
            
        end
    end
end
