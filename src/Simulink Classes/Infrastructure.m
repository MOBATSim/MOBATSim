classdef Infrastructure < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.CustomIcon
    % This block initializes the vehicles on the map, draws and updates their positions on the 2D plot. When a collision happens, the MAP instance detects it and stops the colliding vehicles. Apart from that, it contains an AIM (Autonomous Intersection Manager) to avoid conflictions on intersections. The decisions are sent to the vehicles as stop, wait, and go.
    %

    % These properties are nontunable. When you use the System object™, 
    % you can only change nontunable properties before calling the object 
    % or after calling the release function.
    properties (Nontunable)
        % This properties are needed to set the output size of this block
               
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
            
            %% Crossroad units
            
            for i=1:size(V2Idata,1)
                if V2Idata(i,1) ~= 0
                    vehicle = obj.Vehicles(i);
                    if V2Idata(i,2) == 1
                        % Car reaches the crossroad
                        obj.map.crossroadUnits(V2Idata(i,1)).carReachesCrossroad(vehicle, vehicle.pathInfo.lastWaypoint);
                    elseif V2Idata(i,2) == 2
                        % Car reaches the braking point
                        obj.map.crossroadUnits(V2Idata(i,1)).carReachesBrakingPoint(vehicle, obj.Vehicles, vehicle.pathInfo.lastWaypoint, obj.getCurrentTime);
                    elseif V2Idata(i,2) == 3
                        
                    elseif V2Idata(i,2) == 4
                        % Car leaves the crossroad
                        obj.map.crossroadUnits(V2Idata(i,1)).carLeavesCrossroad(vehicle, obj.Vehicles, obj.getCurrentTime)
                    end
                end
            end
            
            % get the braking flag arrays from all crossroad units
            mergedBrakingFlagArrays = cat(1,obj.map.crossroadUnits.brakingFlagArray);

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
        function mergedBrakingFlagArrays = isOutputFixedSizeImpl(~)
            %Both outputs are always variable-sized
            mergedBrakingFlagArrays = false;
            
        end
    end
    
    methods(Static,Access = protected)
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
        
        
        function mergedBrakingFlagArrays = getOutputSizeImpl(obj)
            % Maximum length of the output
            mergedBrakingFlagArrays = [obj.nrVehicles 2];
            
        end
        
        function mergedBrakingFlagArrays = getOutputDataTypeImpl(~)
            mergedBrakingFlagArrays = 'double'; %Linear indices are always double values
            
        end
        
        function mergedBrakingFlagArrays = isOutputComplexImpl(~)
            mergedBrakingFlagArrays = false; %Linear indices are always real values
           
        end
    end
end
