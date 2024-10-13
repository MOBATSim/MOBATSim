classdef Infrastructure < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.CustomIcon
    % This block initializes the vehicles on the map, draws and updates their positions on the 2D plot. When a collision happens, the MAP instance detects it and stops the colliding vehicles. Apart from that, it contains an AIM (Autonomous Intersection Manager) to avoid conflictions on intersections. The decisions are sent to the vehicles as GO/STOP.
    %
    
    % Contributors: Mustafa Saraoglu, Fabian Hart, Johannes Pintscher, Qianwei Yang
    
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

        function StopCommands = stepImpl(obj, V2Idata)
            % Signal output is the Go/Stop commands (Apart from the visual output or collision detection done without signals but accessing variables) 
            
            %% Autononomous Intersection Management (AIM) Units           
            AIM_Units = obj.map.AIM_Units;
            
            for i=1:size(V2Idata,1)
                
                intersectionID = V2Idata(i,1);  % ID of the intersection: 
                                                % 0 -> none
                                                % 1 -> 4x4 in west
                                                % 2 -> t-junction in east
                                                
                vehicleStateatIntersection = V2Idata(i,2); % Vehicle's state at the intersection
                                                % 0 -> "x" (not at any intersection)
                                                % 1 -> "d" (at decision node)
                                                % 2 -> "i" (at incoming node)
                                                % 3 -> "p" (at conflict zone)
                                                % 4 -> "o" (at outgoing node)
                
                %% Check if any vehicle at intersection from V2I Data
                if intersectionID ~= 0 % If the vehicle is at some intersection
                    vehicle = obj.Vehicles(i);
                    
                    % Vehicle is at the DECISION NODE (just arrived at the intersection)
                    if vehicleStateatIntersection == 1
                        AIM_Units(intersectionID).vehicleReachesDecisionNode(vehicle);
                        
                    % Vehicle is at the incoming node (has to GO or WAIT depending on the AIM's decision)
                    elseif vehicleStateatIntersection == 2
                        AIM_Units(intersectionID).vehicleReachesIncomingNode(vehicle, obj.Vehicles);
                        
                    % Vehicle is passing (in the conflict zone -> has to keep going)
                    elseif vehicleStateatIntersection == 3
                        % No change in states for the vehicle until it leaves the intersection
                          
                    % Vehicle is at the outgoing node (leaving the intersection)
                    elseif vehicleStateatIntersection == 4
                        AIM_Units(intersectionID).vehicleLeavesIntersection(vehicle, obj.Vehicles);
                    end
                    
                end
                
            end
            
            %% (Only for TrafficLight) Update conventional traffic systems
            for i=1:length(AIM_Units)
                if AIM_Units(i).mode == IM_Algorithm_Enum.TrafficLight
                    AIM_Units(i).updateConventionalTrafficLightSystem(obj.getCurrentTime);
                end
            end
            
            
            %% Get the braking flag arrays (stop commands) from all crossroad units (AIM algorithms)
            StopCommands = cat(1,obj.map.AIM_Units.vehicleOrders);
            
            %% If Stop command is given -> brake between the "incoming node" and the "conflict zone" at the intersection zone
            for i=size(StopCommands,1):-1:1
                if obj.Vehicles(StopCommands(i)).status.atIntersection(2) ~= "i"
                    % Delete when not in this zone from braking array
                    StopCommands(i,:) = [];
                end
            end
            
            %% This Infrastructure Block handles: 2D Traffic Plot + Path Dynamic Highlight
            if mod(obj.getCurrentTime,0.2) == 0
                obj.map.dynamicTrafficPlot(); % Update Vehicle locations and speeds
                
                if mod(obj.getCurrentTime,1) == 0
                    obj.map.dynamicRouteHighlighting(); % Update hightlighted paths
                end
            end
            
            %% This Infrastructure Block also handles: Collision detection
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
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon("MOBATSIM-Icon-Set_5- Intersection.png");
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
