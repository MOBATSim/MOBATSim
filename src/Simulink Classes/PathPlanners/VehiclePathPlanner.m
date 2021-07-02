classdef VehiclePathPlanner < matlab.System & handle & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
    % This Path Planner Block uses A* algorithm to find routes to reach the destination node according to the shared data from the other vehicles.

    % Public, tunable properties
    properties
        Vehicle_id
    end
    
    % Pre-computed constants
    properties(Access = protected)
        vehicle
        Map
        accelerationPhase;
        futureData
        breakingFlag
        inCrossroad % [crossroadId crossroadZone]
        % crossroadZone:
        % 1 -> arrivingZone
        % 2 -> stoppingZone
        % 3 -> intersectionZone
        
        %variables for visualization
        pathPlot;
    end
    
    methods
        % Constructor
        function obj = VehiclePathPlanner(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.vehicle = evalin('base', "Vehicles(" + obj.Vehicle_id + ")");
            obj.Map = obj.vehicle.map;
            
            obj.accelerationPhase =  zeros(1,5);
            obj.breakingFlag = 0;
            obj.inCrossroad = [0 0];
        end
        
        function waypointReached = stepImpl(obj,CommunicationIDs)
            %% Check if destination is reached
            if obj.vehicle.checkifDestinationReached() % If true vehicle stops
                FuturePlan = obj.vehicle.decisionUnit.futureData;   %Output 1: Future plan of the vehicle
                waypointReached=0;                                  %Output 2: Waypoint Reached enabler
            else            
                %% Check if the vehicle has reached a waypoint / Then it should reupdate its plan
                if obj.vehicle.pathInfo.calculateNewPathFlag == 1 
                    previousPath = obj.vehicle.pathInfo.path;
                    OtherVehiclesFutureData = obj.CollectFutureData(obj.Map.Vehicles, CommunicationIDs);
                    
                    obj.vehicle.logWaypointArrivalTimeStamps(obj.getCurrentTime); % Log Time Stamps

                    obj.vehicle.setStopStatus(false); % Vehicle continues to move/ Stop Status -> set to false
                    %% OtherVehiclesFutureData Processing
                    OtherVehiclesFutureData = obj.checkEmptyFutureData(OtherVehiclesFutureData); % Replace empty by zeros of nx6
                    OtherVehiclesFutureData = obj.deleteCollidedVehicleFutureData(OtherVehiclesFutureData); % Delete collided Vehicles' Future Data
                    
                    %% This is an abstract method that is implemented separately in each PathPlanner subclass
                    % Build the future plan by deriving the next routes and building the path
                    FuturePlan = obj.findPath(OtherVehiclesFutureData); %Output 1: Future plan of the vehicle
                    waypointReached =1;                                 %Output 2: Waypoint Reached enabler
                    
                    if ~isequal(previousPath,obj.vehicle.pathInfo.path) % If the vehicle changed it's planned path
                        currentTrajectory = obj.vehicle.generateTrajectoryFromPath(obj.vehicle.pathInfo.path);
                        obj.vehicle.setCurrentTrajectory(currentTrajectory); % Generate the new trajectory
                    end
                    
                    obj.vehicle.decisionUnit.futureData = FuturePlan;
                    obj.vehicle.pathInfo.calculateNewPathFlag = 0;
                    %% ------------------------------ FuturePlan Structure ------------------------------ nx6 --------------- 
                    % | car.id | RouteID | Estimated Average Speed | Estimated Entrance Time | Estimated Exit Time | PlannerType
                    % PlannerType: DigraphA*= -1, D*ExtraLite= -2, Shortest= -3, GridA*= non negative value
                else
                    %% If the Vehicle is still on Route -> Vehicle's future plan stays the same
                    %Output 1: Future plan of the vehicle
                    FuturePlan = obj.vehicle.decisionUnit.futureData; %Output 1: Future plan of the vehicle
                    waypointReached =0;                               %Output 2: Waypoint Reached enabler
                end
                
                %% Check if crossroad
                obj.crossroadCheck(obj.vehicle);
            end
            
            %% Grid path generation
            if mod(obj.getCurrentTime,1) == 0 % BOGPath generation sample time (update at every 1 second)
                % BOGPath is only generated if the Map type is GridMap
                obj.vehicle.pathInfo.BOGPath = obj.Map.generate_BOGPath(obj.Map,obj.vehicle.pathInfo.path,obj.vehicle.id,obj.vehicle.pathInfo.BOGPath);
            end
            
        end
        
        function crossroadCheck(~,car)
            
            crossroadId = car.decisionUnit.inCrossroad(1);
            crossroadZone = car.decisionUnit.inCrossroad(2);
            
            if crossroadId ~=0
                
                %log speed and energydata for current crossroad
                if crossroadZone > 1
                    car.dataLog.speedInCrossroad = [car.dataLog.speedInCrossroad car.dynamics.speed];
                end
                
                if crossroadZone > 0
                    car.dataLog.speedInCrossroad2 = [car.dataLog.speedInCrossroad2 car.dynamics.speed];
                end
                
                if car.map.crossroadUnits(crossroadId).params.conventionalTrafficLights == 1
                    car.map.crossroadUnits(crossroadId).updateTrafficStateFromConventionalSystem(obj.getCurrentTime);
                end
                
                if crossroadZone == 2
                    
                    if car.decisionUnit.breakingFlag == 1
                        car.pathInfo.stopAt = car.pathInfo.path(2);
                    else
                        car.pathInfo.stopAt = 0;
                        car.setStopStatus(false);
                    end
                end
            else
                car.pathInfo.stopAt = 0;
            end
            
            
        end
        
        function futureData = CollectFutureData(obj, Vehicles, CommunicationIDs)
            i = 1:length(Vehicles);
            
            i = i(CommunicationIDs==1); % Remove the vehicles that don't have V2V connection to the car
            i(obj.vehicle.id)=[]; % Remove the car with the same id
            
            futureData =cat(1,cat(1,[Vehicles(i).decisionUnit]).futureData);
            
        end
        
        function OtherVehiclesFutureData = checkEmptyFutureData(~,OtherVehiclesFutureData)
            if isempty(OtherVehiclesFutureData)
                OtherVehiclesFutureData = [0 0 0 0 0 0]; % Default nx6 Structure to avoid any size errors
            end
        end
                       
        function path = composePath(~,waypoints, startingPoint, endingPoint)
            %% define path from waypoints array
            predecessor = waypoints(endingPoint,2);
            i = 2;
            path = zeros(1,15); % Memory preallocation limit of 15 can be edited
            path(1) = endingPoint;
            try
                while (predecessor ~= startingPoint)
                    path(i)= predecessor;
                    i = i + 1;
                    predecessor = waypoints(predecessor,2);
                    
                end
            catch
                disp('Path not found error')
            end
            path(path==0)=[]; %remove the extra cells created by memory allocation
            path(i) = startingPoint;
            path = fliplr(path);
            
        end
        
        function bool = checkforAccelerationPhase(obj,currentSpeed,maxSpeed)
            if abs(maxSpeed - currentSpeed) > 1 && obj.accelerationPhase(1) == 0
                bool = true;
            else
                bool = false;
            end
        end
        
        function accelerationPhaseData = setAccelerationPhase(obj,currentSpeed,maxSpeed)
            
            % Neural Network is used to get average acceleration value
            averageAcceleration = NN_acceleration([currentSpeed; maxSpeed-currentSpeed]);
            accelerationDistance = obj.getAccelerationDistance(averageAcceleration, currentSpeed, maxSpeed);
            accelerationPhaseData = [1,currentSpeed,maxSpeed, accelerationDistance, averageAcceleration];
            
        end

        % Equation 5 & 8 in NecSys Paper
        function accelerationDistance = getAccelerationDistance(~, averageAcceleration, currentSpeed, speedTo)
            delta_v = speedTo-currentSpeed;
            accelerationDistance = delta_v^2/(2*averageAcceleration)+currentSpeed*delta_v/averageAcceleration;
        end
        
        % Eigenvalues of Equation 8 in NecSys Paper
        function timeToReach = timeToReachNextWaypointInAccelerationPhase(~, currentSpeed, averageAcceleration, distance)
            timeToReach = -currentSpeed/averageAcceleration + sqrt((currentSpeed/averageAcceleration)^2+2*distance/averageAcceleration);
        end
        
        %% Standard Simulink Output functions
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            
            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);
            
            % Set private and protected properties
            %s.myproperty = obj.myproperty;
        end
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon("PathPlanner.png");
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            
            % Set private and protected properties
            % obj.myproperty = s.myproperty;
            
            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
        
    end
    
    methods(Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename('class'));
        end
        
        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(mfilename('class'));
        end
        
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
        
        function ds = getDiscreteStateImpl(~)
            % Return structure of properties with DiscreteState attribute
            ds = struct([]);
        end
        
        function flag = isInputSizeLockedImpl(~,~)
            % Return true if input size is not allowed to change while
            % system is running
            flag = true;
        end
        
        function out = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 1];
            
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
        
        function out = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = 'double';
            
            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end
        
        function out = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
        
        function out = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            
            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
        
    end
    
    %% Abstract Methods / Must be implemented by Subclasses
    methods (Abstract, Access = protected)
        % --------------------------------FuturePlan Structure----nx6-----------------------------------
        % | car.id | RouteID | Estimated Average Speed | Estimated Entrance Time | Estimated Exit Time | -1 for Digraph
        FuturePlan = findPath(obj,OtherVehiclesFutureData)
        
        % Every Path Planner should delete the collided Vehicle Future Data in their own way
        OtherVehiclesFutureData = deleteCollidedVehicleFutureData(obj,OtherVehiclesFutureData)
        
    end
end