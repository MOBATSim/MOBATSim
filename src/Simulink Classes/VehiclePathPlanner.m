classdef VehiclePathPlanner < matlab.System & handle & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
    % This Path Planner Block uses A* algorithm to find routes to reach the destination node according to the shared data from the other vehicles.
    %
    % NOTE: When renaming the class name Untitled, the file name
    % and constructor name must be updated to use the class name.
    %
    % This template includes most, but not all, possible properties, attributes,
    % and methods that you can implement for a System object in Simulink.
    
    % Public, tunable properties
    properties
        Vehicle_id
    end
    
    % Public, non-tunable properties
    properties(Nontunable)
        
    end
    
    properties(DiscreteState)
        
    end
    
    % Pre-computed constants
    properties(Access = protected)
        vehicle
        Map = evalin('base','Map');
        accelerationPhase;
        simSpeed = evalin('base','simSpeed');
        modelName = evalin('base','modelName');
        initialFutureData
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
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.vehicle = evalin('base',strcat('Vehicle',int2str(obj.Vehicle_id)));
            
            obj.accelerationPhase =  zeros(1,5);
            obj.simSpeed = evalin('base','simSpeed');
            obj.breakingFlag = 0;
            obj.inCrossroad = [0 0];
        end
        
        function [FuturePlan, waypointReached] = stepImpl(obj,OtherVehiclesFutureData)
            %% Check if destination is already reached
            if obj.vehicle.pathInfo.destinationReached
                FuturePlan = obj.vehicle.decisionUnit.futureData;
                waypointReached=1;
            else
                %% Check if destination is reached now
                % If the vehicle has reached its destination it should stop
                obj.vehicle.checkifDestinationReached();
                
                %% Check if the vehicle has completed its route but still did not reach its destination
                if obj.vehicle.pathInfo.routeCompleted == 1 && obj.vehicle.pathInfo.destinationReached == 0
                    % Time Stamps are logged when waypoints are reached
                    obj.vehicle.dataLog.timeStamps = [obj.vehicle.dataLog.timeStamps;[obj.vehicle.pathInfo.lastWaypoint get_param(obj.modelName,'SimulationTime')]];

                    % Vehicle continues to move so the Stop is set to false
                    obj.vehicle.setStopStatus(false);
                    
                    % Build the future plan by deriving the next routes and building the path
                    %Output 1: Future plan of the vehicle   
                    FuturePlan = findPath(obj,OtherVehiclesFutureData); % This is an abstract that is implemented separately in each subclass
                    % --------------------------------FuturePlan Structure----nx5-----------------------------------
                    % | car.id | RouteID | Estimated Average Speed | Estimated Entrance Time | Estimated Exit Time |
                    

                else
                    %% If the vehicle is still on its route -> the Future Plan stays the same
                    %Output 1: Future plan of the vehicle
                    FuturePlan = obj.vehicle.decisionUnit.futureData;
                end
                
                %% Check if crossroad
                obj.crossroadCheck(obj.vehicle);
                
                
                %Output 2: Waypoint Reached enabler
                if obj.vehicle.pathInfo.routeCompleted
                    waypointReached =1;
                else
                    waypointReached =0;
                end
            end
            %% Grid path generation
            if mod(get_param(obj.modelName,'SimulationTime'),0.2) == 0
                % Plotting can decrease performance, so dont update to often (update at every 0.2 seconds)
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
                    car.map.crossroadUnits(crossroadId).updateTrafficStateFromConventionalSystem(get_param(obj.modelName,'SimulationTime'));
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
            averageAcceleration = obj.simSpeed^2 * NN_acceleration([currentSpeed; maxSpeed-currentSpeed]);
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
            flag = false;
        end
        
        function [out,out2] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [50 5];
            out2 = [1 1];
            
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
        
        function [out,out2] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = 'double';
            out2 = 'double';
            
            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end
        
        function [out,out2] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
        
        function [out,out2] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = false;
            out2 = true;
            
            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
        
    end
    
    methods (Abstract, Access = protected)
        % --------------------------------FuturePlan Structure----nx5-----------------------------------
        % | car.id | RouteID | Estimated Average Speed | Estimated Entrance Time | Estimated Exit Time |
        FuturePlan = findPath(obj,OtherVehiclesFutureData)

    end
end