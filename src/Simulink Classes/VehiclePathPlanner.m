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
        
        
        
        function FuturePlan = findNextRoute(obj, car, starting_point, ending_point, global_timesteps,futureData)
            
            [Path,newFutureData] = obj.findFastestPath(car, starting_point, ending_point, global_timesteps, futureData);
            
            car.setPath(Path);
            
            FuturePlan = newFutureData;
            
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
        
        function [path, newFutureData] = findFastestPath(obj, car, startingPoint, endingPoint, global_timesteps, futureData)
            %This function performs a normal A* search inside the Digraph.
            %OUTPUT: newFutureData = | car.id | RouteID | Estimated Average Speed | Estimated Entrance Time | Estimated Exit Time |
            %path = [nr of nodes from start to finish]
            %% Initialization
            if isempty(futureData)
                futureData = [0 0 0 0 0]; % 1x5 to fit the output size
            end
            
            % Create a table for all the waypoints (nodes) where each row index is an identifier for a waypoint
            waypoints =  zeros(length(obj.Map.waypoints),7); % initialize with zeros (later could be turned into table but might lead to overhead)
            % --------------------------------------------Waypoints Structure----nx7-----------------------------------------------------
            % | State of Waypoint | Current Node | Current Route | Current Speed | Time Costs | Time + Heuristic Costs | Total distance |
            
            %get maximum speed for every edge
            maxSpeed = car.dynamics.maxSpeed ;
            currentSpeed = car.dynamics.speed ;
            speedRoutes = [obj.Map.connections.circle(:,end);obj.Map.connections.translation(:,end)];
            maxSpeedRoutes = speedRoutes;
            maxSpeedRoutes(speedRoutes>maxSpeed)= maxSpeed; %possible speed for every route
            
            connections = obj.Map.connections.all;
            
            distances = obj.Map.connections.distances;
            
            currentNode = startingPoint; %currentNode: current Node from where to expand neighbour Nodes
            waypoints(startingPoint,5) = global_timesteps;
            waypoints(startingPoint,4) = currentSpeed;
            
            %% check for acceleration phase
            if abs(maxSpeed - currentSpeed) > 1
                if obj.accelerationPhase(1) == 0
                    
                    % Neural Network is used to get average acceleration value
                    averageAcceleration = obj.simSpeed^2 * NN_acceleration([currentSpeed; maxSpeed-currentSpeed]);
                    accelerationDistance = obj.getAccelerationDistance(averageAcceleration, currentSpeed, maxSpeed);
                    obj.accelerationPhase = [1,currentSpeed,maxSpeed, accelerationDistance, averageAcceleration];
                end
            end
            %% main loop
            while (1)
                waypoints(currentNode,1) = 2; %set state of waypoint to 2 -> waypoint in closed List
                
                %% find neighbours
                routes2neighbourNode = find(connections(:,1) == currentNode); % route ID
                neighbourNodes = connections(connections(:,1) == currentNode,2); % waypointID of neighbour
                neighbourNodes_Routes = [neighbourNodes'; routes2neighbourNode'];
                %% loop over all neighbours
                for neighbourNode_Route=neighbourNodes_Routes
                    neighbourWP = waypoints(neighbourNode_Route(1),:); % waypointID of neighbour
                    currentTime = waypoints(currentNode,5); % time the car will reach the node
                    currentTotalDistance = waypoints(currentNode,7); %distance travveled unto this node
                    currentSpeed = waypoints(currentNode,4); % the speed of the car when entering the node
                    currentRoute = neighbourNode_Route(2); % route ID
                    
                    currentMaxSpeedRoutes = maxSpeedRoutes;
                    
                    %% check for acceleration phase
                    if obj.accelerationPhase(1) == 1
                        accelerationDistance = obj.accelerationPhase(4);
                        averageAcceleration = obj.accelerationPhase(5);
                        if ((currentTotalDistance + distances(currentRoute) - accelerationDistance) < 0)
                            % whole route in acceleration phase
                            timeToReach = 1/ obj.simSpeed * obj.timeToReachNextWaypointInAccelerationPhase( currentSpeed, averageAcceleration, distances(currentRoute));
                            nextSpeed = currentSpeed + averageAcceleration*timeToReach * obj.simSpeed;
                        else
                            % route is divided in acceleration phase (t1)
                            % and constant speed phase (t2)
                            t1 = 1/ obj.simSpeed * obj.timeToReachNextWaypointInAccelerationPhase(currentSpeed, averageAcceleration, accelerationDistance - currentTotalDistance);
                            t2 =  1/ obj.simSpeed * (currentTotalDistance+ distances(currentRoute) - accelerationDistance)/ currentMaxSpeedRoutes(currentRoute);
                            timeToReach = t1+t2;
                            nextSpeed = obj.accelerationPhase(3);
                            obj.accelerationPhase = zeros(1,5); %set acceleration phase to zero
                            
                        end
                        
                    else
                        timeToReach =  1/ obj.simSpeed * distances(currentRoute)/ currentMaxSpeedRoutes(currentRoute); %timesteps to reach neighbour
                        nextSpeed = currentSpeed;
                    end
                    %% check for other cars on same route (using merged future data)
                    %get every future data info for the current edge
                    currentFutureData = futureData(futureData(:,2) == currentRoute,:);
                    %relevant data has to contain an arrival time before
                    %current car and an exit time after that car
                    currentFutureData = currentFutureData(currentFutureData(:,4)<= currentTime & currentFutureData(:,5)>currentTime,:);
                    if ~isempty(currentFutureData)
                        %% disturbing car on same route
                        %search for the highest exit time, that will slow
                        %us down the most
                        index = find(max(currentFutureData(:,5)));%TODO use max function properly
                        timeToReachDisturbingVehicle = currentFutureData(index,5);
                        %get speed of the slower vehicle
                        speedDisturbingVehicle =  currentFutureData(index,3);
                        %currentTime = entry time of the edge
                        %timeToReach = how long does it take to drive over current edge
                        %timeToReachDisturbingVehicle = exit time of other vehicle
                        %differnce = current car exit time - other car exit time
                        timeDifference = (currentTime + timeToReach) - timeToReachDisturbingVehicle ;
                        
                        spacingTime = 6 * 1/obj.simSpeed;
                        if (timeDifference < spacingTime)
                            timeToReach = timeToReachDisturbingVehicle + spacingTime - currentTime;
                            nextSpeed = speedDisturbingVehicle;
                        end
                        
                        
                    end
                    
                    %% calculate costs (costs = distance/speed)
                    costs = timeToReach;
                    
                    %% calculate heuristic (Luftlinie)
                    heuristicCosts = 1/ obj.simSpeed * 1/maxSpeed * norm(get_coordinates_from_waypoint(obj.Map, neighbourNode_Route(1))-get_coordinates_from_waypoint(obj.Map, endingPoint));
                    
                    
                    %% update waypoints array
                    if neighbourWP(1) == 0
                        neighbourWP(2) = currentNode;
                        
                        neighbourWP(5) = waypoints(currentNode,5) + costs;
                        neighbourWP(1) = 1;
                        neighbourWP(3) = currentRoute;
                        neighbourWP(4) = nextSpeed;
                        neighbourWP(6) = waypoints(currentNode,5) + costs + heuristicCosts;
                        neighbourWP(7) = waypoints(currentNode,7) + distances(currentRoute) ;
                        
                    elseif  neighbourWP(1) == 1
                        
                        %% replace costs if smaller
                        if  (waypoints(currentNode,5)+ costs < neighbourWP(5))
                            
                            neighbourWP(2) = currentNode;
                            neighbourWP(5) = waypoints(currentNode,5) + costs;
                            neighbourWP(3) = currentRoute;
                            neighbourWP(4) = nextSpeed;
                            neighbourWP(6) = waypoints(currentNode,5 )+ costs + heuristicCosts;
                            neighbourWP(7) = waypoints(currentNode,7) + distances(currentRoute);
                            
                        end
                    end
                    waypoints(neighbourNode_Route(1),:) = neighbourWP;
                end
                
                
                %% loop exit conditions
                if ismember(1,waypoints(:,1)) % Check if there is any node with state 1 (open and touched)
                    minCosts = min(waypoints(waypoints(:,1) == 1,6)); % get waypoint with min costs
                else
                    break
                end
                
                if waypoints(endingPoint,1) ~= 0 % check if waypoint state is 1 or 2
                    if minCosts >  waypoints(endingPoint)
                        % Uncomment the code below to convert the waypoints into table to analyze
                        % array2table(waypoints,'VariableNames',{'State of Waypoint','Current Node','Current Route','Current Speed','Time Costs','Time + Heuristic Costs','Total distance'})
                        break
                    end
                end
                
                %% get new waypoint to analyze -> next iteration in loop
                currentNode =  find(waypoints(:,6) == minCosts& waypoints(:,1)==1);
                currentNode = currentNode(1);
                
            end
            
            % Compose the path by using the analyzed waypoints array
            path = obj.composePath(waypoints, startingPoint, endingPoint);
            
            %% updateFuture Date of this vehicle
            newFutureData = zeros((length(path)-1),5); %Matrix Preallocation
            for i = 1: (length(path)-1)
                newFutureData(i,:) = [car.id waypoints(path(i+1),3) waypoints(path(i+1),4) waypoints(path(i),5)  waypoints(path(i+1),5)];
            end
            
            if global_timesteps == 0 % save the future data at the beginning of the simulation for validation after simulation
                car.decisionUnit.initialFutureData = newFutureData;
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