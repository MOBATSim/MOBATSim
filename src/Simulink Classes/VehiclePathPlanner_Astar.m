classdef VehiclePathPlanner_Astar < VehiclePathPlanner
    % VehiclePathPlanner_A* Inherits the VehiclePathPlanner. Generates a path to reach the destination waypoint.
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.
    
    % Public, tunable properties
    properties
        
    end
    
    % Public, non-tunable properties
    properties(Nontunable)
        
    end
    
    properties(DiscreteState)
        
    end
    
    % Pre-computed constants
    properties(Access = private)
        
    end
    
    methods
        % Constructor
        function obj = VehiclePathPlanner_Astar(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods (Access = protected)
        
        function setupImpl(obj)
            setupImpl@VehiclePathPlanner(obj); % Inherit the setupImpl function of the Superclass @VehiclePathPlanner
        end
        
        function FuturePlan = findPath(obj,OtherVehiclesFutureData)
            
            OtherVehiclesFutureData = obj.getOnlyDigraphFutureData(OtherVehiclesFutureData); % TODO - remove later, just for testing
            OtherVehiclesFutureData = deleteCollidedVehicleFutureData(obj,OtherVehiclesFutureData);
            
            starting_point = obj.vehicle.pathInfo.lastWaypoint;
            ending_point = obj.vehicle.pathInfo.destinationPoint;
            
            [Path,newFutureData] = obj.AStarPathfinder(obj.vehicle, starting_point, ending_point, get_param(obj.modelName,'SimulationTime'), OtherVehiclesFutureData);
            
            obj.vehicle.setPath(Path);
            FuturePlan = newFutureData; 
        end
        
        
        function [path, newFutureData] = AStarPathfinder(obj, car, startingPoint, endingPoint, global_timesteps, futureData)
            %This function performs a normal A* search inside the Digraph.
            %OUTPUT: newFutureData = | car.id | RouteID | Estimated Average Speed | Estimated Entrance Time | Estimated Exit Time |
            %path = [nr of nodes from start to finish]
            %% Initialization
            if isempty(futureData)
                futureData = [0 0 0 0 0 -1]; % 1x6 to fit the output size
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
            if checkforAccelerationPhase(obj,currentSpeed,maxSpeed)
                obj.accelerationPhase = setAccelerationPhase(obj,currentSpeed,maxSpeed);
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
                    
                    %% If the vehicle is still in the acceleration phase
                    if obj.accelerationPhase(1) == 1
                        accelerationDistance = obj.accelerationPhase(4);
                        averageAcceleration = obj.accelerationPhase(5);
                        if ((currentTotalDistance + distances(currentRoute) - accelerationDistance) < 0)
                            % Whole route in acceleration phase
                            timeToReach = 1/ obj.simSpeed * obj.timeToReachNextWaypointInAccelerationPhase( currentSpeed, averageAcceleration, distances(currentRoute));
                            nextSpeed = currentSpeed + averageAcceleration*timeToReach * obj.simSpeed;
                        else
                            % The route is divided in two phases: First acceleration phase (t1)
                            % and the second: Constant speed phase (t2)
                            t1 = 1/ obj.simSpeed * obj.timeToReachNextWaypointInAccelerationPhase(currentSpeed, averageAcceleration, accelerationDistance - currentTotalDistance);
                            t2 =  1/ obj.simSpeed * (currentTotalDistance+ distances(currentRoute) - accelerationDistance)/ currentMaxSpeedRoutes(currentRoute);
                            timeToReach = t1+t2;
                            nextSpeed = obj.accelerationPhase(3);
                            obj.accelerationPhase = zeros(1,5); % Set acceleration phase data to zero
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
            newFutureData = zeros((length(path)-1),6); %Matrix Preallocation
            for i = 1: (length(path)-1)
                newFutureData(i,:) = [car.id waypoints(path(i+1),3) waypoints(path(i+1),4) waypoints(path(i),5)  waypoints(path(i+1),5) -1];
            end
            
            if global_timesteps == 0 % save the future data at the beginning of the simulation for validation after simulation
                car.decisionUnit.initialFutureData = newFutureData;
            end
            
            
        end
        
        function OtherVehiclesFutureData = deleteCollidedVehicleFutureData(obj,OtherVehiclesFutureData)
            otherCars = unique(OtherVehiclesFutureData(:,1))'; % OtherCars which have the same FutureData
            vehicles = obj.Map.Vehicles;
            for carID = otherCars
                if vehicles(carID).status.collided
                    %remove every entry with the collided car from FD
                    OtherVehiclesFutureData = OtherVehiclesFutureData(OtherVehiclesFutureData(:,1)~=carID,:);
                end
            end
        end
        
    end
    
    
end