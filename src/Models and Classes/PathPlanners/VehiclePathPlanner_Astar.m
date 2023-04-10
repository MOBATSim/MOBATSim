classdef VehiclePathPlanner_Astar < VehiclePathPlanner
    % VehiclePathPlanner_A* Inherits the VehiclePathPlanner. Generates a path to reach the destination waypoint.
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.
    
    % Contributors: Fabian Hart, Mustafa Saraoglu, Johannes Pintscher
    
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
                        
            starting_point = obj.vehicle.pathInfo.lastWaypoint;
            ending_point = obj.vehicle.pathInfo.destinationPoint;
            
            [Path,newFutureData] = obj.AStarPathfinder(obj.vehicle, starting_point, ending_point, obj.getCurrentTime, OtherVehiclesFutureData);
            
            obj.vehicle.setPath(Path);
            FuturePlan = newFutureData; 
        end
        
        
        function [path, newFutureData] = AStarPathfinder(obj, car, startingWaypoint, destinationWaypoint, global_timesteps, futureData)
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
            % | State of Waypoint | Current Node | Current Route | Maximum Speed | Time Costs | Time + Heuristic Costs | Total distance |

            maxSpeed = car.dynamics.maxSpeed;
            currentSpeed = car.dynamics.speed;
            
            connections = obj.Map.connections.all;
            
            distances = obj.Map.connections.distances;
            
            expandingWaypoint = startingWaypoint; %expandingWaypoint: the Node from where to expand to neighbouring waypoints
            waypoints(expandingWaypoint,5) = global_timesteps;
            waypoints(expandingWaypoint,4) = currentSpeed;
            
            %% The while loop will continue until it finds the lowest cost until the destination point
            while (1)
                waypoints(expandingWaypoint,1) = 2; %set state of the expanding waypoint to 2 -> waypoint in closed List
                
                %% find possible forward Routes
                possibleNextRoutes = find(connections(:,1) == expandingWaypoint); % Find the Route IDs
                
                %% Loop through all possible next Routes
                for possibleNextRoute=possibleNextRoutes'
                    possibleNextWaypointID = connections(possibleNextRoute,2); % end of Route - next Waypoint
                    nextWP_list = waypoints(possibleNextWaypointID,:); % next Waypoint - Future Data
                    
                    timeUntilReachingWP = waypoints(expandingWaypoint,5); % the time when the car will reach the node
                                        
                    %% Calculate time to reach the next waypoint(costs = distance/speed)
                    timeToReach = distances(possibleNextRoute)/ maxSpeed; % Estimated to to reach the next possible Waypoint
                    nextSpeed = maxSpeed;
                    
                    %% check for other cars on same route (using merged future data)
                    [timeToReach,nextSpeed]= obj.checkForSlowerVehicles(possibleNextRoute,timeToReach,nextSpeed,timeUntilReachingWP,futureData);
                    % Overwrite the Estimated time costs and speed if there
                    % is a slower vehicle [timeToReach,nextSpeed]
                    
                    %% calculate heuristic (Luftlinie)
                    heuristicTimeCostToDestination = 1/maxSpeed * norm(get_coordinates_from_waypoint(obj.Map, possibleNextWaypointID)-get_coordinates_from_waypoint(obj.Map, destinationWaypoint));
                    
                    %% update waypoints array
                    if nextWP_list(1) == 0
                        
                        nextWP_list(1) = 1; % WP checked state =1
                        nextWP_list(2) = expandingWaypoint;
                        nextWP_list(3) = possibleNextRoute;
                        nextWP_list(4) = nextSpeed;
                        nextWP_list(5) = waypoints(expandingWaypoint,5) + timeToReach;
                        nextWP_list(6) = waypoints(expandingWaypoint,5) + timeToReach + heuristicTimeCostToDestination;
                        nextWP_list(7) = waypoints(expandingWaypoint,7) + distances(possibleNextRoute) ;
                        
                    elseif  nextWP_list(1) == 1
                        
                        %% replace costs if smaller
                        if  (waypoints(expandingWaypoint,5)+ timeToReach < nextWP_list(5))
                            
                            nextWP_list(2) = expandingWaypoint;
                            nextWP_list(5) = waypoints(expandingWaypoint,5) + timeToReach;
                            nextWP_list(3) = possibleNextRoute;
                            nextWP_list(4) = nextSpeed;
                            nextWP_list(6) = waypoints(expandingWaypoint,5 )+ timeToReach + heuristicTimeCostToDestination;
                            nextWP_list(7) = waypoints(expandingWaypoint,7) + distances(possibleNextRoute); % Total distance the end of the Route
                            
                        end
                    end
                    waypoints(possibleNextWaypointID,:) = nextWP_list;
                end
                
                minCosts = min(waypoints(waypoints(:,1) == 1,6)); % get waypoint with min costs
                
                %% While(1) Loop "break" Condition Check
                if obj.checkIfLowestCostToDestinationFound(waypoints, destinationWaypoint, minCosts)
                    break
                else
                    %% Get another next possible waypoint to check -> Next iteration in loop
                    expandingWaypoint =  find(waypoints(:,6) == minCosts& waypoints(:,1)==1);
                    
                    % if length(expandingWaypoint)>1 % In case there are two next possible waypoints with exact same costs == minCosts
                    %   expandingWaypoint = expandingWaypoint(1);
                    % end
                end   
                
            end
            
            %% Compose the path by using the checked waypoints array
            path = obj.composePath(waypoints, startingWaypoint, destinationWaypoint);
            
            %% UpdateFuture Date of this vehicle
            newFutureData = zeros((length(path)-1),6); %Matrix Preallocation
            for i = 1: (length(path)-1)
                newFutureData(i,:) = [car.id waypoints(path(i+1),3) waypoints(path(i+1),4) waypoints(path(i),5)  waypoints(path(i+1),5) -1];
            end
                                    
        end
        
        function OtherVehiclesFutureData = deleteCollidedVehicleFutureData(obj,OtherVehiclesFutureData)       
            otherCarIDs = unique(OtherVehiclesFutureData(:,1))'; % OtherCars which have the same FutureData            
            
            % other cars with same future data found
            if otherCarIDs %#ok<BDSCI,BDLGI>                
                % find collided cars
                otherCars = obj.Map.Vehicles(otherCarIDs);
                collidedCarIDs = otherCarIDs([cat(1,otherCars.status).collided] == 1);
                
                if collidedCarIDs
                    % remove future data from collided cars
                    OtherVehiclesFutureData(ismember(OtherVehiclesFutureData(:,1),collidedCarIDs),:) = [];
                end
            end
            
        end
        
        function [timeToReach,nextSpeed] = checkForSlowerVehicles(~,nextRoute,timeToReach,nextSpeed,timeUntilReachingWP,futureData)
            %get every future data info for the current edge
            currentFutureData = futureData(futureData(:,2) == nextRoute,:);
            %relevant data has to contain an arrival time before
            %current car and an exit time after that car
            currentFutureData = currentFutureData(currentFutureData(:,4)<= timeUntilReachingWP & currentFutureData(:,5)>timeUntilReachingWP,:);
            if ~isempty(currentFutureData)
                %% Another car on same route
                %search for the highest exit time, that will slow us down the most
                index = find(max(currentFutureData(:,5)));
                timeToReachSlowerVehicle = currentFutureData(index,5);
                %get speed of the slower vehicle
                speedDisturbingVehicle =  currentFutureData(index,3);
                %currentTime = entry time of the edge
                %timeToReach = how long does it take to drive over current edge
                %timeToReachDisturbingVehicle = exit time of other vehicle
                %differnce = current car exit time - other car exit time
                timeDifference = (timeUntilReachingWP + timeToReach) - timeToReachSlowerVehicle ;
                
                spacingTime = 6;
                if (timeDifference < spacingTime)
                    timeToReach = timeToReachSlowerVehicle + spacingTime - timeUntilReachingWP;
                    nextSpeed = speedDisturbingVehicle;
                end
            end
            
        end
        
        function bool = checkIfLowestCostToDestinationFound(~,waypoints, destinationWaypoint, minCosts)
            bool = false;
            costToDest = waypoints(destinationWaypoint,6);
            
            % Check if waypoint state is 1 or 2 && Lowest cost found until destination       
            if (waypoints(destinationWaypoint,1) ~= 0) && (minCosts == costToDest)
                bool = true;
            end
        end
        
    end
    
    
end