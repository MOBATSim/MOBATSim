classdef VehiclePathPlanner_GridAStar< VehiclePathPlanner
    % This Path Planner Block uses A* algorithm to find routes to reach the destination node according to the shared data from the other vehicles.
    %
    % NOTE: When renaming the class name Untitled, the file name
    % and constructor name must be updated to use the class name.
    %
    % This template includes most, but not all, possible properties, attributes,
    % and methods that you can implement for a System object in Simulink.
    
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
        %variables for gridA*
        tempGoalNode;               % temporary goal node     
        parentMap;                  % Input: current key, Output: parent key
    end
    
    methods
        % Constructor
        function obj = VehiclePathPlanner_GridAStar(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.vehicle = evalin('base',strcat('Vehicle',int2str(obj.Vehicle_id)));
            
            obj.accelerationPhase =  zeros(1,5);
            obj.simSpeed = evalin('base','simSpeed');
            obj.breakingFlag = 0;
            obj.inCrossroad = [0 0];
            
            obj.initializeGrid();
        end
        
        %% Grid A* Code
        function newFutureData = gridAStar(obj, globalTime,futureData)
            %this function performs a A* search with the grid location
            %objects from obj.Map.gridLocationMap
            %futureData = [carID, Grid x value, Grid y value, speed, time, deviation]
            %globalTime is the current time of the simulation
            %newFutureData is the newly created FD from this vehicle
            
            %% prepare the search
            if isempty(futureData)
                futureData = [0 0 0 0 0 0];
            else
                futureData = deleteCollidedVehicleFutureData(obj,futureData);
            end
            futureData = obj.detectBlockingCarsGridForLoop(futureData);
            %% Update temp goal if we reached our temporary goal
            if obj.tempGoalNode == obj.vehicle.pathInfo.lastWaypoint
                obj.tempGoalNode = obj.vehicle.pathInfo.destinationPoint;
                %we have to repush our goal to the open list to search if
                %the path is still blocked
            end
            carID = obj.vehicle.id;
            %first we will set up our current position as a field
            openList = containers.Map();
            closedList = containers.Map();
            %add your location to the open list
            currentPosition = obj.Map.get_coordinates_from_waypoint(obj.vehicle.pathInfo.lastWaypoint);
            currentGridPosition = obj.Map.bogMap.world2grid([currentPosition(1)-obj.Map.xOffset,-currentPosition(3)-obj.Map.yOffset]);
            currentKey = append( num2str(currentGridPosition(1)),",",num2str(currentGridPosition(2)) );
            startKey = currentKey;
            currentGrid = obj.Map.gridLocationMap(currentKey);
            currentGrid.gValue = globalTime;
            currentGrid.speedVector(carID) = obj.vehicle.dynamics.speed;
            openList(currentKey) = currentGrid;
            
            %get key of goal node
            currentPosition = obj.Map.get_coordinates_from_waypoint(obj.tempGoalNode);
            goalPosition = obj.Map.bogMap.world2grid([currentPosition(1)-obj.Map.xOffset,-currentPosition(3)-obj.Map.yOffset]);
            goalKey = append( num2str(goalPosition(1)),",",num2str(goalPosition(2)) );
            goalCoordinates = str2num(goalKey); %#ok<*ST2NM>
            
            %check acceleration
            if obj.checkforAccelerationPhase(obj.vehicle.dynamics.speed,obj.vehicle.dynamics.maxSpeed)
                obj.accelerationPhase = setAccelerationPhase(obj,obj.vehicle.dynamics.speed,obj.vehicle.dynamics.maxSpeed);
            end
            
            %% start searching
            while ~isempty(openList.keys)
                %as long as open list is not empty
                %search for min f in open list
                
                openListKeys = openList.keys;
                currentKey = openListKeys{1};
                currentGrid = openList(currentKey); %preassign first entry
                openListKeys = openListKeys(1,2:end);
                for k = openListKeys
                    newGrid = openList(k{1,1});
                    if newGrid.fValue < currentGrid.fValue
                        currentGrid = newGrid;
                        currentKey = k{1,1};
                    end
                end
                %now remove the Grid from the open list
                openList.remove(currentKey);
                % calculate time over currentGrid
                [nextSpeed,travelTime] = obj.checkACCOnGrid(currentGrid,currentGrid.speedVector(carID));
                currentGrid = obj.correctTimeWithFD(carID,nextSpeed,obj.simSpeed,travelTime,currentGrid,futureData);
                %now we can push the currentGrid  to the closed list
                closedList(currentKey) = currentGrid;  
                %% for every successor
                [successors,lastNodeNR] = obj.getSuccessors(currentGrid,currentGrid.edgeStart);
                %now check every successor
                for successorKey = successors    
                    %test for block or closed list
                    if closedList.isKey(successorKey) || obj.isSuccBlocked(successorKey,futureData,currentGrid.gValue + travelTime)
                        %if the successor is already in the closed list or
                        %blocked we can ignore it
                        continue;
                    end 
                    successorGrid = obj.Map.gridLocationMap(successorKey);
                    %% regular search step
                    %if we have yet to reach the goal, we continue the search
                    successorGrid = obj.getGLCosts(carID,currentGrid.speedVector(carID),currentGrid.timeVector(carID),obj.simSpeed,obj.vehicle.dynamics.maxSpeed,currentGrid.gValue,currentGrid.totalDistance,currentGrid.distance,successorGrid,goalCoordinates);
                    %% if we found the goal
                    if strcmp(successorKey , goalKey)
                        %we found the goal node
                        %set parent
                        obj.parentMap(successorKey) = currentKey;
                        %mark as blocked
                        successorGrid.deviation = successorGrid.deviation * -1;
                        %push it to closed list
                        closedList(successorKey) = successorGrid;
                        %build path
                        [newFutureData,newPath] = obj.gridBuildPath(closedList, goalKey, startKey);
                        obj.vehicle.pathInfo.path = newPath;
                        
                        return;
                    end
                    %% test the rest                    
                    if openList.isKey(successorKey) && openList(successorKey).fValue <= successorGrid.fValue
                        %if we already have a shorter way to go on succGL,
                        %we dont need to push
                        continue;
                    else
                        %set parent
                        obj.parentMap(successorKey) = currentKey;
                        %set start of edge
                        successorGrid.edgeStart = lastNodeNR;
                        %add to open list
                        openList(successorKey) = successorGrid;
                    end                    
                end                              
            end
            %% no path was found
            %if we reach this code, no path was found
            %try to update to a closer node
            if obj.newGoalNodeGrid(futureData)
                newFutureData = obj.gridAStar(globalTime,futureData);
                return;
            else
                %we cant reach any node from now
                disp(["No possible path was found from vehicle " num2str(carID)])
                obj.stopVehicle();
                newFutureData = [];
                return;
            end
        
        end
        
        function initializeGrid(obj)
            %set up everything to perform a search inside the grid
            obj.tempGoalNode = obj.vehicle.pathInfo.destinationPoint;
            obj.parentMap = containers.Map();
        end
        function [newFD,newPath] = gridBuildPath(obj, closedList, goalKey, startKey)
            curGL = closedList(goalKey);
            curKey = goalKey;
            newPath = [];
            newFD = [];
            while ~strcmp(curKey,startKey)
                if strcmp(curKey,"")
                    %if we searched and there was no parent assigned, we
                    %made a mistake somewhere
                    disp(append("Error in pathbuilding with vehicle ", num2str(carID)) )
                    newFD = [];
                    newPath = [];
                    return;
                end
                
                %assign cur GL to path
                if curGL.nodeNR ~= 0
                    newPath = [newPath, curGL.nodeNR];
                end
                %build future data
                %FD [carID, coordinates of GL x, y, speed, time, deviation]
                newFD = [newFD; [obj.vehicle.id, curGL.coordinates, curGL.speedVector(obj.vehicle.id), curGL.gValue, curGL.deviation]];
                %got to the parent
                curKey = obj.parentMap(curKey);
                curGL = closedList(curKey);                
            end
            %we have built FD and path now
            %we need to flip path to be in order from start to goal
            newPath = fliplr([newPath, closedList(startKey).nodeNR]);
        end
        function [successors,lastNodeNR] = getSuccessors(~,curGL,startNodeNR)
            %returns the successors of the grid location curGL
            if curGL.nodeNR ~= 0
                    %node
                    successors = curGL.successors;
                    lastNodeNR = curGL.nodeNR;
                else
                    %part of edge
                    successors = curGL.transMapSucc(num2str(startNodeNR));                    
                    lastNodeNR = curGL.edgeStart;
            end
        end
        function curGL = correctTimeWithFD(~,carID,nextSpeed,simSpeed,travelTime,curGL,futureData)
            %to know how long we actually took, we need to keep future data
            %in mind
            %FD [carID, coordinates of GL x, y, speed, time, deviation]
            %% check for other cars on current grid location
            %we want to know how long it takes to reach succGL
            currentFutureData = futureData(futureData(:,2) == curGL.coordinates(1) & futureData(:,3) == curGL.coordinates(2),:);
            if ~isempty(currentFutureData)
                curGL.timeVector(carID) = travelTime + curGL.gValue;                
                %we now need to calculate probability
                %for that we use pdf
                for j = 1 : size(currentFutureData,1)
                    id = currentFutureData(j,1);
                    x = [curGL.timeVector(carID)-curGL.deviation,curGL.timeVector(carID)+curGL.deviation];
                    mu = currentFutureData(j,5);
                    sigma = abs(currentFutureData(j,6));                    
                    %assign the pbb value to the car id entry in timeVcetor
                    %using normal distribution
                    pbb = normcdf(x,mu,sigma);
                    curGL.probabilityVector(id) = pbb(2)-pbb(1);
                    curGL.speedVector(id) = currentFutureData(j,4);
                end
                %% now use the probability to alter travel time
                otherCars = currentFutureData(:,1)';
                for car = otherCars
                    if curGL.probabilityVector(car) > 0.3 && curGL.speedVector(car) < nextSpeed
                        %% disturbing car on same route
                        nextSpeed = curGL.speedVector(car);
                        travelTime = (1/ simSpeed) *curGL.distance * (1/ nextSpeed);% 
                    end
                end
                curGL.timeVector(carID) = travelTime;
                curGL.speedVector(carID) = nextSpeed;
            end
        end
        
        function succGL = getGLCosts(~,carID,nextSpeed,travelTime,simSpeed,maxSpeed,curG,curTotalDistance,curDistance,succGL,goalCoordinates)
            %this function calculates the cost to get through curGL to succGL            
            %% now calculate the g and f values
            succGL.gValue = curG + travelTime;%we need this much time to go to succGL
            succGL.totalDistance = curTotalDistance + curDistance;%we travelled so much so far
            succGL.speedVector(carID) = nextSpeed;%this will be our speed on this GL
            h = (1/ simSpeed) * (1/maxSpeed) * norm(goalCoordinates - succGL.coordinates);
            succGL.fValue = succGL.gValue + h;
            %% calculate deviation
            %deviation d(speed,totalDistance)
%             x = (maxSpeed*10 + curTotalDistance)/1000;
%             succGL.deviation = travelTime * power(x,2);
            succGL.deviation = 0.01*succGL.gValue;
        end
        
        function blocked = isSuccBlocked(~,succKey,futureData,arrivalTime)
            %this function returns true, if the node is blocked
            %FD [carID, coordinates of GL x, y, speed, time, deviation]
            coordinates = str2num(succKey);
            %get every entry in fd that has the same coordinates and
            %negative sigma (is blocked)
            curFD = futureData(futureData(:,2) == coordinates(1) & futureData(:,3) == coordinates(2) & futureData(:,6) < 0,:);            
            %now check, if someone would block it before we arrive
            for j = 1 : size(curFD,1)
%                 x = [0,arrivalTime];
%                 mu = curFD(j,5);
%                 sigma = curFD(j,6);
%                 pbb = normcdf(x,mu,sigma);
%                 p = pbb(2)-pbb(1);
%                 if p > 0
%                     blocked = true;
%                     return;
%                 end
                    if arrivalTime > curFD(j,5)
                        blocked = true;
                        return;
                    end
            end
            blocked = false;
        end
        
        function futureData = deleteCollidedVehicleFutureData(obj,futureData)
            %deletes future data of vehicles that will not move because of collision
            %FD [carID, coordinates of GL x, y, speed, time, deviation]
            otherCars = unique(futureData(:,1))'; % OtherCars which have the same FutureData
            otherCars(otherCars==0) = []; % If zero comes as index, make it empty
            vehicles = obj.Map.Vehicles;
            for carID = otherCars
                if vehicles(carID).status.collided
                    %remove every entry with the collided car from FD
                    futureData = futureData(futureData(:,1)~=carID,:);
                    %% block the start node of the crash
                    coords = obj.Map.waypoints(vehicles(carID).pathInfo.lastWaypoint,:);
                    coords = obj.Map.bogMap.world2grid([coords(1)-obj.Map.xOffset,-coords(3)-obj.Map.yOffset]);                    
                    futureData = [futureData;[carID , coords , 0, 0, -1]];
                    %% block the future node of the crash
                    coords = obj.Map.waypoints(vehicles(carID).pathInfo.path(2),:);
                    coords = obj.Map.bogMap.world2grid([coords(1)-obj.Map.xOffset,-coords(3)-obj.Map.yOffset]);
                    futureData = [futureData;[carID , coords , 0, 0, -1]];
                end                
            end
        end
        function possible = newGoalNodeGrid(obj,futureData)
            %try to update obj.tempGoalNode to a node that is not blocked
            %FD [carID, coordinates of GL x, y, speed, time, deviation]
            path = obj.vehicle.pathInfo.path;
            for i = 3 : length(path)
                %test every node if it is blocked (3 is the next node)
                %we go on our previous path until we reach a blocked one
                coords = obj.Map.waypoints(path(i),:);
                coords = obj.Map.bogMap.world2grid([coords(1)-obj.Map.xOffset,-coords(3)-obj.Map.yOffset]);
                if ~isempty(futureData(   futureData(:,2) == coords(1) & futureData(:,3) == coords(2) &  futureData(:,6)<0  ))
                    %if we have encountered a block, we need to see if it
                    %is reachable
                    i = i-1;
                    if i > 2
                        obj.tempGoalNode = path(i);
                        possible = true;                      
                    else
                        possible = false;
                    end
                    return;
                end
            end
            possible = false;
        end
        function futureData = detectBlockingCarsGridForLoop(obj,futureData)
            %if a car without future data blocks a node, we set the node blocked
            allCars = 1:length(obj.Map.Vehicles);
            otherCars = allCars(allCars ~= obj.vehicle.id);
            
            % TODO: needs to be vectorized like: [cat(1,obj.Map.Vehicles(otherCars).pathInfo).destinationReached] == 1
            for carId = otherCars
                if obj.Map.Vehicles(carId).pathInfo.destinationReached                    
                    coords = obj.Map.waypoints(obj.Map.Vehicles(carId).pathInfo.lastWaypoint,:);
                    coords = obj.Map.bogMap.world2grid([coords(1)-obj.Map.xOffset,-coords(3)-obj.Map.yOffset]);
                    futureData = [futureData;[carId , coords , 0, 0, -1]];
                end
            end
        end
        function stopVehicle(obj)
            car = obj.vehicle;            
            %code from vehicle.checkifDestinationReached
            car.pathInfo.path = [];
            car.pathInfo.destinationReached = true;
            car.setStopStatus(true);
            car.pathInfo.routeCompleted = true;
            car.dynamics.speed = 0;
            car.dataLog.totalTravelTime = get_param(car.modelName,'SimulationTime');
            car.V2VdataLink(car.V2VdataLink==1) =0;
        end
        
        %% acceleration estimation related             
        function [nextSpeed,timeToReach] = checkACCOnGrid(obj,currentGL,currentSpeed)
            %nextSpeed = speed on end of edge, timeToReach = exit time of edge            
            
            distance = currentGL.distance;
            
            %% check for acceleration phase
            if obj.accelerationPhase(1) == 1
                accelerationDistance = obj.accelerationPhase(4);
                averageAcceleration = obj.accelerationPhase(5);
                %global distance
                currentTotalDistance = currentGL.totalDistance;
                if ((currentTotalDistance + distance - accelerationDistance) < 0)
                    % whole route in acceleration phase
                    timeToReach = (1/ obj.simSpeed) * (-currentSpeed/averageAcceleration + sqrt((currentSpeed/averageAcceleration)^2+2*distance/averageAcceleration));
                    nextSpeed = currentSpeed + averageAcceleration*timeToReach * obj.simSpeed;
                else
                    % route is divided in acceleration phase (t1)
                    % and constant speed phase (t2)
                    t1 = (1/ obj.simSpeed) * (-currentSpeed/averageAcceleration + sqrt((currentSpeed/averageAcceleration)^2+2*(accelerationDistance - currentTotalDistance)/averageAcceleration));
                    t2 =  (1/ obj.simSpeed) * (currentTotalDistance+ distance - accelerationDistance)/ currentGL.speedLimit(obj.vehicle.id);
                    timeToReach = t1+t2;
                    nextSpeed = obj.accelerationPhase(3);
                    obj.accelerationPhase = zeros(1,5); %set acceleration phase to zero
                    
                end
                
            else
                timeToReach =  (1/ obj.simSpeed) * distance* (1/ currentSpeed); %timesteps to reach neighbour
                nextSpeed = currentSpeed;
            end
        end
        
        
        %% Normal A* Code
        
        function [path, newFutureData]=findShortestPath(obj, car, startingPoint, endingPoint, global_timesteps, futureData)
            %% Initialization
            if isempty(futureData)
                futureData = [0 0 0 0 0];
            end

            % TODO: Explanation of the code
            waypoints =  zeros(length(obj.Map.waypoints),7);
            
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
                    currentTime = waypoints(currentNode,5);
                    currentTotalDistance = waypoints(currentNode,7);
                    currentSpeed = waypoints(currentNode,4);
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
                    currentFutureData = futureData(futureData(:,2) == currentRoute,:);
                    currentFutureData = currentFutureData(currentFutureData(:,4)<= currentTime & currentFutureData(:,5)>currentTime,:);
                    if ~isempty(currentFutureData)
                        
                        %% disturbing car on same route
                        index = find(max(currentFutureData(:,5)));
                        timeToReachDisturbingVehicle = currentFutureData(index,5);
                        speedDisturbingVehicle =  currentFutureData(index,3);
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


        function FuturePlan = findPath(obj,OtherVehiclesFutureData)
            OtherVehiclesFutureData = obj.getOnlyGridFutureData(OtherVehiclesFutureData); % TODO - remove later, just for testing
            FuturePlan = obj.gridAStar(get_param(obj.modelName,'SimulationTime'),OtherVehiclesFutureData);
            
            FuturePlan(1,6) = -FuturePlan(1,6);% TODO - find the source of the problem rather than this work around

        end
        

        
        

    end
    
    
end
