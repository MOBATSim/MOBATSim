classdef VehiclePathPlanner_GridAStar< VehiclePathPlanner
    % This Path Planner Block uses A* algorithm to find routes to reach the destination node according to the shared data from the other vehicles.
    %
    % NOTE: When renaming the class name Untitled, the file name
    % and constructor name must be updated to use the class name.
    %
    % This template includes most, but not all, possible properties, attributes,
    % and methods that you can implement for a System object in Simulink.
    
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
            setupImpl@VehiclePathPlanner(obj); % Inherit the setupImpl function of the Superclass @VehiclePathPlanner
            
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
                currentGrid = obj.correctTimeWithFD(carID,nextSpeed,travelTime,currentGrid,futureData);
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
                    successorGrid = obj.getGLCosts(carID,currentGrid.speedVector(carID),currentGrid.timeVector(carID),obj.vehicle.dynamics.maxSpeed,currentGrid.gValue,currentGrid.totalDistance,currentGrid.distance,successorGrid,goalCoordinates);
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
                disp(strcat("No possible path was found for Vehicle ",num2str(carID)))
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
        
        function curGL = correctTimeWithFD(~,carID,nextSpeed,travelTime,curGL,futureData)
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
                        travelTime = curGL.distance * (1/ nextSpeed);% 
                    end
                end
                curGL.timeVector(carID) = travelTime;
                curGL.speedVector(carID) = nextSpeed;
            end
        end
        
        function succGL = getGLCosts(~,carID,nextSpeed,travelTime,maxSpeed,curG,curTotalDistance,curDistance,succGL,goalCoordinates)
            %this function calculates the cost to get through curGL to succGL            
            %% now calculate the g and f values
            succGL.gValue = curG + travelTime;%we need this much time to go to succGL
            succGL.totalDistance = curTotalDistance + curDistance;%we travelled so much so far
            succGL.speedVector(carID) = nextSpeed;%this will be our speed on this GL
            h = 1/maxSpeed * norm(goalCoordinates - succGL.coordinates);
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
        
        function stopVehicle(obj)% TODO - Remove this from here, if needed, add a function in Vehicle class
            car = obj.vehicle;            
            %code from vehicle.checkifDestinationReached
            car.pathInfo.path = [];
            car.pathInfo.destinationReached = true;
            car.setStopStatus(true);
            % car.setRouteCompleted(true); % TODO - needs to be removed, keeping it for now just in case
            car.updateActualSpeed(0);
            car.dataLog.totalTravelTime = obj.getCurrentTime;
            car.V2VdataLink(car.V2VdataLink==1) =0;
        end
      
        function [nextSpeed,timeToReach] = checkACCOnGrid(obj,currentGL,currentSpeed)
            % acceleration estimation related  
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
                    timeToReach = -currentSpeed/averageAcceleration + sqrt((currentSpeed/averageAcceleration)^2+2*distance/averageAcceleration);
                    nextSpeed = currentSpeed + averageAcceleration*timeToReach;
                else
                    % route is divided in acceleration phase (t1)
                    % and constant speed phase (t2)
                    t1 = -currentSpeed/averageAcceleration + sqrt((currentSpeed/averageAcceleration)^2+2*(accelerationDistance - currentTotalDistance)/averageAcceleration);
                    t2 = (currentTotalDistance+ distance - accelerationDistance)/ currentGL.speedLimit(obj.vehicle.id);
                    timeToReach = t1+t2;
                    nextSpeed = obj.accelerationPhase(3);
                    obj.accelerationPhase = zeros(1,5); %set acceleration phase to zero
                    
                end
                
            else
                timeToReach = distance* (1/ currentSpeed); %timesteps to reach neighbour
                nextSpeed = currentSpeed;
            end
        end
        
        function FuturePlan = findPath(obj,OtherVehiclesFutureData)
            FuturePlan = obj.gridAStar(obj.getCurrentTime,OtherVehiclesFutureData);
            
            FuturePlan = obj.checkEmptyFutureData(FuturePlan); % TODO: A rare bug occurs when GridA* vehicle arrives last
            FuturePlan(1,6) = -FuturePlan(1,6);% TODO: find the source of the problem rather than this work around

        end
        

        
        

    end
    
    
end
