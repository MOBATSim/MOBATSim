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
    properties(Access = private)
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
        
        %variables for gridA*
        tempGoalNode;
        
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
            
            obj.initializeGrid();
        end
        
        
        
        
        
        
        
        
        
        
        function [FuturePlan, waypointReached] = stepImpl(obj,OtherVehiclesFutureData)
            %This block shouldn't run if the vehicle has reached its
            %destination 
            
            %% visualization
            %comment this in if you want the visualization of the cars
            if mod(get_param(obj.modelName,'SimulationTime'),0.2) == 0 
                %plotting can decrease performance, so dont update to often
                delete(obj.pathPlot)
                obj.pathPlot = plotPath(obj.Map,obj.vehicle.pathInfo.path,obj.vehicle.id);
            end
            
            %% choose path
            if obj.vehicle.pathInfo.destinationReached %check if already reached goal
                FuturePlan = obj.vehicle.decisionUnit.futureData;
                %FuturePlan = [];
                waypointReached=1;
            else
                % Firstly check if the vehicle has reached its destination so it stops.
                % Then check if the vehicle has completed its route but still needs to reach its destination
                if obj.vehicle.pathInfo.routeCompleted == 1 && ~obj.vehicle.checkifDestinationReached()
                    % Check if crossroad
                    obj.crossroadCheck(obj.vehicle);
                    % Time Stamps are logged when waypoints are reached
                    obj.vehicle.dataLog.timeStamps = [obj.vehicle.dataLog.timeStamps;[obj.vehicle.pathInfo.lastWaypoint get_param(obj.modelName,'SimulationTime')]];
                    
                    %Build the future plan by deriving the next routes and building the path
                    %Output 1: Future plan of the vehicle
                    
                    %FuturePlan = obj.findNextRoute(obj.vehicle, obj.vehicle.pathInfo.lastWaypoint, obj.vehicle.pathInfo.destinationPoint,get_param(obj.modelName,'SimulationTime'),OtherVehiclesFutureData);
                    FuturePlan = obj.gridAStar(get_param(obj.modelName,'SimulationTime'),OtherVehiclesFutureData);
                    obj.vehicle.setStopStatus(false);
                    waypointReached =1;
                else
                    % If the vehicle is still on its route then the future data stays the same
                    %Output 1: Future plan of the vehicle
                    FuturePlan = obj.vehicle.decisionUnit.futureData;
                    waypointReached =0;
                end
            end
        end        
        
        
        
        
        
        
        

        %% Grid A* Code
        function newFutureData = gridAStar(obj, globalTime,futureData)
            %this function performs a A* search with the grid location
            %objects from obj.Map.gridLocationMap
            %futureData = [carID, coordinates of GL x, y, speed, time, probability]
            %globalTime is the current time of the simulation
            %newFutureData is the newly created FD from this vehicle
            
            %% prepare the search
            if isempty(futureData)
                futureData = [0 0 0 0 0 0];  
            else
                futureData = deleteCollidedFutureDataonGridForLoop(obj,futureData);
            end
            futureData = obj.detectBlockingCarsGridForLoop(futureData);
            %update temp goal if necessary
            if obj.tempGoalNode == obj.vehicle.pathInfo.lastWaypoint
                %if we are at our temp goal
                
                %we need to update oour goal
                obj.tempGoalNode = obj.vehicle.pathInfo.destinationPoint;
                %we have to repush our goal to the open list to search if
                %the path is still blocked
            end
            carID = obj.vehicle.id;
            %first we will set up our current position as a field
            openList = containers.Map();
            closedList = containers.Map();
            %add your location to the open list
            curPos = obj.vehicle.pathInfo.lastWaypoint;
            curPos = obj.Map.waypoints(curPos,:);
            curGridPos = obj.Map.bogMap.world2grid([curPos(1)-obj.Map.xOffset,-curPos(3)-obj.Map.yOffset]);
            curKey = append( num2str(curGridPos(1)),",",num2str(curGridPos(2)) );
            startKey = curKey;
            curGL = obj.Map.gridLocationMap(curKey);
            curGL.gValue = globalTime;
            curGL.speedVector(carID) = obj.vehicle.dynamics.speed;
            openList(curKey) = curGL;
            
            %get key of goal node
            curPos = obj.Map.waypoints(obj.tempGoalNode,:);
            goalPos = obj.Map.bogMap.world2grid([curPos(1)-obj.Map.xOffset,-curPos(3)-obj.Map.yOffset]);%TODO move tempgoal init to a new init function
            goalKey = append( num2str(goalPos(1)),",",num2str(goalPos(2)) );
            goalCoordinates = str2num(goalKey); %#ok<*ST2NM>
            
            %check acceleration
            obj.checkForAccelerationPhase();
            
            %% start searching
            while ~isempty(openList.keys)
                %as long as open list is not empty
                %search for min f in open list
                
                olKeys = openList.keys;
                curKey = olKeys{1};
                curGL = openList(curKey); %preassign first entry
                olKeys = olKeys(1,2:end);
                for k = olKeys
                    newGL = openList(k{1,1});
                    if newGL.fValue < curGL.fValue
                        curGL = newGL;
                        curKey = k{1,1};
                    end
                end
                %now remove the gl from the open list
                openList.remove(curKey);
                % calculate time over curGL
                [nextSpeed,travelTime] = obj.checkACCOnGrid(curGL,curGL.speedVector(carID));
                curGL = obj.correctTimeWithFD(carID,nextSpeed,obj.simSpeed,travelTime,curGL,futureData);
                %now we can push the current grid location to the closed list
                closedList(curKey) = curGL;  
                %% for every successor
                [successors,lastNodeNR] = obj.getSuccessors(curGL,curGL.edgeStart);
                %now check every successor
                for succKey = successors    
                    %test for block or closed list
                    if closedList.isKey(succKey) || obj.isSuccBlocked(succKey,futureData,curGL.gValue + travelTime)
                        %if the successor is already in the closed list or
                        %blocked we can ignore it
                        continue;
                    end 
                    succGL = obj.Map.gridLocationMap(succKey);
                    %% regular search step
                    %if we have yet to rech the goal, we continue the search
                    succGL = obj.getGLCosts(carID,curGL.speedVector(carID),curGL.timeVector(carID),obj.simSpeed,obj.vehicle.dynamics.maxSpeed,curGL.gValue,curGL.totalDistance,curGL.distance,succGL,goalCoordinates);
                    %% if we found the goal
                    if strcmp(succKey , goalKey)
                        %we found the goal node
                        %set parent
                        succGL.parent = curKey;
                        %mark as blocked
                        succGL.deviation = succGL.deviation * -1;
                        %push it to closed list
                        closedList(succKey) = succGL;
                        %build path
                        [newFutureData,newPath] = obj.gridBuildPath(carID, closedList, goalKey, startKey);
                        obj.vehicle.pathInfo.path = newPath;
                        
                        return;
                    end
                    %% test the rest                    
                    if openList.isKey(succKey) && openList(succKey).fValue <= succGL.fValue
                        %if we already have a shorter way to go on succGL,
                        %we dont need to push
                        continue;
                    else
                        %set parent
                        succGL.parent = curKey;
                        %set start of edge
                        succGL.edgeStart = lastNodeNR;
                        %add to open list
                        openList(succKey) = succGL;
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
            obj.tempGoalNode = obj.vehicle.pathInfo.destinationPoint;
        end
        function [newFD,newPath] = gridBuildPath(~, carID, closedList, goalKey, startKey)
            curGL = closedList(goalKey);
            curKey = goalKey;
            newPath = [];
            newFD = [];
            while ~strcmp(curKey,startKey)
                if curGL.parent == 0
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
                %FD [carID, coordinates of GL x, y, speed, time, probability]
                newFD = [newFD; [carID, curGL.coordinates, curGL.speedVector(carID), curGL.gValue, curGL.deviation]];
                %got to the parent
                curKey = curGL.parent;
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
        function futureData = deleteCollidedFutureDataonGridForLoop(obj,futureData)
            %deletes future data of vehicles that will not move because of collision
            %FD [carID, coordinates of GL x, y, speed, time, deviation]
            otherCars = 1:10;
            otherCars = otherCars(otherCars ~= obj.vehicle.id);
            vehicles = obj.Map.Vehicles;
            for car = otherCars
                if vehicles(car).status.collided
                    %remove every entry with the collided car from FD
                    futureData = futureData(futureData(:,1)~=car,:);
                    %% block the start node of the crash
                    coords = obj.Map.waypoints(vehicles(car).pathInfo.lastWaypoint,:);
                    coords = obj.Map.mapBOG.world2grid([coords(1)-obj.Map.xOffset,-coords(3)-obj.Map.yOffset]);                    
                    futureData = [futureData;[car , coords , 0, 0, -1]];
                    %% block the future node of the crash
                    coords = obj.Map.waypoints(vehicles(car).pathInfo.path(2),:);
                    coords = obj.Map.mapBOG.world2grid([coords(1)-obj.Map.xOffset,-coords(3)-obj.Map.yOffset]);
                    futureData = [futureData;[car , coords , 0, 0, -1]];
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
            otherCars = 1:10;
            otherCars = otherCars(otherCars ~= obj.vehicle.id);
            vehicles = obj.Map.Vehicles;
            for car = otherCars
                if vehicles(car).pathInfo.destinationReached                    
                    coords = obj.Map.waypoints(vehicles(car).pathInfo.lastWaypoint,:);
                    coords = obj.Map.bogMap.world2grid([coords(1)-obj.Map.xOffset,-coords(3)-obj.Map.yOffset]);
                    futureData = [futureData;[car , coords , 0, 0, -1]];
                end
            end
        end
        
        %% acceleration estimation related
        function checkForAccelerationPhase(obj)
            %used to calculate acceleration for future data estimation
            car = obj.vehicle;
            maxSpeed = car.dynamics.maxSpeed ;
            currentSpeed = obj.vehicle.dynamics.speed ;
            if abs(maxSpeed - currentSpeed) > 1
                if obj.accelerationPhase(1) == 0
                    % Neural Network is used to get average acceleration value
                    averageAcceleration = obj.simSpeed^2 * NN_acceleration([currentSpeed; maxSpeed-currentSpeed]);
                    accelerationDistance = obj.getAccelerationDistance(averageAcceleration, currentSpeed, maxSpeed);
                    obj.accelerationPhase = [1,currentSpeed,maxSpeed, accelerationDistance, averageAcceleration];
                end
            end
        end                
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
                    t2 =  (1/ obj.simSpeed) * (currentTotalDistance+ distance - accelerationDistance)/ currentGL.speedLimit;
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
        function manuallyChangeRoute(~,car)
            % Some experiment done for NecSys, could be reused in future if necessary
            
            if car.id ==3
                if car.pathInfo.lastWaypoint == 9
                    car.pathInfo.destinationPoint = 11;
                end
                
                if car.pathInfo.lastWaypoint == 10
                    %Fault injection
                    p = rand();
                    if p >0.5
                        %V3 - Route 51 / 1-p(example: for p>0.2 this %80)
                        car.pathInfo.destinationPoint = 33;
                        
                    else
                        %V3 - Route 5 / p
                        car.pathInfo.destinationPoint = 35;
                        
                    end
                    
                end
                
                if car.pathInfo.lastWaypoint == 13
                    car.pathInfo.destinationPoint = 31;
                elseif car.pathInfo.lastWaypoint == 32
                    car.pathInfo.destinationPoint = 31;
                end
            end
            
            if car.id == 1
                % Decision R5
                %                 if car.pathInfo.lastWaypoint == 9
                %                     car.pathInfo.destinationPoint = 13;
                %                 end
                %
                %                 if car.pathInfo.lastWaypoint == 12
                %                     car.pathInfo.destinationPoint = 26;
                %                 end
                
                
                %                 % Decision R51
                %                 if car.pathInfo.lastWaypoint == 9
                %                     car.pathInfo.destinationPoint = 33;
                %                 end
                %
                %                 if car.pathInfo.lastWaypoint == 32
                %                     car.pathInfo.destinationPoint = 26;
                %                 end
                
            end
            
        end
        
        
        function FuturePlan = findNextRoute(obj, car, starting_point, ending_point, global_timesteps,futureData)
            
            [path,newFutureData] = obj.findShortestPath(car, starting_point, ending_point, global_timesteps, futureData);
            
            car.pathInfo.path = path;           
            
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
        
        % Equation 5 & 8 in NecSys Paper
        function accelerationDistance = getAccelerationDistance(~, averageAcceleration, currentSpeed, speedTo)
            delta_v = speedTo-currentSpeed;
            accelerationDistance = delta_v^2/(2*averageAcceleration)+currentSpeed*delta_v/averageAcceleration;
        end
        % Eigenvalues of Equation 8 in NecSys Paper
        function timeToReach = timeToReachNextWaypointInAccelerationPhase(~, currentSpeed, averageAcceleration, distance)
            timeToReach = -currentSpeed/averageAcceleration + sqrt((currentSpeed/averageAcceleration)^2+2*distance/averageAcceleration);
        end
        
        function checkCrossroadActions(obj,car,starting_point,global_timesteps)    
            %% check for crossroad actions
            if find(any(obj.Map.crossroads.startingNodes==starting_point,2)) % car reaches crossroad
                crossroadId = find(any(obj.Map.crossroads.startingNodes==starting_point,2));
                car.decisionUnit.inCrossroad = [crossroadId 1];
                car.V2I.carReachesCrossroadV2I(car,starting_point,global_timesteps,crossroadId);
                                     
            elseif find(any(obj.Map.crossroads.breakingNodes==starting_point,2)) % car reaches Breaking Point
                crossroadId = find(any(obj.Map.crossroads.breakingNodes==starting_point,2));
                car.decisionUnit.inCrossroad = [crossroadId 2];
                car.V2I.carReachesBreakingPointV2I(car,starting_point,global_timesteps,crossroadId);
                
            elseif find(any(obj.Map.crossroads.stoppingNodes==starting_point,2)) % car reaches Stopping Point  
                crossroadId = find(any(obj.Map.crossroads.stoppingNodes==starting_point,2));
                car.decisionUnit.inCrossroad = [crossroadId 3];
                
            elseif find(any(obj.Map.crossroads.leavingNodes==starting_point,2)) % car leaves crossroad
                crossroadId = find(any(obj.Map.crossroads.leavingNodes==starting_point,2));
                car.V2I.carLeavesCrossroadV2I(car,global_timesteps,crossroadId);
                car.decisionUnit.inCrossroad = [0 0];
                
            end
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
            out = [2000 6];
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
end
