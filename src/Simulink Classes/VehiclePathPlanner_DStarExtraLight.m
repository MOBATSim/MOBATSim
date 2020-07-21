classdef VehiclePathPlanner_DStarExtraLight < matlab.System & handle & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
    % Path Planner Plans paths.
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
        initialFutureData;
        futureData;
        breakingFlag;
        inCrossroad; % [crossroadId crossroadZone]
        % crossroadZone:
        % 1 -> arrivingZone
        % 2 -> stoppingZone
        % 3 -> intersectionZone
        
        %variables for D Star Extra Lite
        
        km =0; %biased value for D*ELite
        
        nrOfNodes;              % number of all nodes on the map
        nodesKey;               % key for every node
        nodesG;                 % g value
        nodesOpen;              % if is in open list => 1
        nodesParent;            % nr of parent node for cutting
        nodesVisited;           % if visited by search() => 1
        nodesGlobalDistance;    % total sim distance for future data calc
        nodesBlocked;           % if node is blocked by a car => 1
        
        nrOfEdges;              % number of all edges on the map
        maxEdgeSpeed;           % the forced speed limit on this road
        edgesSpeed;             % the max speed caused by cars
        edgesCost;              % cost to move over this edge [time]
        edgesEntry;             % time when you enter this edge
        edgesExit;              % time when you exit this edge
        
        seeds;                  % seeds are nodes to reopen later in search()
        slast;                  % node to calculate new km
        haveCostsChanged = false;   % is there different future data than before
        oldFutureData =[0 0 0 0 0]; % to compare with new to get difference
        changedEdges = [];      % edges with new future data entries
        
        %end of D Star Extra Light variables
    end
    
    methods
        % Constructor
        function obj = Untitled(varargin)
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
            %This block shouldn't run if the vehicle has reached its
            %destination
            if( isempty(obj.nodesKey) )
                %only init once during first turn
                initialize(obj);
            end
            if obj.vehicle.pathInfo.destinationReached %check if already reached goal
                FuturePlan = obj.vehicle.decisionUnit.futureData;
                waypointReached=1;
            else
                % Firstly check if the vehicle has reached its destination so it stops.
                % Then check if the vehicle has completed its route but still needs to reach its destination
                if obj.vehicle.pathInfo.routeCompleted == 1 && ~obj.vehicle.checkifDestinationReached()
                    % Check if crossroad
                    obj.crossroadCheck(obj.vehicle);
                    % Time Stamps are logged when waypoints are reached
                    obj.vehicle.dataLog.timeStamps = [obj.vehicle.dataLog.timeStamps;[obj.vehicle.pathInfo.lastWaypoint get_param(obj.modelName,'SimulationTime')]];
                    
                    % Build the future plan by deriving the next routes and building the path
                    %Output 1: Future plan of the vehicle
                    FuturePlan = obj.dStarExtraLite(get_param(obj.modelName,'SimulationTime'),OtherVehiclesFutureData);
                    %check acc in pathbuilding!
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
        
        
        
        
        function newFutureData = dStarExtraLite(obj, globalTime,futureData)
            %check https://doi.org/10.1515/amcs-2017-0020 for more details
            %algorithm from  D* Extra Lite: A Dynamic A* With Search–Tree Cutting and Frontier–Gap Repairing by Maciej Przybylski
            %
            %newFutureData: [carID edgeNr speed enterTime exitTime]
            %obj: this
            %car: the car that uses this method to get new instructions            
            %globalTime: current simulation time
            %futureData: futureData of other vehicles
            %% Initialization
%             moved in deleteCollidedFD
%             if isempty(futureData)%workaround for first turn
%                 futureData = [0 0 0 0 0];
%             end
            
            futureData = deleteCollidedFutureDataForLoop(obj,futureData);
            
            whichEdgecostsChangedForLoop(obj,futureData);%TODO vectorize
            %obj.haveCostsChanged = false; Just to test performance without reinit
                        
            %% check for acceleration phase
            checkForAccelerationPhase(obj);
            
            %% Only Update if anything changed or first time
            if obj.haveCostsChanged || isempty(obj.vehicle.pathInfo.path)
                %% Reinititialize
                if obj.haveCostsChanged
                    reinitialize(obj,futureData);
                end
                % Use D*EL to get path to goal node
                if ( ~search(obj)) %Search for optimal path and alter lists accordingly
                    disp('Path not found error ')
                    disp(obj.vehicle.id)
                    stopVehicle(obj);
                    newFutureData = [];
                    return;
                else                   
                    
                    %if anything changed, we need to calculate a new path
                    newFutureData = calculateNewPath(obj,globalTime);
                                        
                end
                                
                if globalTime == 0 % save the future data at the beginning of the simulation for validation after simulation
                    obj.vehicle.decisionUnit.initialFutureData = newFutureData;
                end
            else
                %if we dont have to calculate we need to adjust path and new FD
                
                newFutureData = obj.vehicle.decisionUnit.futureData;%new = old FD
                obj.vehicle.pathInfo.path = obj.vehicle.pathInfo.path(2:end);%delete old node in path
            end
            
        end
        
        function stopVehicle(obj)
            car = obj.vehicle;            
            %code from vehicle.checkifDestinationReached
            car.pathInfo.path = []; %%TODO is this ok?
            car.pathInfo.destinationReached = true;
            car.setStopStatus(true);
            car.pathInfo.routeCompleted = true;
            car.dynamics.speed = 0;
            car.dataLog.totalTravelTime = get_param(car.modelName,'SimulationTime');
            car.V2VdataLink(car.V2VdataLink==1) =0;
        end
        
        function newFutureData = calculateNewPath(obj,globalTime)
            %we calculate the new path for our vehicle and create the
            %future data and new edge costs in the process
            %we need to calculate our entry and exit of every edge as well
            
            %preallocating
            newpath = zeros(obj.nrOfNodes);
            newFutureData = zeros(obj.nrOfEdges,5);
            
            id = obj.vehicle.id;
            sstart = obj.vehicle.pathInfo.lastWaypoint;
            sgoal = obj.vehicle.pathInfo.destinationPoint;            
            i = 1;     
            currentSpeed = obj.vehicle.dynamics.speed ;           
            
            time = globalTime;%time in the simulation
            %we need to reset the distance that we travelled so far, because
            %we want to calculate evrything like we would start the
            %simulation directly from last waypoint
            obj.nodesGlobalDistance(sstart) = 0;
            while sstart ~= sgoal
                %add next node to path
                newpath(i) = sstart; 
                oldstart = sstart;
                
                %get next node
                sstart = actionSelection(obj,sstart);
                %if for loop is better try 
                %sstart = actionSelectionForLoop(obj,sstart);
                
                %get edge between old and new
                curEdge = getEdge(obj,oldstart,sstart);
                
                %calculate times
                obj.edgesEntry(curEdge) = time;                
                [nextSpeed,timeToReach] = checkForAccelerationInPathbuilding(obj,curEdge,oldstart, currentSpeed);
                obj.edgesSpeed(curEdge) = nextSpeed;
                currentSpeed = nextSpeed;
                time1 = time;
                time = time + timeToReach;
                obj.edgesExit(curEdge) = time;
                
                %distance = old + new
                obj.nodesGlobalDistance(sstart) = obj.nodesGlobalDistance(oldstart)+ obj.Map.connections.distances(curEdge);
                %create FD entry
                newFutureData(i,:) =  [id curEdge nextSpeed time1 time];
                i = i + 1;
            end
            newpath(i) = sstart; %sgoal
            obj.vehicle.pathInfo.path = newpath(1:i);%cut preallocated vector
            newFutureData = newFutureData(1:i-1,:);
        end
        
        function pushOpen(obj,s,k)
            %if node s is not open, it inserts s to the openlist with key k
            %if node s is open, it updates the priority
            %if the node is blocked, we must not push it
            
            if obj.nodesBlocked(s)
                return;
            end
            if (obj.nodesOpen(s)==1)%if open
                if obj.nodesKey(s,1) ~= k(1) || obj.nodesKey(s,2) ~= k(2)
                    obj.nodesKey(s,:) = k; %update key
                end
            else%if not open
                obj.nodesOpen(s) = 1; %set open
                obj.nodesKey(s,:) = k; %update key
            end
            
            
        end
        
        function s = topOpen(obj)
            %returns the node with the lowest key in the open list;
            
            openList = find(obj.nodesOpen ==1);
            s = openList(1); %initialisation
            for i = 2:size(openList,1)
                knew = obj.nodesKey(openList(i),:);
                kold = obj.nodesKey(s,:);
                if knew(1) < kold(1) || (knew(1) == kold(1) && knew(2) < kold(2))
                    s = openList(i);
                end                
            end
        end

        function less = compareKeys(~,k1a,k1b,k2a,k2b) 
            %returns true if k(s1) < k(s2)
            
            less = false;
            if(k1a < k2a)
                less = true;
            elseif(k1a == k2a)
                if(k1b < k2b)
                    less = true;
                end
            end
        end
        
        function nextNode = actionSelection(obj,sstart)
            % return arg min( s in Succ(sstart)(cost(sstart,s)+g(s)) )
           
            indx = obj.Map.connections.all(:,1)== sstart; 
            successors = obj.Map.connections.all(indx,2);
            cost = obj.edgesCost(indx);
            G = obj.nodesG(successors);
            [~,indx] = min(G+cost);
            nextNode = successors(indx);
        end
        
        function nextNode = actionSelectionForLoop(obj,sstart)
            % return arg min( su in Succ(sstart)(cost(sstart,su)+g(su)) )
            
            %get Successors
            su = succ(obj,sstart);
            %get lowest key
            nextNode = su(1);           
            key = obj.edgesCost(getEdge(obj,sstart,nextNode))+obj.nodesG(nextNode);
            for i = 2:length(su)
                key2 = obj.edgesCost(getEdge(obj,sstart,su(i)))+obj.nodesG(su(i));
                if (key2 < key)
                    nextNode = su(i);
                    key = key2;
                end                
            end
        end
        
        function solution = search(obj)
            %a normal A* algorithm
            %first turn, the goal node is the only entry in open list
            while fastMember(obj,1,obj.nodesOpen)  %as long as open List is not empty
                s = topOpen(obj);
                if solutionFound(obj,s)%search for a possible path
                    solution = true;
                    return;
                end
                %if we have not reached the start yet, we go to the node
                %with the lowest key in the open list
                searchStep(obj,s);
            end
            solution = false; %no possble path was found
        end
        
        function foundSmth = solutionFound(obj,topOpen)
            %return (TOPOPEN()== sstart || (visited(sstart) && NOT open(sstart)))
            sstart = obj.vehicle.pathInfo.lastWaypoint;
            foundSmth = (topOpen == sstart || (obj.nodesOpen(sstart)~=1 &&  (obj.nodesVisited(sstart) == 1) ));
            %we have found a way from goal to start or are still on the way
        end
        
        function searchStep(obj,s)
            %popOpen
            obj.nodesOpen(s) = 0;%delete s from open
            %compare old and new key
            kold = obj.nodesKey(s,:);
            knew = calculateKey(obj,s);
            %is key still correct?
            if (compareKeys(obj,kold(1),kold(2),knew(1),knew(2)))
                %if the old key was lower, we need to update and reexplore s
                pushOpen(obj,s,knew);
            else
                %determine predecessors of s
                
                pre = pred(obj,s);                
                for sz = pre %for every predecessor of s
                    %if NOT visited(sz)OR g(sz)>cost(sz,s)+g(s) (g(sz) to high)
                    costs = obj.edgesCost(getEdge(obj,sz,s)) +obj.nodesG(s);
                    if ((obj.nodesVisited(sz) ~= 1) || obj.nodesG(sz) > costs)
                        %parent(s′)=s to be able to cut branches later on
                        obj.nodesParent(sz) = s;
                        %g(sz)=cost(sz,s)+g(s)
                        obj.nodesG(sz) = costs;
                        %if NOT visited(sz)then
                        if( (obj.nodesVisited(sz) ~= 1) )
                            %visited(s′)=true
                            obj.nodesVisited(sz) = 1;
                        end
                        pushOpen(obj,sz,calculateKey(obj,sz))%Add to open
                    end
                end
            end
        end
        
        function key = calculateKey(obj,s)
            %return[g(s)+h(sstart,s)+km;g(s)]
            gn = obj.nodesG(s);
            key = [gn+h(obj,s,obj.vehicle.pathInfo.lastWaypoint)+obj.km,gn];
        end
        
        function h = h(obj,start,goal)
            %time travel euclidian distance
            %h is time, to make it comparable with edge time(cost)
            h = (1/ obj.simSpeed) * (1/obj.vehicle.dynamics.maxSpeed) * norm(get_coordinates_from_waypoint(obj.Map, start)-get_coordinates_from_waypoint(obj.Map, goal));
        end
        
        function initialize(obj)
            %initialize whole map
            lMap = length(obj.Map.waypoints);
            obj.nrOfNodes = lMap;
            %nodes
            obj.nodesKey = zeros(lMap,2);
            obj.nodesOpen = zeros(lMap,1);
            obj.nodesParent = zeros(lMap,1);
            obj.nodesVisited = zeros(lMap,1);
            obj.nodesGlobalDistance = zeros(lMap,1);
            obj.nodesG = ones(lMap,1)*2000000000;%intmax
            obj.nodesBlocked = zeros(lMap,1);
            %edges
            nMap = length(obj.Map.connections.all);
            obj.nrOfEdges = nMap;
            obj.edgesSpeed = zeros(nMap,1);
            obj.edgesEntry = zeros(nMap,1);
            obj.edgesExit = zeros(nMap,1);
            obj.seeds = zeros(nMap,1);%if you change the init, change other code too (reinitialize)!
            
            %INIT Cost
            
            maxSpeed = obj.vehicle.dynamics.maxSpeed ;
            speedRoutes = [obj.Map.connections.circle(:,end);obj.Map.connections.translation(:,end)];
            speedRoutes(speedRoutes>maxSpeed)= maxSpeed; %possible speed for every route
            obj.maxEdgeSpeed = speedRoutes;
            distances = obj.Map.connections.distances;
            %calculate costs
            obj.edgesCost = (1/ obj.simSpeed) .* distances .* (1./ speedRoutes);
                        
            %rest
            %visited(sgoal)=true
            sgoal = obj.vehicle.pathInfo.destinationPoint;
            obj.nodesVisited(sgoal) = 1;
            %g(sgoal)=0
            obj.nodesG(sgoal) = 0;
            pushOpen(obj,sgoal,calculateKey(obj,sgoal));
            obj.vehicle.pathInfo.path = [];            
            obj.slast = obj.vehicle.pathInfo.lastWaypoint;
            
        end   
        
        function newCost = updateEdgeCost(obj,curEdge,futureData)
            %this function returns the new cost of an edge, after its costs
            %have changed due to future data
            %we also check, if a car now blocks this path
            %keep in mind, that if we want to calculate our future position
            %correctly, you have to search from start to goal, or find a
            %new way to calculate entry and exit times for future data
            
            %futureData = [carID edgeID Speed enterTime exitTime]            
            
            curNode = getEndOfEdge(obj,curEdge);
            nextSpeed = obj.edgesSpeed(curEdge);
            
            % now check, if a car will stop here and block the road
            %we need all entries with cur edge in cFD
            currentFutureData = futureData(futureData(:,2) == curEdge,:);            
            
            %we have to check the other edge of crossroad
            currentEntryTime = max(obj.edgesEntry(ePred(obj,curNode)));
            currentExitTime = max(obj.edgesExit(ePred(obj,curNode)));
            %if you entered before us or exit before us, you can block
            currentFutureData = currentFutureData(currentFutureData(:,4)<= currentEntryTime | currentFutureData(:,5) <= currentExitTime,:);
            %check for other cars            
            otherCars = fastUnique(obj,currentFutureData(:,1));
            
            %test, if road is blocked
            if ~isempty(otherCars)
                blocked = checkIfBlocked(obj,otherCars,curEdge);
            else
                blocked = false;
            end
            
            if blocked
                
                newCost = 2147400000;% blocked cost (int max value)
                nextSpeed = 0;%max speed
                obj.nodesBlocked(curNode) = 1;
            else
                %a car is disturbing, if it exits after us, but entered before us
                currentFutureData = currentFutureData(currentFutureData(:,5)>currentEntryTime,:);
                if( ~isempty(currentFutureData) )                    
                        %% disturbing car on same route
                        nextSpeed = min(currentFutureData(:,3));
                        distance = obj.Map.connections.distances(curEdge);
                        newCost = (1/ obj.simSpeed) * distance * (1/ nextSpeed);
                else
                    %no disturbing car means we can reuse old cost
                    newCost = obj.edgesCost(curEdge);
                end
                %if it is not blocked, we can reset status
                obj.nodesBlocked(curNode) = 0;
            end
            
            %% calculate costs (costs = distance/speed)
            obj.edgesCost(curEdge) = newCost;%cost, gets returned
            obj.edgesSpeed(curEdge) = nextSpeed;%max speed
            
        end
        
        function blocked = checkIfBlocked(obj,otherCars, curEdge)
            %if a car will stop after current edge, it will block the node
            %if a car has stopped there, it is already blocked
            pathInfo = [obj.vehicle.map.Vehicles.pathInfo];            
            %% stopping cars
            futurePathInfo = pathInfo(otherCars);
            destinationPoints = [futurePathInfo.destinationPoint];
            endOfCurEdge = obj.Map.connections.all(curEdge,2);
            %check if the any car wants to stop after the edge
            willBeBlocked = fastMember(obj,endOfCurEdge,destinationPoints);
            %% stopped cars             
            reached = [pathInfo.destinationReached];
            if ~isempty(otherCars)
                destinationPoints = [pathInfo(reached).destinationPoint];
                %check if a car has stopped after the edge
                alreadyBlocked = fastMember(obj,endOfCurEdge,destinationPoints);
            else
                alreadyBlocked = false;
            end
            
            blocked = willBeBlocked || alreadyBlocked;
        end
        
                
        function blocked = checkIfBlockedForLoop(obj,otherCars, curEdge)
            %if a car will stop after current edge, it will block the node
            for c = otherCars                
                roads = ePred(obj,obj.vehicle.map.Vehicles(c).pathInfo.destinationPoint);
                blocked = fastMember(obj,1,fastMember(obj,roads,curEdge));
                if blocked
                    break;
                end
            end
        end
        
        function reinitialize(obj,futureData)            
                %if any edge cost changed
                %remove every node from our storage, that has a wrong key
                cutBranches(obj,futureData);
                %as long as cutBranches found some edges to reopen
                if ( fastMember(obj,1,obj.seeds) )
                    %update biased value to repair old and now wrong h values
                    obj.km = obj.km + h(obj,obj.slast,obj.vehicle.pathInfo.lastWaypoint);
                    obj.slast = obj.vehicle.pathInfo.lastWaypoint;
                    seed = find(obj.seeds == 1)';
                    for s = seed
                        % if s is part of our explored map, but was already
                        % removed from the open list, we need to reopen it
                        % after it was cut 
                        if ((obj.nodesVisited(s) == 1) && obj.nodesOpen(s)~=1)
                            pushOpen(obj,s,calculateKey(obj,s));
                        end
                    end
                    %reset seeds
                    obj.seeds = zeros(obj.nrOfEdges,1);
                end
            
        end
        
        function whichEdgecostsChanged(obj,futureData)
            %check, which edgecosts have changed and return those edges
            
            %substract smaller from bigger to get difference
            if(size(obj.oldFutureData,1) < size(futureData,1))
                S = obj.oldFutureData;
                B = futureData;
            else
                S= futureData;
                B = obj.oldFutureData;
            end
            
            %store old future data
            obj.oldFutureData = futureData;
            
            %get difference
            B = setdiff(B,S,'rows');%TODO to slow

            %B is now all rows that are changes
            
            %if we have something found, we need to update the map later
            if isempty(B)
                obj.haveCostsChanged = false;
            else
                obj.changedEdges = fastUnique(obj,B(:,2));
                obj.haveCostsChanged = true;
            end
        end
        function whichEdgecostsChangedForLoop(obj,futureData)
            %check, which edgecosts have changed and return those edges
            
                    %substract smaller from bigger to get difference
                    if(size(obj.oldFutureData,1) < size(futureData,1))
                        S = obj.oldFutureData;
                        B = futureData;
                    else
                        S= futureData;
                        B = obj.oldFutureData;
                    end 
                    
                    %store old future data
                    obj.oldFutureData = futureData;
                    
            
                for i = 1:size(S,1)%compare smaller vector to bigger, to see what changed
                    index = find(B(:,1) == S(i,1) & B(:,2) == S(i,2) & B(:,4) == S(i,4) );%compare id, edge and entry time
                    if( ~isempty(index) )%check if row exists in both fd
                        B(index,:) = [];% %if so, it hasnt changed -> delete it
                    end
                end
                %B is now all rows that are changes
            
            %if we have something found, we need to update the map later
            if isempty(B)
                obj.haveCostsChanged = false;
            else
                obj.changedEdges = fastUnique(obj,B(:,2));
                obj.haveCostsChanged = true;
            end
        end
        
        
        function cutBranches(obj,futureData)
            %resets every node with changed edge costs
            sstart = obj.vehicle.pathInfo.lastWaypoint;
            reopen_start=false;
            %for all directed edges(u, v)with changed cost
            for s = obj.changedEdges
                u = getStartOfEdge(obj,s);%start of edge
                v = getEndOfEdge(obj,s);%end of edge
                %if we visited nodes through this edge, we need to recalculate
                if ((obj.nodesVisited(u) == 1) && (obj.nodesVisited(v) == 1))
                    cold = obj.edgesCost(s);
                    cnew = updateEdgeCost(obj,s,futureData);
                    if (cold > cnew )                        
                        if (obj.nodesG(sstart)>obj.nodesG(v)+ cnew + h(obj,sstart,u))
                            %if an edge decreased in a way, that
                            %actionSelection() will automatically find the
                            %best possible way, we dont have to delete the
                            %wrong way and can just try to use the second
                            %solutionFound() condition
                            reopen_start=true;
                        end
                        obj.seeds(v) = 1;
                    elseif (cold<cnew)%if costs increased we will mark edge as unvisited (cut from expl. branch)
                        %if we moved over the changed edge, values of all nodes in this branch are wrong                        
                            cutBranch(obj,v);%TODO parent?                        
                    end
                end
            end
            if (reopen_start && (obj.nodesVisited(sstart) == 1))
                obj.seeds(sstart) = 1;
            end
        end
        
        function cutBranch(obj,s)
            %reset node as unvisited
            obj.nodesVisited(s)=0;
            obj.nodesG(s) = 2000000000;
            %reset parents
            obj.nodesParent(s) = 0;
            %wrong nodes cant be in open list until they where updated
            obj.nodesOpen(s) = 0;
            %for all successors
            suc = succ(obj,s);
            for sz = suc
                %every node closer to the goal may need a reopening after
                %cutting
                if ((obj.nodesVisited(sz) == 1) && obj.nodesParent(sz) ~= s)
                    %all nodes between s and the goal, that lead backwards to the goal
                    %are now the last node before a possible blocking on s
                    %they need to be reopened to check, how bad the block is
                    %( parent ~= only possible if we can go both ways on edges) 
                    
                    obj.seeds(sz) = 1;
                end
            end
            %for all predecessors
            pre = pred(obj,s);
            for sx = pre
                %if the cost between u and v increased, and we add costs from
                %goal to start for a key of the node, every node between s and our current
                %position might have an underestimated key, so we have to
                %treat it as unexplored
                if ((obj.nodesVisited(sx) == 1) && obj.nodesParent(sx) ==s)
                    cutBranch(obj,sx);
                end
            end
        end
        
        function pre = pred(obj,s)
            %returns all predecessors of s
            indx = obj.Map.connections.all(:,2)== s;
            pre = obj.Map.connections.all(indx,1)';
        end
        function predE = ePred(obj,s)
            %returns all edges to predecessors of s
            predE = find(obj.Map.connections.all(:,2)== s)';
        end
        
        function su = succ(obj,s)
            %returns all successors of s
            indx = obj.Map.connections.all(:,1)== s;
            su = obj.Map.connections.all(indx,2)';
        end
        function succE = eSucc(obj,s)
            %returns all edges to successors of s
            succE = find(obj.Map.connections.all(:,1)== s)';
        end
        
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
        
        function [nextSpeed,timeToReach] = checkForAccelerationInPathbuilding(obj,currentRoute,currentNode,currentSpeed)
            %nextSpeed = speed on end of edge, timeToReach = exit time of edge
            
            obj.vehicle.pathInfo.currentRoute = currentRoute;%setUp calculation 
            distance = obj.Map.connections.distances(currentRoute);
            
            currentMaxSpeedRoutes = obj.maxEdgeSpeed;
            %% check for acceleration phase
            if obj.accelerationPhase(1) == 1
                accelerationDistance = obj.accelerationPhase(4);
                averageAcceleration = obj.accelerationPhase(5);
                %global distance
                currentTotalDistance = obj.nodesGlobalDistance(currentNode);
                if ((currentTotalDistance + distance - accelerationDistance) < 0)
                    % whole route in acceleration phase
                    timeToReach = (1/ obj.simSpeed) * (-currentSpeed/averageAcceleration + sqrt((currentSpeed/averageAcceleration)^2+2*distance/averageAcceleration));
                    nextSpeed = currentSpeed + averageAcceleration*timeToReach * obj.simSpeed;
                else
                    % route is divided in acceleration phase (t1)
                    % and constant speed phase (t2)
                    t1 = (1/ obj.simSpeed) * (-currentSpeed/averageAcceleration + sqrt((currentSpeed/averageAcceleration)^2+2*(accelerationDistance - currentTotalDistance)/averageAcceleration));
                    t2 =  (1/ obj.simSpeed) * (currentTotalDistance+ distance - accelerationDistance)/ currentMaxSpeedRoutes(currentRoute);
                    timeToReach = t1+t2;
                    nextSpeed = obj.accelerationPhase(3);
                    obj.accelerationPhase = zeros(1,5); %set acceleration phase to zero
                    
                end
                
            else
                timeToReach =  (1/ obj.simSpeed) * distance* (1/ currentMaxSpeedRoutes(currentRoute)); %timesteps to reach neighbour
                nextSpeed = currentSpeed;
            end
        end
        
        function edge = getEdge(obj,start,goal)
            %returns edge between start and goal
            edge = find(obj.Map.connections.all(:,1)==start & obj.Map.connections.all(:,2)==goal);
            if(isempty(edge))%try the other way around
                edge = find(obj.Map.connections.all(:,1)==goal & obj.Map.connections.all(:,2)==start);
            end
        end
        function start = getStartOfEdge(obj,edge)
            %returns starting node of edge
            start = obj.Map.connections.all(edge,1);
        end
        function ende = getEndOfEdge(obj,edge)
            %returns end-node of edge
            ende = obj.Map.connections.all(edge,2);
        end
        
        function u = fastUnique(~,A)
            %returns an array with unique numbers from array A
            u = zeros(length(A));
            x = 0;
            for i = 1:length(A)
                contained = false;
                for j = 1:length(u)
                    contained = A(i) == u(j);
                    if contained
                        break;
                    end
                end
                if ~contained
                    x = x + 1;
                    u(x) = A(i);
                end
            end
            u = u(1:x);
        end        
        function m = fastMember(~,test, A)
            %returns true if test is member of A
            m = false;
            for i = 1: length(A)
                m = A(i) == test;
                if m
                    break;
                end
            end
        end
        
        function detectBlockingCarsForLoop(obj)
            %if a car without future data blocks a node, we set the node blocked
            
            %get all other cars
            otherCars = getNrOfAllOtherCars(obj); 
            vehicles = obj.vehicle.map.Vehicles;
            for car = otherCars
                if vehicles(car).pathInfo.destinationReached
                    obj.nodesBlocked(vehicles(car).pathInfo.destinationPoint)=1;
                end
            end
        end
        
        function detectBlockingCars(obj)
            otherCars = getNrOfAllOtherCars(obj);
            vehicles = obj.vehicle.map.Vehicles(otherCars);
            pathInfo = [vehicles.pathInfo];
            reached = [pathInfo.destinationReached];
            otherCars = otherCars(reached);
            destinationPoints = [pathInfo(otherCars).destinationPoint];
            obj.nodesBlocked(destinationPoints) = 1;            
        end
        
        function futureData = deleteCollidedFutureDataForLoop(obj,futureData)
            %deletes future data of vehicles that will not move because of collision
            
            otherCars = getNrOfAllOtherCars(obj); 
            vehicles = obj.vehicle.map.Vehicles;
            for car = otherCars
                if vehicles(car).status.collided
                    %remove every entry with the collided car from FD
                    futureData = futureData(futureData(:,1)~=car,:);
                end
            end
            if isempty(futureData)%workaround for first turn
                futureData = [0 0 0 0 0];
            end
        end
        
        function otherCars = getNrOfAllOtherCars(obj)
            %returns a vector with all other cars id
            
            otherCars = 1:10;
            otherCars = otherCars(otherCars~= obj.vehicle.id);
        end
        
        function newFutureData = shortestPathFinder(obj, globalTime)
            %returns the shortest path based on digraph
            %no FD was usedto calculate it
            %we only need to calculate turn one, because we wont make any
            %changes to the edge costs
                                            
            if globalTime == 0 
                checkForAccelerationPhase(obj);
                if ( ~search(obj)) %Search for optimal path and alter lists accordingly
                    disp('Path not found error ')
                    disp(obj.vehicle.id)
                    stopVehicle(obj);
                    newFutureData = [];
                    return;
                else
                    newFutureData = calculateNewPath(obj,globalTime);
                end
                % save the future data at the beginning of the simulation for validation after simulation
                obj.vehicle.decisionUnit.initialFutureData = newFutureData;
            else
                %just return the old data
                newFutureData = obj.vehicle.decisionUnit.futureData;%new = old FD
                obj.vehicle.pathInfo.path = obj.vehicle.pathInfo.path(2:end);%delete old node in path
            end
        end
        
        %% Old Code
        
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
            %newFutureData = [carID edge speed entryTime exitTime]
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
                neighbourNodes = connections(find(connections(:,1) == currentNode),2); % waypointID of neighbour
                neighbourNodes_Routes = [neighbourNodes'; routes2neighbourNode'];
                
                
                
                %% loop over all neighbours
                for neighbourNode_Route=neighbourNodes_Routes
                    
                    
                    neighbourWP = waypoints(neighbourNode_Route(1),:); % waypointID of neighbour
                    currentTime = waypoints(currentNode,5);
                    currentTotalDistance = waypoints(currentNode,7);
                    currentSpeed = waypoints(currentNode,4);
                    currentRoute = neighbourNode_Route(2); % route ID
                    car.pathInfo.currentRoute = currentRoute;
                    
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
            newFutureData = [];
            for i = 1: (length(path)-1)
                
                newFutureData =  [newFutureData; [car.id waypoints(path(i+1),3) waypoints(path(i+1),4) waypoints(path(i),5)  waypoints(path(i+1),5)]];
                
            end
            
            if global_timesteps == 0 % save the future data at the beginning of the simulation for validation after simulation
                car.decisionUnit.initialFutureData = newFutureData;
            end
            
            
        end
        
        function path = composePath(~,waypoints, startingPoint, endingPoint)
            %% define path from waypoints array
            predecessor = waypoints(endingPoint,2);
            i = 2;
            path = 0;
            path(1) = endingPoint;
            try
                while (predecessor ~= startingPoint)
                    path(i)= predecessor; % TODO memory allocation
                    i = i + 1;
                    predecessor = waypoints(predecessor,2);
                    
                end
            catch
                disp('Path not found error')
            end
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
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            
            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);
            
            % Set private and protected properties
            %s.myproperty = obj.myproperty;
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            
            % Set private and protected properties
            % obj.myproperty = s.myproperty;
            
            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
        
        function ds = getDiscreteStateImpl(obj)
            % Return structure of properties with DiscreteState attribute
            ds = struct([]);
        end
        
        function flag = isInputSizeLockedImpl(obj,index)
            % Return true if input size is not allowed to change while
            % system is running
            flag = false;
        end
        
        function [out,out2] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [50 5];
            out2 = [1 1];
            
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
        
        function [out,out2] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = 'double';
            out2 = 'double';
            
            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end
        
        function [out,out2] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
        
        function [out,out2] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = false;
            out2 = true;
            
            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
        
        function icon = getIconImpl(obj)
            % Return text as string or cell array of strings for the System
            % block icon
            icon = mfilename('class'); % Use class name
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
    end
end
