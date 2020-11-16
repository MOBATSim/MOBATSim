classdef VehiclePathPlanner_DStarExtraLite < VehiclePathPlanner
    % Path Planner Plans paths.         
    %
    % NOTE: When renaming the class name Untitled, the file name
    % and constructor name must be updated to use the class name.
    %
    % This template includes most, but not all, possible properties, attributes,
    % and methods that you can implement for a System object in Simulink.
    
    % Pre-computed constants
    properties(Access = private)
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
        tempGoalNode;
    end
    
    methods
        % Constructor
        function obj = VehiclePathPlanner_DStarExtraLite(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        
        
        %% Common functions
        function setupImpl(obj)
            setupImpl@VehiclePathPlanner(obj); % Inherit the setupImpl function of the Superclass @VehiclePathPlanner
            %initialize whole map
            nodesMap = length(obj.Map.waypoints);
            obj.nrOfNodes = nodesMap;
            %nodes
            obj.nodesKey = zeros(nodesMap,2);
            obj.nodesOpen = zeros(nodesMap,1);
            obj.nodesParent = zeros(nodesMap,1);
            obj.nodesVisited = zeros(nodesMap,1);
            obj.nodesGlobalDistance = zeros(nodesMap,1);
            obj.nodesG = ones(nodesMap,1)*2000000000;%intmax
            obj.nodesBlocked = zeros(nodesMap,1);
            %edges
            edgesMap = length(obj.Map.connections.all);
            obj.nrOfEdges = edgesMap;
            obj.edgesSpeed = zeros(edgesMap,1);
            obj.edgesEntry = zeros(edgesMap,1);
            obj.edgesExit = zeros(edgesMap,1);
            obj.seeds = zeros(edgesMap,1);%if you change the init, change other code too (reinitialize)!
            
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
            obj.vehicle.setPath([]);
            obj.slast = obj.vehicle.pathInfo.lastWaypoint;
            %set up backup goal node
            obj.tempGoalNode = sgoal;
        end
        
           
        %% D* Extra Light
        function newFutureData = dStarExtraLite(obj, globalTime,futureData)
            %% Check https://doi.org/10.1515/amcs-2017-0020 for more details
            % Algorithm from  D* Extra Lite: A Dynamic A* With Search–Tree Cutting and Frontier–Gap Repairing by Maciej Przybylski
            %
            %newFutureData: [carID edgeNr speed enterTime exitTime]
            %obj: this
            %car: the car that uses this method to get new instructions            
            %globalTime: current simulation time
            %futureData: futureData of other vehicles
            
            %% Initialization
            if obj.checkTempGoalReached()
                obj.setTempGoaltoDestination();
            end
                       
            %%vectorized, but slower for now
            futureData = deleteCollidedVehicleFutureData(obj,futureData);
            
            %in the first round all vehicles have stop status, so only do it later
            obj.detectBlockingCarsForLoop(globalTime);
            
            whichEdgecostsChangedForLoop(obj,futureData);%TODO vectorize
            
            %% check for acceleration phase
            if obj.checkforAccelerationPhase(obj.vehicle.dynamics.speed,obj.vehicle.dynamics.maxSpeed)
                obj.accelerationPhase = setAccelerationPhase(obj,obj.vehicle.dynamics.speed,obj.vehicle.dynamics.maxSpeed);
            end
            
            %% Only Update if anything changed or first time
            if obj.haveCostsChanged || isempty(obj.vehicle.pathInfo.path)
                %% Reinititialize
                if obj.haveCostsChanged
                    reinitialize(obj,futureData);
                end
                % Use D*EL to get path to goal node
                if ( ~searchForGoalNode(obj)) %Search for optimal path and alter lists accordingly
                    %test if we can reach a closer node at least
                    newFutureData = newGoalNode(obj);
                    if isempty(newFutureData)
                        %we cant reach any node from now
                        disp(['No possible path was found for Vehicle ' num2str(obj.vehicle.id)])
                        stopVehicle(obj); % TODO: This should be removed and vehicle path should not be pruned!!!
                        return;
                    end
                else                   
                    
                    %if anything changed, we need to calculate a new path
                    %and future data
                    newFutureData = calculateNewPath(obj,globalTime);
                                        
                end
                                
                if globalTime == 0 % save the future data at the beginning of the simulation for validation after simulation
                    obj.vehicle.decisionUnit.initialFutureData = newFutureData;
                end
            else
                %if we dont have to calculate we need to adjust path and new FD
                
                newFutureData = obj.vehicle.decisionUnit.futureData;%new = old FD
                obj.vehicle.setPath(obj.vehicle.pathInfo.path(2:end));%delete old node in path
            end
            
        end        

        
        %% vehicle commands
        function stopVehicle(obj)
            car = obj.vehicle;            
            %code from vehicle.checkifDestinationReached
            car.setPath([]);
            car.pathInfo.destinationReached = true;
            car.setStopStatus(true);
            car.pathInfo.routeCompleted = true;
            car.dynamics.speed = 0;
            car.dataLog.totalTravelTime = get_param(car.modelName,'SimulationTime');
            car.V2VdataLink(car.V2VdataLink==1) =0;
        end
        
        %% open list related
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
        
        %% path building related
        function newFutureData = calculateNewPath(obj,globalTime)
            %we calculate the new path for our vehicle and create the
            %future data and new edge costs in the process
            %we need to calculate our entry and exit of every edge as well
            
            %preallocating
            newpath = zeros(obj.nrOfNodes);
            newFutureData = zeros(obj.nrOfEdges,5);
            
            id = obj.vehicle.id;
            sstart = obj.vehicle.pathInfo.lastWaypoint;
            sgoal = obj.tempGoalNode;            
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
            obj.vehicle.setPath(newpath(1:i));%cut preallocated vector
            newFutureData = newFutureData(1:i-1,:);
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
        
        %% search related
        function solution = searchForGoalNode(obj)
            %a normal A* algorithm
            %first turn, the goal node is the only entry in open list
            while fastMember(obj,1,obj.nodesOpen)  %as long as open List is not empty
                s = obj.topOpen();
                if obj.solutionFound(s)%search for a possible path
                    solution = true;
                    return;
                end
                %if we have not reached the start yet, we go to the node
                %with the lowest key in the open list
                obj.searchStep(s);
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
        
        %% reinitialize related
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
        function cutBranches(obj,futureData)
            %resets every node with changed edge costs
            sstart = obj.vehicle.pathInfo.lastWaypoint;
            reopen_start=false;
            %for all directed edges(u, v)with changed cost
            for s = obj.changedEdges
                u = getStartOfEdge(obj,s);%start of edge
                v = getEndOfEdge(obj,s);%end of edge
                %if we visited nodes through this edge, we need to recalculate
                if obj.nodesVisited(v) == 1 %(obj.nodesVisited(u) == 1) || 
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
                            cutBranch(obj,u);                      
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
        
        %% map information related
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
                blocked = obj.nodesBlocked(curNode);
            end
            
            if blocked
                
                newCost = 2147400000;% blocked cost (int max value)
                nextSpeed = 0;%max speed
                obj.nodesBlocked(curNode) = 1;
            else
                %unblock the node
                obj.nodesBlocked(curNode) = 0;
                %a car is disturbing, if it exits after us, but entered before us
                currentFutureData = currentFutureData(currentFutureData(:,5)>currentEntryTime,:);
                if( ~isempty(currentFutureData) )                    
                        %% disturbing car on same route
                        
                        %use this instead of the other code, if the
                        %inaccurate time estimation causes problems
%                         nextSpeed = min(currentFutureData(:,3));
%                         %if we are slower, we keep our speed
%                         if nextSpeed > obj.vehicle.dynamics.maxSpeed
%                             nextSpeed = obj.maxEdgeSpeed(curEdge);
%                         end
%                         distance = obj.Map.connections.distances(curEdge);
%                         newCost = (1/ obj.simSpeed) * distance * (1/ nextSpeed);

                        [timeToReachDisturbingVehicle ,index] = max(currentFutureData(:,5));                       
                        speedDisturbingVehicle =  currentFutureData(index,3);
                        timeDifference = (currentEntryTime + obj.edgesCost(curEdge)) - timeToReachDisturbingVehicle ;
                        
                        spacingTime = 6 * 1/obj.simSpeed;
                        if (timeDifference < spacingTime)
                            newCost = timeToReachDisturbingVehicle + spacingTime - currentEntryTime;
                            nextSpeed = speedDisturbingVehicle;
                        else
                            newCost = obj.edgesCost(curEdge);
                        end
                        %change comment in/out code above to fix some
                        %errors
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
        
        %% utility functions
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
        
        function u = fastUnique(~,A)
            % Returns an array with unique numbers from array A without sorting
            % Equivalent to built-in << unique(A,'stable') >> but much faster.
            
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
        
        %% blocking nodes
        function detectBlockingCarsForLoop(obj,globalTime)
            if globalTime ~= 0
                %if a car without future data blocks a node, we set the node blocked
                
                %get all other cars
                otherCars = getNrOfAllOtherCars(obj);
                vehicles = obj.vehicle.map.Vehicles;
                for car = otherCars
                    %check for stop
                    if vehicles(car).status.stop
                        obj.nodesBlocked(vehicles(car).pathInfo.lastWaypoint)=1;
                    end
                end
            end
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
        
        %% edit and evaluate FutureData        
        function futureData = deleteCollidedVehicleFutureData(obj,futureData)
            %deletes future data of vehicles that will not move because of collision
            otherCars = getNrOfAllOtherCars(obj); 
            vehicles = obj.vehicle.map.Vehicles;
            obj.changedEdges = [];%reset
            obj.haveCostsChanged = false;
            for car = otherCars
                %check for collision
                if vehicles(car).status.collided
                    %remove every entry with the collided car from FD
                    futureData = futureData(futureData(:,1)~=car,:);
                    %block the start and the future node of the crash
                    area = [vehicles(car).pathInfo.lastWaypoint,vehicles(car).pathInfo.path(2)];
                    obj.nodesBlocked(area(1))=1;
                    obj.nodesBlocked(area(2))=1;
                    obj.changedEdges = [obj.changedEdges, obj.getEdge(area(1),area(2))];
                    obj.haveCostsChanged = true;
                end                  
            end
            
            
            if isempty(futureData)%workaround for first turn
                futureData = [0 0 0 0 0];
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
            if ~isempty(B)                
                obj.changedEdges = [obj.changedEdges,fastUnique(obj,B(:,2))];
                obj.haveCostsChanged = true;
            end
        end
        
        %% get information on other cars in the simulation
        function otherCars = getNrOfAllOtherCars(obj)
            %returns a vector with all other cars id
            
            otherCars = 1:10;
            otherCars = otherCars(otherCars~= obj.vehicle.id);
        end
        
        %% alternative D* related pathfinding
        function newFutureData = shortestPathFinder(obj, globalTime)
            %returns the shortest path based on digraph
            %no FD was usedto calculate it
            %we only need to calculate turn one, because we wont make any
            %changes to the edge costs
                                            
            if globalTime == 0 
                checkForAccelerationPhase(obj);
                if ( ~searchForGoalNode(obj)) %Search for optimal path and alter lists accordingly
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
                obj.vehicle.setPath(obj.vehicle.pathInfo.path(2:end));%delete old node in path
            end
        end        
        
        %% temporary goal node realted
        function tempFD = newGoalNode(obj)
            %if our original goal node gets unreachable, we try to reach at least a
            %closer node on our path
            path = obj.vehicle.pathInfo.path;
            %path(3) is the next node from current node on
            for i = 3 : size(path,2)
                %if the current node is blocked, we need to set the prior
                %one as a temporary goal
                if obj.nodesBlocked(path(i))
                    %now set new goal
                    if i-1 > 2
                        %to save memory we use i and return
                        i = i-1;
                    else 
                        tempFD = [];
                        return;
                    end
                    temp = path(i);
                    obj.tempGoalNode = temp;
                    %adjust path and FutureData
                    obj.vehicle.setPath(obj.vehicle.pathInfo.path(2:i));
                    tempFD = obj.vehicle.decisionUnit.futureData(1:i,:);
                    %push to open list                    
                    %reinit again like in initilaize()                    
                    initializeGoal(obj,temp);
                    return;
                end
            end
            tempFD = [];
        end         
        function initializeGoal(obj,s)
            %set up new goal node s
            obj.nodesG(s) = 0;
            obj.km = 0;
            obj.nodesParent(s) = 0;
            obj.nodesOpen = zeros(length(obj.Map.waypoints),1);
            obj.nodesVisited = zeros(length(obj.Map.waypoints),1);
            obj.nodesVisited(s) = 1;
            pushOpen(obj,s,calculateKey(obj,s));
        end
        
        %% acceleration estimation related 
        function [nextSpeed,timeToReach] = checkForAccelerationInPathbuilding(obj,currentRoute,currentNode,currentSpeed)
            %nextSpeed = speed on end of edge, timeToReach = exit time of edge
            
            obj.vehicle.setCurrentRoute(currentRoute);%setUp calculation 
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
        
        function  bool = checkTempGoalReached(obj)
            bool = obj.tempGoalNode == obj.vehicle.pathInfo.lastWaypoint;
            % True if TempGoal reached
        end
        
        function setTempGoaltoDestination(obj)                  
                %we need to update oour goal
                obj.tempGoalNode = obj.vehicle.pathInfo.destinationPoint;
                %we have to repush our goal to the open list to search if
                %the path is still blocked
                initializeGoal(obj,obj.tempGoalNode); 
        end
               
        function FuturePlan = findPath(obj,OtherVehiclesFutureData)
            OtherVehiclesFutureData = obj.getOnlyDigraphFutureData(OtherVehiclesFutureData); % TODO - remove later, just for testing
            
            
            FuturePlan = obj.dStarExtraLite(get_param(obj.modelName,'SimulationTime'),OtherVehiclesFutureData);
            
            if size(FuturePlan,2) < 6 % Check if the "-2" tag has already been added
                FuturePlan = [FuturePlan ones(size(FuturePlan,1),1)*-2]; % TODO: to make it nx6 for now
            end
            
        end
        
    end
    
end
