classdef VehiclePathPlanner < matlab.System & handle & matlab.system.mixin.Propagates ...
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
        State        
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
        
        tempGoalNode;           %use this, to drive before a blocked node      
        %end of D Star Extra Light variables
        %start of grid A*
        mapBOG;                 %binary occupancy grid map        
        gridSize = 0.5;         %size of the binary occupancy grid (0.5-2)
        gridLocationMap;
        xOffset;
        yOffset;        
        %end of grid A*
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
            obj.initialize();   

%                 obj.gridLocationMap = containers.Map();
%                 obj.mapBOG = obj.generateBOGrid();                

        end
        
        function [FuturePlan, waypointReached] = stepImpl(obj,OtherVehiclesFutureData)
            %This block shouldn't run if the vehicle has reached its
            %destination 
            
            %comment this in if you want the visualization of the cars
%             if mod(get_param(obj.modelName,'SimulationTime'),0.5) == 0 && obj.vehicle.id == 1                 
%                 obj.visualizePath("");               
%             end
            
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
                    
                    % Build the future plan by deriving the next routes and building the path
                    %Output 1: Future plan of the vehicle
                    
                    %FuturePlan = obj.findNextRoute(obj.vehicle, obj.vehicle.pathInfo.lastWaypoint, obj.vehicle.pathInfo.destinationPoint,get_param(obj.modelName,'SimulationTime'),OtherVehiclesFutureData);
                    FuturePlan = obj.dStarExtraLite(get_param(obj.modelName,'SimulationTime'),OtherVehiclesFutureData);
                    %FuturePlan = obj.dSELCompareToAStar(get_param(obj.modelName,'SimulationTime'),OtherVehiclesFutureData); 
                    %if obj.vehicle.id == 1
                        %FuturePlan = obj.gridAStar(get_param(obj.modelName,'SimulationTime'),OtherVehiclesFutureData);
%                     else
%                         obj.stopVehicle();
%                         FuturePlan = [];
%                     end
                    obj.vehicle.setStopStatus(false);
                    waypointReached =1;                    
%                     obj.visualizePath("");
                else
                    % If the vehicle is still on its route then the future data stays the same
                    %Output 1: Future plan of the vehicle
                    FuturePlan = obj.vehicle.decisionUnit.futureData;
                    waypointReached =0;
                end
            end
        end        
        
        
        %% D* Extra Light
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
            if obj.tempGoalNode == obj.vehicle.pathInfo.lastWaypoint
                %if we are at our temp goal
                
                %we need to update oour goal
                obj.tempGoalNode = obj.vehicle.pathInfo.destinationPoint;
                %we have to repush our goal to the open list to search if
                %the path is still blocked
                initializeGoal(obj,obj.tempGoalNode);
            end
            %futureData = deleteCollidedFutureData(obj,futureData);
            %%vectorized, but slower for now
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
                if ( ~searchForGoalNode(obj)) %Search for optimal path and alter lists accordingly
                    %test if we can reach a closer node at least
                    newFutureData = newGoalNode(obj);
                    if isempty(newFutureData)
                        %we cant reach any node from now
                        disp(['No possible path was found from vehicle ' num2str(obj.vehicle.id)])
                        stopVehicle(obj);
                        return;
                    end
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
        function initialize(obj)
            %initialize whole map
            nMap = length(obj.Map.waypoints);
            obj.nrOfNodes = nMap;
            %nodes
            obj.nodesKey = zeros(nMap,2);
            obj.nodesOpen = zeros(nMap,1);
            obj.nodesParent = zeros(nMap,1);
            obj.nodesVisited = zeros(nMap,1);
            obj.nodesGlobalDistance = zeros(nMap,1);
            obj.nodesG = ones(nMap,1)*2000000000;%intmax
            obj.nodesBlocked = zeros(nMap,1);
            %edges
            eMap = length(obj.Map.connections.all);
            obj.nrOfEdges = eMap;
            obj.edgesSpeed = zeros(eMap,1);
            obj.edgesEntry = zeros(eMap,1);
            obj.edgesExit = zeros(eMap,1);
            obj.seeds = zeros(eMap,1);%if you change the init, change other code too (reinitialize)!           
            
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
            %set up backup goal node
            obj.tempGoalNode = sgoal;            
            
        end   
        
        %% vehilce commands
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
            obj.vehicle.pathInfo.path = newpath(1:i);%cut preallocated vector
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
            newCost = obj.edgesCost(curEdge);
            
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
%                         %% disturbing car on same route
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
                        end
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
        
        %% blocking nodes
        function detectBlockingCarsForLoop(obj)
            %if a car without future data blocks a node, we set the node blocked
            
            %get all other cars
            otherCars = getNrOfAllOtherCars(obj); 
            vehicles = obj.vehicle.map.Vehicles;
            for car = otherCars
                if vehicles(car).pathInfo.destinationReached
                    obj.nodesBlocked(vehicles(car).pathInfo.lastWaypoint)=1;
                end
            end
        end        
        function detectBlockingCars(obj)
            %blocks every goal node of a finished car
            %this includes stopped cars by accident and destination
            otherCars = obj.getNrOfAllOtherCars();
            vehicles = obj.vehicle.map.Vehicles(otherCars);
            pathInfo = [vehicles.pathInfo];
            reached = [pathInfo.destinationReached];
            otherCars = otherCars(reached);
            destinationPoints = [pathInfo(otherCars).lastWaypointt];
            obj.nodesBlocked(destinationPoints) = 1;            
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
        function futureData = deleteCollidedFutureDataForLoop(obj,futureData)
            %deletes future data of vehicles that will not move because of collision
            otherCars = getNrOfAllOtherCars(obj); 
            vehicles = obj.vehicle.map.Vehicles;
            obj.changedEdges = [];%reset
            obj.haveCostsChanged = false;
            for car = otherCars
                if vehicles(car).status.collided
                    %remove every entry with the collided car from FD
                    futureData = futureData(futureData(:,1)~=car,:);
                    %block the start and the future node of the crash
                    area = [vehicles(car).pathInfo.lastWaypoint,vehicles(car).pathInfo.path(2)];
                    obj.nodesBlocked(area(1))=1;
                    %TODO do we block too often here?
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
                obj.vehicle.pathInfo.path = obj.vehicle.pathInfo.path(2:end);%delete old node in path
            end
        end
        function newFutureData = dSELCompareToAStar(obj, globalTime,futureData)
            %returns the shortest path based on digraph
            %no FD was usedto calculate it
            %we only need to calculate turn one, because we wont make any
            %changes to the edge costs
            
            checkForAccelerationPhase(obj);
            whichEdgecostsChangedForLoop(obj,futureData)
            if obj.haveCostsChanged || isempty(obj.vehicle.pathInfo.path)
                %% Reinititialize
                if obj.haveCostsChanged
                    reinitialize(obj,futureData);
                end
                % Use D*EL to get path to goal node
                if ( ~searchForGoalNode(obj)) %Search for optimal path and alter lists accordingly
                    %we cant reach any node from now
                    disp(['No possible path was found from vehicle ' num2str(obj.vehicle.id)])
                    stopVehicle(obj);
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
                        %to safe memory we use i and return
                        i = i-1;
                    else 
                        tempFD = [];
                        return;
                    end
                    temp = path(i);
                    obj.tempGoalNode = temp;
                    %adjust path and FutureData
                    obj.vehicle.pathInfo.path = obj.vehicle.pathInfo.path(2:i);
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
            obj.nodesG(s) = 0;
            obj.km = 0;
            obj.nodesParent(s) = 0;
            obj.nodesOpen = zeros(length(obj.Map.waypoints),1);
            obj.nodesVisited = zeros(length(obj.Map.waypoints),1);
            obj.nodesVisited(s) = 1;
            pushOpen(obj,s,calculateKey(obj,s));
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
        
        %% grid related
        function map = generateBOGrid(obj)
            %create binary occupancy grid object and grid location objects
            
            %% prepare for drawing
            nrOfCars = length([obj.Map.Vehicles.id]);
            gSize = obj.gridSize;
            %first we need map size
            %we can get it from waypoints with some space for better
            %display
            distances = obj.Map.connections.distances;
            w = obj.vehicle.map.waypoints;  %[x z y]
            w(:,3) = -1.*w(:,3);%transform to mobatsim coordinates
            xSize = max(w(:,1))-min(w(:,1))+100;
            xOff = min(w(:,1))-50;
            obj.xOffset = xOff;
            ySize = max(w(:,3))-min(w(:,3))+100;
            yOff = min(w(:,3))-50;
            obj.yOffset = yOff;
            map = binaryOccupancyMap(xSize,ySize,gSize);
            %mark everything as blocked
            occ = ones(ySize*gSize,xSize*gSize);
            setOccupancy(map,[0,0],occ);%lower left corner to set values
            
            %import map details
            trans = obj.vehicle.map.connections.translation; %[from to speed]
            circ = obj.vehicle.map.connections.circle; %[from to angle xzyCenter speed]
            circ(:,6) = -1.*circ(:,6);           
            
            %we iterate backwards in case we want to preallocate something
            
            %% bresenham algorithm
            for j = size(obj.vehicle.map.connections.all,1) :-1: (size(circ,1)+1) 
                t= j - size(circ,1);
                %j is nr for all, t only inside transitions
                dist = distances(j);
                %start node coordinates in grid
                p1 = map.world2grid([w(trans(t,1),1)-xOff,w(trans(t,1),3)-yOff]);
                %end node coordinates in grid
                p2 = map.world2grid([w(trans(t,2),1)-xOff,w(trans(t,2),3)-yOff]);
                %get difference
                deltaX = p2(1)-p1(1);
                delatY = p2(2)-p1(2);
                %get deistance and direction
                absDx = abs(deltaX);
                absDy = abs(delatY); % distance
                signDx = sign(deltaX);
                signDy = sign(delatY); % direction
                %determine which direction to go
                if absDx > absDy
                    % x is shorter
                    pdx = signDx;
                    pdy = 0;        %p is parallel
                    ddx = signDx;
                    ddy = signDy; % d is diagonal
                    deltaLongDirection  = absDy;
                    deltaShortDirection  = absDx;
                else
                    % y is shorter
                    pdx = 0;
                    pdy = signDy; % p is parallel
                    ddx = signDx;
                    ddy = signDy; % d is diagonal
                    deltaLongDirection  = absDx;
                    deltaShortDirection  = absDy;
                end
                %start at node 1
                x = p1(1);
                y = p1(2);
                setOccupancy(map,[x,y],0,"grid");
                
                %create grid location================================= 
                startNodeNR = obj.vehicle.map.connections.all(j,1);
                endNodeNR = obj.vehicle.map.connections.all(j,2);
                curKey = append(num2str(x), ",", num2str(y));
                %if we dont have it -> create, else we import it
                if ~obj.gridLocationMap.isKey(curKey)                
                    %if we dont have the gl in our map we need to add it
                    curGL = GridLocation([x,y],nrOfCars,startNodeNR,0);
                else
                    curGL = obj.gridLocationMap(curKey);
                end
                pixelArray = [];
                %=====================================================                
                
                error = deltaShortDirection/2;
                
                %now set the pixel
                for i= 1:deltaShortDirection          
                    % for each pixel
                    % update error
                    error = error - deltaLongDirection;
                    if error < 0
                        error = error + deltaShortDirection; % error is never < 0
                        % go in long direction
                        x = x + ddx;
                        y = y + ddy; % diagonal
                    else
                        % go in short direction
                        x = x + pdx;
                        y = y + pdy; % parallel
                    end
                    setOccupancy(map,[x,y],0,"grid");
                    %================={build grid}=======================
                    %treat the last one different
                    if i ~= deltaShortDirection                        
                        %create new GL
                        newKey = append(num2str(x), ",", num2str(y));                        
                        %create new key
                        if ~obj.gridLocationMap.isKey(newKey)
                            newGL = GridLocation([x,y],nrOfCars,0,j);
                        else
                            newGL = obj.gridLocationMap(newKey);
                        end
                        curGL = curGL.addTransSucc(newKey,startNodeNR);
                        newGL = newGL.addTransPred(curKey,endNodeNR);
                        %now we can add the old gl to the map                        
                        pixelArray = [pixelArray,curGL];
                        %remember for next round
                        
                        curGL = newGL;
                        curKey = newKey;
                    else
                        %if we dont have the gl in our map we need to add it
                        newKey = append(num2str(x), ",", num2str(y));
                        if ~obj.gridLocationMap.isKey(newKey)
                            %create new GL
                            newGL = GridLocation([x,y],nrOfCars,obj.vehicle.map.connections.all(j,2),0);
                        else
                            %if it already exist, we just update succs and preds
                            newGL = obj.gridLocationMap(newKey);
                        end
                        %assign grid relation
                        curGL = curGL.addTransSucc(newKey,startNodeNR);
                        newGL = newGL.addTransPred(curKey,endNodeNR);
                        pixelArray = [pixelArray,curGL,newGL];
                    end
                    %=====================================================
                end 
                %now assign properties
                dist = dist/size(pixelArray,2);
                for pix = pixelArray
                    p = pix.assignDistance(dist);
                    p.speedLimit = obj.maxEdgeSpeed(j);
                    %assign to map
                    obj.gridLocationMap(append(num2str(p.coordinates(1)), ",", num2str(pix.coordinates(2))))=p;
                end
            end
                        
            %% draw a circle pixel by pixel
            for t = size(circ,1):-1:1
                
                dist = distances(t);
                pStart = map.world2grid([w(circ(t,1),1)-xOff,w(circ(t,1),3)-yOff]); %start
                pGoal = map.world2grid([w(circ(t,2),1)-xOff,w(circ(t,2),3)-yOff]); %goal
                pCenter = map.world2grid([circ(t,4)-xOff,circ(t,6)-yOff]); %central point
                radius = round(norm( pGoal-pCenter  ),0);
                phiStart = angle(complex((pStart(1)-pCenter(1)) , (pStart(2)-pCenter(2))));
                phiGoal = angle(complex((pGoal(1)-pCenter(1)) , (pGoal(2)-pCenter(2))));
                direction = sign(circ(t,3));
                if phiStart <0
                    phiStart = phiStart + 2*3.1415;
                end
                if phiGoal <0
                    phiGoal = phiGoal + 2*3.1415;
                end
                offset = 0;
                %make turns through 0° possible
                if (direction == -1 && phiStart < phiGoal)
                    offset = 2*3.1415;
                    phiStart = phiStart + offset;
                end
                if (direction == 1 && phiStart > phiGoal)
                    offset = 2*3.1415;
                    phiGoal = phiGoal + offset;
                end
                
                %calculate the next pixel
                curPix = pStart;
                pixelArray = [];
                pixelArray = [pixelArray,append(num2str(curPix(1)), ",", num2str(curPix(2)))];
                curPhi = phiStart;
                while curPix(1) ~= pGoal(1) || curPix(2) ~= pGoal(2)
                    nextPix = [];   %the next pixel to draw in grid
                    nextPhi = 0;
                    deltaR = 200000;     %distance to the radius point with angle phi
                    %for every neighbour
                    for x = -1 : 1
                        for y = -1 : 1
                            if x ~= 0 || y ~= 0
                                %compare pixel for the closest one to the original
                                %point
                                %calculate the distance between the pixel and the
                                %reference and use the closest
                                neighbourPix = [curPix(1)+x,curPix(2)+y];%new pixel in grid
                                phi = angle(complex((neighbourPix(1)-pCenter(1)) , (neighbourPix(2)-pCenter(2))));%angle in world
                                if phi < 0
                                    phi = phi + 2*3.1415;
                                else
                                    phi = phi + offset;
                                end
                                %test, if the angle is relevant
                                if (direction == 1 && curPhi <= phi && phiGoal >= phi)|| (direction ==-1 && curPhi >= phi && phiGoal <= phi)
                                    referencePoint = [radius * cos(phi)+pCenter(1),radius*sin(phi)+pCenter(2)];%reference in world
                                    refDeltaR = norm(neighbourPix - referencePoint);
                                    if refDeltaR < deltaR
                                        deltaR = refDeltaR;
                                        nextPix = neighbourPix;
                                        nextPhi = phi;
                                    end
                                end
                            end
                        end
                    end                    
                    curPix = nextPix;
                    curPhi = nextPhi;
                    %now curPix is the next pixel to draw
                    pixelArray = [pixelArray, append(num2str(curPix(1)), ",", num2str(curPix(2)))];
                end
                %now draw pixel and assign to map
                startNodeNR = obj.vehicle.map.connections.all(t,1);
                endNodeNR = obj.vehicle.map.connections.all(t,2);
                curKey = pixelArray(1);
                dist = dist/size(pixelArray,2);
                %load or create gl for starting node
                if ~obj.gridLocationMap.isKey(curKey)
                    curGL = GridLocation(pStart,nrOfCars,obj.vehicle.map.connections.all(t,1),0);
                else
                    curGL = obj.gridLocationMap(curKey);
                end
                curGL = curGL.assignDistance(dist);%assign distance
                curGL.speedLimit = obj.maxEdgeSpeed(t);
                p = str2num(pixelArray(1));
                setOccupancy(map,p,0,"grid");%draw
                
                for s = 2 : (length(pixelArray)-1)   %start connecting                 
                    newKey = pixelArray(s);
                    p = str2num(newKey);
                    setOccupancy(map,p,0,"grid");
                    %create new GL
                    if ~obj.gridLocationMap.isKey(newKey)
                        newGL = GridLocation(p,nrOfCars,0,t);
                    else
                        %load
                        newGL = obj.gridLocationMap(newKey);
                    end
                    newGL = newGL.assignDistance(dist);
                    curGL.speedLimit = obj.maxEdgeSpeed(t);
                    %assign succ
                    curGL = curGL.addTransSucc(newKey,startNodeNR);
                    newGL = newGL.addTransPred(curKey,endNodeNR);
                    obj.gridLocationMap(curKey)=curGL;
                    curGL = newGL;
                    oldKey = curKey;
                    curKey = newKey;                    
                end
                %if we dont have the gl in our map we need to add it
                newKey = pixelArray(end);
                if ~obj.gridLocationMap.isKey(newKey)
                    %create new
                    p = str2num(newKey);
                    newGL = GridLocation(p,nrOfCars,obj.vehicle.map.connections.all(t,2),0);
                else
                    %load
                    newGL = obj.gridLocationMap(newKey);
                end
                setOccupancy(map,p,0,"grid");
                p = str2num(oldKey);
                setOccupancy(map,p,0,"grid");
                %assign grid relation               
                newGL = newGL.assignDistance(dist);
                newGL.speedLimit = obj.maxEdgeSpeed(t);
                curGL = curGL.addTransSucc(newKey,startNodeNR);
                newGL = newGL.addTransPred(curKey,endNodeNR);
                obj.gridLocationMap(curKey)=curGL;
                obj.gridLocationMap(newKey)=newGL;
            end
            
            %% export maps
            %TODO export to other vehicles or make the generation global
        end     
        %% visualization
        function generateMapVisual(~,map,displayInGridCoordinates)
            %This function plots any XML Map of MOBATSim. Keep in mind that you have to
            %do a coordinate transformation between normal coordinates and grid / mobatsim
            %Input: XML Map object of MOBATSim, boolean wether to plot mobatsim or grid coordinates
            %Output TODO return the plot to use it later

            %% prepare everything
            %open another figure
            %figure(3);
            plot(1,1,'color',[1,1,1]);%TODO replace with better solution for deleting old plot
            hold on
            w = map.waypoints;
            circ = map.connections.circle;
            trans = map.connections.translation;
            %coordinate transformation
            w(:,3) = -1.*w(:,3);
            circ(:,6) = -1.*circ(:,6);
            if displayInGridCoordinates
                xOff = min(w(:,1))-50;  %TODO make it global later
                yOff = min(w(:,3))-50;  %offset should be global
                
                w(:,3) = w(:,3)-yOff;
                w(:,1) = w(:,1)-xOff;                
                
                circ(:,4) = circ(:,4)-xOff;
                circ(:,6) = circ(:,6)-yOff;
            end
           
            %% Generate a usable plot
            %% generate curves
            for c = 1 : length(circ)
                cPart = circ(c,:);
                %start
                x1 = w(cPart(1),1); 
                y1 = w(cPart(1),3);   
                %goal
                x2 = w(cPart(2),1); 
                y2 = w(cPart(2),3);
                %central point
                x0W = cPart(4); 
                y0W = cPart(6);
                %radius
                radius = norm( [x2,y2]-[x0W,y0W] );
                %angles
                phiStart = angle(complex((x1-x0W) , (y1-y0W)));
                phiGoal = angle(complex((x2-x0W) , (y2-y0W)));
                %direction
                direction = sign(cPart(3));
                %% make angle allways usable
                if phiStart <0
                    phiStart = phiStart + 2*3.1415;
                end
                if phiGoal <0
                    phiGoal = phiGoal + 2*3.1415;
                end
                %make turns through 0° possible
                if (direction == -1 && phiStart < phiGoal)
                    phiStart = phiStart + 2*3.1415;
                end
                if (direction == 1 && phiStart > phiGoal)
                    phiGoal = phiGoal + 2*3.1415;
                end
                phi1 = phiStart : direction*0.01 : phiGoal;
                phi1(1) = phiStart;
                phi1(end) = phiGoal;
                points = [(radius .* cos(phi1)+x0W)',(radius .* sin(phi1))'+y0W];
                
                plot(points(:,1),points(:,2),'color',[0 1 0],'LineWidth',2)
            end
            %% generate straight lines
            for t = 1 : length(trans)
                position = zeros(2,2);
                %get both points and plot a line in between
                position(1,:) = [w(trans(t,1),1) ,w(trans(t,1),3)];
                position(2,:) = [w(trans(t,2),1) ,w(trans(t,2),3)];
                plot(position(:,1),position(:,2),'color',[0 1 0],'LineWidth',2);
            end
            %% plot nodes with numbers
            for n = 1 : length(w)
                pos = [w(n,1),w(n,3)];
                plot(pos(1),pos(2),'Marker','o','MarkerFaceColor',[0 0 0],'color',[0,0,0]);
                text(pos(1)-5,pos(2)-15,num2str(n));%TODO make it appear automatically under dot
                %maybe there is a way to let it not collide with other text?
            end
            hold off
        end
        function visualizePath(obj,displayStyle)
            %shows binary occupancy grid with the path of every vehicle
            %Input: boolean, should we plot the grid or the normal plot?
            %Only use true, if you have created the bog globaly!
            
            map = obj.vehicle.map;
            figure(4)%figure(2) causes errors
            
            if strcmp(displayStyle, "bog")
                show(obj.mapBOG)
                contrastArray = [1 1 1];
                displayInGridCoordinates = true;
            elseif strcmp(displayStyle, "bogPlot")  
                displayInGridCoordinates = true;
                contrastArray = [0 0 0];
                obj.generateMapVisual(map,true);%TODO use only the plot later 
            else
                displayInGridCoordinates = false;
                contrastArray = [0 0 0];
                obj.generateMapVisual(map,false);%TODO use only the plot later
            end
            hold on
            obj.plotPath(map,displayInGridCoordinates,contrastArray);
            hold off
        end
        function plotPath(~,map,displayInGridCoordinates,contrastArray)
            %this function plots the current position and path of all
            %vehicles on the map
            %Input: map is the XML map object, 
            %displayInGridCoordinates is a boolean: true = display grid, false = display normal plot
            %% prepare everything
            w = map.waypoints;
            w(:,3) = -1.*w(:,3);
            circ = map.connections.circle;
            circ(:,6) = -1.*circ(:,6);
            trans = map.connections.translation;
            if displayInGridCoordinates
            xOff = min(w(:,1))-50;
            yOff = min(w(:,3))-50;
            else
                xOff = 0;
                yOff = 0;
            end
            %% plot path for every vehicle
            for c = [map.Vehicles.id]
                %for every vehicle draw the path
                path = [map.Vehicles.pathInfo];
                path = path(c);
                path = path.path;%path of the vehicle
                bogPath = [];%list of points to plot
                %determine if circle or not
                for k = 2 : size(path,2)
                    p1 = path(k-1);
                    p2 = path(k);
                    straight = ~isempty(trans(trans(:,1) == p1 & trans(:,2) == p2,:));
                    if straight
                        %just draw a line
                        if isempty(bogPath)
                            bogPath = [bogPath;[w(p1,1)-xOff,w(p1,3)-yOff]; [w(p2,1)-xOff,w(p2,3)-yOff]];
                            %replace first entry with current position
                            bogPath(1,:) = [map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff];
                        else
                            bogPath = [bogPath;[w(p1,1)-xOff,w(p1,3)-yOff]; [w(p2,1)-xOff,w(p2,3)-yOff]];
                        end  
                    else
                        %gather points on the circle
                        rad = circ( circ(:,1)==p1 & circ(:,2) == p2,:);
                        if isempty(bogPath)
                            x1 = map.Vehicles(c).dynamics.position(1)-xOff;
                            y1 = -map.Vehicles(c).dynamics.position(3)-yOff;
                        else
                        x1 = w(rad(1),1)-xOff; %start
                        y1 = w(rad(1),3)-yOff;
                        end
                        x2 = w(rad(2),1)-xOff; %goal
                        y2 = w(rad(2),3)-yOff;
                        x0W = rad(4)-xOff; %central point
                        y0W = rad(6)-yOff;
                        radius = norm( [x2,y2]-[x0W,y0W] );
                        phiStart = angle(complex((x1-x0W) , (y1-y0W)));
                        phiGoal = angle(complex((x2-x0W) , (y2-y0W)));
                        direction = sign(rad(3));
                        if phiStart <0
                            phiStart = phiStart + 2*3.1415;
                        end
                        if phiGoal <0
                            phiGoal = phiGoal + 2*3.1415;
                        end
                        %make turns through 0° possible
                        if (direction == -1 && phiStart < phiGoal)
                            phiStart = phiStart + 2*3.1415;
                        end
                        if (direction == 1 && phiStart > phiGoal)
                            phiGoal = phiGoal + 2*3.1415;
                        end
                        phi1 = phiStart : direction*0.1 : phiGoal;
                        phi1(1) = phiStart;
                        phi1(end) = phiGoal;
                        points = [(radius .* cos(phi1)+x0W)',(radius .* sin(phi1))'+y0W];
                        bogPath = [bogPath;points];
                    end
                    
                end
                if ~isempty(bogPath)
                    switch c
                        case 1
                            %red
                            p = plot(bogPath(:,1),bogPath(:,2),'color',[1 0 0],'LineWidth',2);
                            plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[1 0 0],'color',contrastArray);
                        case 2
                            %yellow
                            p = plot(bogPath(:,1),bogPath(:,2),'color',[1 1 0],'LineWidth',2);
                            plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[1 1 0],'color',contrastArray);
                        case 3
                            %light blue
                            p = plot(bogPath(:,1),bogPath(:,2),'color',[0 1 1],'LineWidth',2);
                            plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[0 1 1],'color',contrastArray);
                        case 4
                            %green
                            p = plot(bogPath(:,1),bogPath(:,2),'color',[0.4660 0.6740 0.1880],'LineWidth',2);
                            plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[0.4660 0.6740 0.1880],'color',contrastArray);
                        case 5
                            %orange
                            p = plot(bogPath(:,1),bogPath(:,2),'color',[0.8500 0.3250 0.0980],'LineWidth',2);
                            plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[0.8500 0.3250 0.0980],'color',contrastArray);
                        case 6
                            %magenta
                            p = plot(bogPath(:,1),bogPath(:,2),'color',[1 0 1],'LineWidth',2);
                            plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[1 0 1],'color',contrastArray);
                        case 7
                            %blue
                            p = plot(bogPath(:,1),bogPath(:,2),'color',[0 0.4470 0.7410],'LineWidth',2);
                            plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[0 0.4470 0.7410],'color',contrastArray);
                        case 8
                            %dark red
                            p = plot(bogPath(:,1),bogPath(:,2),'color',[0.6350 0.0780 0.1840],'LineWidth',2);
                            plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[0.6350 0.0780 0.1840],'color',contrastArray);
                        case 9
                            %light green
                            p = plot(bogPath(:,1),bogPath(:,2),'color',[0.9290 0.6940 0.1250],'LineWidth',2);
                            plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[0.9290 0.6940 0.1250],'color',contrastArray);
                        otherwise
                            %violet
                            p = plot(bogPath(:,1),bogPath(:,2),'color',[0.4940 0.1840 0.5560],'LineWidth',2);
                            plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[0.4940 0.1840 0.5560],'color',contrastArray);
                    end 
                    %make it transparent to see better
                    p.Color(4) = 0.75;
                    text(map.Vehicles(c).dynamics.position(1)-xOff-15,-map.Vehicles(c).dynamics.position(3)-yOff+20,append("V",num2str(c)),'color',contrastArray);
                end
            end            
        end
        %% Grid A* Code
        function newFutureData = gridAStar(obj, globalTime,futureData)
            %this function performs a A* search with the grid location
            %objects from obj.gridLocationMap
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
            curGridPos = obj.mapBOG.world2grid([curPos(1)-obj.xOffset,-curPos(3)-obj.yOffset]);
            curKey = append( num2str(curGridPos(1)),",",num2str(curGridPos(2)) );
            startKey = curKey;
            curGL = obj.gridLocationMap(curKey);
            curGL.gValue = globalTime;
            curGL.speedVector(carID) = obj.vehicle.dynamics.speed;
            openList(curKey) = curGL;
            
            %get key of goal node
            curPos = obj.Map.waypoints(obj.tempGoalNode,:);
            goalPos = obj.mapBOG.world2grid([curPos(1)-obj.xOffset,-curPos(3)-obj.yOffset]);%TODO move tempgoal init to a new init function
            goalKey = append( num2str(goalPos(1)),",",num2str(goalPos(2)) );
            goalCoordinates = str2num(goalKey);
            
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
                    succGL = obj.gridLocationMap(succKey);
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
%                     coords = obj.Map.waypoints(vehicles(car).pathInfo.lastWaypoint,:);
%                     coords = obj.mapBOG.world2grid([coords(1)-obj.xOffset,-coords(3)-obj.yOffset]);                    
%                     futureData = [futureData;[car , coords , 0, 0, -1]];
                    %% block the future node of the crash
                    coords = obj.Map.waypoints(vehicles(car).pathInfo.path(2),:);
                    coords = obj.mapBOG.world2grid([coords(1)-obj.xOffset,-coords(3)-obj.yOffset]);
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
                coords = obj.mapBOG.world2grid([coords(1)-obj.xOffset,-coords(3)-obj.yOffset]);
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
                    coords = obj.mapBOG.world2grid([coords(1)-obj.xOffset,-coords(3)-obj.yOffset]);
                    futureData = [futureData;[car , coords , 0, 0, -1]];
                end
            end
        end   
        
        
        
        %% Old A* Code
        
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
            %out = [50 5];
            out = [2000 6];
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
