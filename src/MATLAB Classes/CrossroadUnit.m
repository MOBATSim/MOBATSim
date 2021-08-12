classdef CrossroadUnit < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %static
        id                  % identification of the crossroad unit
        startingNodes
        brakingNodes
        stoppingNodes
        leavingNodes
        params
             % alpha
             % criticalETA
             % intelligentDecision
             % platooningTimeBehind
             % conventionalTrafficLights
             % energyEquation               
        brakingFlagArray % Array with the following structure: [ carId brakingFlag]
                         %                                     [ carId brakingFlag]
                         % brakingFlag: 1 -> Stop!, 0 -> Go!
        
        overlappingTable    % each column contains the overlapping directions of direction in header
        vehicleOrders       % array with rows containing the vehicle id and the order ( 0 = go, 1 = stop)
        arrivingGroup       % all vehicles arriving at the crossroad
        leavingGroup        % all vehicles at the crossroad
        % Conventional traffic lights
        stateStartingTime       = 0   % when the new traffic state has started
        currentState            = 1   % current traffic light state
        stateDuration           = 7   % the duration a state stays active
        waitingBetweenStates    = 7   % the duration between to states
    end
    
    methods
        function obj = CrossroadUnit(id, startingNodes,brakingNodes,stoppingNodes,leavingNodes, configurations)
            arguments
                id                                         (1,1) double
                startingNodes                              (1,4) double
                brakingNodes                               (1,4) double
                stoppingNodes                              (1,4) double
                leavingNodes                               (1,4) double
                configurations.conventionalTrafficLights   (1,1) logical   = true  % use conventional traffic lights
                configurations.intelligentDecision         (1,1) logical   = false  % 0 for FCFS, 1 for intelligent algorithm
                configurations.energyEquation              (1,1) logical   = false  % 0 for time optimized approach, 1 for energy optimized approach
            end
            obj.id = id;          
            obj.startingNodes = startingNodes;
            obj.brakingNodes = brakingNodes;
            obj.stoppingNodes = stoppingNodes;
            obj.leavingNodes = leavingNodes;                      
            
            % edit all the parameters at this point to change the algorithm
            obj.params.criticalETA = 10; % delta_TTR in our paper
            obj.params.alpha = 0.072;
            obj.params.alpha2 = 0.001;
            %platooning control isn't working as expected, therefore a
            %constant platooning time behind the car ahead is set:
            obj.params.platooningTimeBehind = 0.2;           
            obj.params.intelligentDecision          = configurations.intelligentDecision;
            obj.params.conventionalTrafficLights    = configurations.conventionalTrafficLights;
            obj.params.energyEquation               = configurations.energyEquation;
            
            % table containing all overlaps of the crossroad
            obj.overlappingTable = obj.generateOverlappingTable();
        end %Constructor
        
        function overlappingTable = generateOverlappingTable(~)
            % table containing all overlapping lanes on crossroad
            
            % the direction in one array are colliding with the direction
            % that is the variable name
            NE = ["ES";"EW";"SN";"WN"];
            NS = ["EW";"SW";"WE";"WN"];
            ES = ["NE";"SN";"SW";"WE"];
            EW = ["NE";"NS";"SN";"WN"];
            SN = ["NE";"ES";"EW";"WE"];
            SW = ["NS";"ES";"WN";"WE"];
            WN = ["NE";"NS";"EW";"SW"];
            WE = ["NS";"ES";"SN";"SW"];
            
            
            overlappingTable = table(NE,NS,ES,EW,SN,SW,WN,WE);
        end        
        
        function carReachesCrossroad(obj,vehicle)
            % when a car is reaching the starting node of a crossroad, the
            % car has to be registrated in the arriving queue
            
            % Traffic lights activated
            if obj.params.conventionalTrafficLights == 1
                
                % PLACEHOLDER
                
            % Intelligent algorithm activated
            elseif obj.params.intelligentDecision == 1
                
                % Add vehicle to arriving queue
                arrivingGroupEntry = obj.makeArrivingGroupEntry(vehicle, obj.startingNodes, obj.leavingNodes);
                if ~isempty(arrivingGroupEntry)
                    obj.arrivingGroup(end+1,:) = arrivingGroupEntry;
                end
                
            % FCFS algorithm activated
            elseif obj.params.intelligentDecision == 0
                
                % PLACEHOLDER
                
            end            
            
        end
        
        function carReachesBrakingPoint(obj, vehicle, vehicles)
            % this function is executed when a car reaches the braking
            % point.
             % Traffic lights activated
            if obj.params.conventionalTrafficLights == 1               
                
                % Add vehicle to arriving queue
                arrivingGroupEntry = obj.makeArrivingGroupEntry(vehicle, obj.brakingNodes, obj.leavingNodes);
                if ~isempty(arrivingGroupEntry)
                    obj.arrivingGroup(end+1,:) = arrivingGroupEntry;
                end
                
            % Intelligent algorithm activated
            elseif obj.params.intelligentDecision == 1
                
                % call crossroad algorithm
                [obj.vehicleOrders, obj.arrivingGroup, obj.leavingGroup] = obj.runCrossroadAlgorithm(vehicles, obj.arrivingGroup, obj.leavingGroup);
            
            % FCFS algorithm activated
            elseif obj.params.intelligentDecision == 0
                               
                % Add vehicle to arriving queue
                arrivingGroupEntry = obj.makeArrivingGroupEntry(vehicle, obj.brakingNodes, obj.leavingNodes);
                if ~isempty(arrivingGroupEntry)
                    obj.arrivingGroup(end+1,:) = arrivingGroupEntry;
                end
                % call crossroad algorithm
                [obj.vehicleOrders, obj.arrivingGroup, obj.leavingGroup] = obj.runCrossroadAlgorithm(vehicles, obj.arrivingGroup, obj.leavingGroup);
            
            end
            
        end
        
        function carReachesStartingPoint(obj, ~)
            % car reaches the start of the crossroad
            
            % Traffic lights activated
            if obj.params.conventionalTrafficLights == 1
                
                % PLACEHOLDER
                
            % Intelligent algorithm activated
            elseif obj.params.intelligentDecision == 1
                
                % PLACEHOLDER
            
            % FCFS algorithm activated
            elseif obj.params.intelligentDecision == 0
                
                % PLACEHOLDER
                
            end
        end
        
        function carLeavesCrossroad(obj,vehicle, vehicles)
            % when a vehicle reaches the leaving node of a crossroad, the
            % main algorithm has to be executed again and the car has to be
            % deleted from the leaving queue
            
            % Traffic lights activated
            if obj.params.conventionalTrafficLights == 1
                
                if ~isempty(obj.leavingGroup)
                    
                    % Remove vehicle from leavingGroup
                    obj.leavingGroup(obj.leavingGroup(:,1)== vehicle.id,:) = [];
                    
                end
                
            % Intelligent algorithm activated
            elseif obj.params.intelligentDecision == 1
                
                if ~isempty(obj.leavingGroup)
                    
                    % Remove vehicle from leavingGroup
                    obj.leavingGroup(obj.leavingGroup(:,1)== vehicle.id,:) = [];
                    
                    % call crossroad algorithm
                    [obj.vehicleOrders, obj.arrivingGroup, obj.leavingGroup] = obj.runCrossroadAlgorithm(vehicles, obj.arrivingGroup, obj.leavingGroup);
                end
                
            % FCFS algorithm activated
            elseif obj.params.intelligentDecision == 0
                
                if ~isempty(obj.leavingGroup)
                    
                    % Remove vehicle from leavingGroup
                    obj.leavingGroup(obj.leavingGroup(:,1)== vehicle.id,:) = [];
                    
                    % call crossroad algorithm
                    [obj.vehicleOrders, obj.arrivingGroup, obj.leavingGroup] = obj.runCrossroadAlgorithm(vehicles, obj.arrivingGroup, obj.leavingGroup);
                end
                
            end
            
        end
        
        function [vehicleOrders, arrivingGroup, leavingGroup] = runCrossroadAlgorithm(obj, vehicles, arrivingGroup, leavingGroup)
            % This is the main method for the intelligent crossroad manager
            % algorithm.
            
            % Only when a vehicle is arriving execute the algorithm
            if ~isempty(obj.arrivingGroup)
                % get all prioritized vehicles and their priorities
                priorityGroup = obj.buildPriorityGroup(vehicles, arrivingGroup);
                % give every priority vehicle a GO/STOP order
                vehicleOrders = obj.getVehicleOrders(priorityGroup, leavingGroup);
                % transfer vehicles that will pass to leaving group
                [arrivingGroup, leavingGroup] = obj.moveFromArrivingToLeavingGroup(arrivingGroup, leavingGroup, vehicleOrders);
            else
                vehicleOrders = [];
            end
        end
        
        function entry = makeArrivingGroupEntry(~, vehicle, startingNodes, leavingNodes)
            % add a vehicle to arriving group
                
                entry = [];
                % check that cars destination is not  before crossroad
                if length(vehicle.pathInfo.path)>3
                    arrivingDirection = find(vehicle.pathInfo.lastWaypoint == startingNodes); % from which direction the car is coming 1=N, 2=E, 3=S, 4=W
                    
                    
                    index = find(ismember(vehicle.pathInfo.path,leavingNodes));                   

                    try
                        leavingDirection = find(vehicle.pathInfo.path(index(1)) == leavingNodes); % in which direction the car is going 1=N, 2=E, 3=S, 4=W
                        
                    catch
                        
                        disp('crossroad error');
                        
                    end
                    
                    % arring queue definition (one vehicle entry):
                    % | vehicle id | arriving direction | leaving direction |
                    entry = [vehicle.id arrivingDirection leavingDirection];
                end
        end

        function [arrivingGroup, leavingGroup] = moveFromArrivingToLeavingGroup(~, arrivingGroup, leavingGroup, vehicleOrders)
            % Move all vehicles that got a 'go' command from the arriving
            % group to the leaving group
            
            % no vehicles to move
            if isempty(arrivingGroup)
                return
            end
            
            % get all vehicles that have a 'go' command
            passingVehicles = vehicleOrders(vehicleOrders(:,2) == 0,1);
            
            % nothing changed
            if isempty(passingVehicles)
                return
            end
            
            % move every passing vehicle from arrivingGroup to leavingGroup
            entriesLeaving = arrivingGroup(any(arrivingGroup(:,1) == passingVehicles',2),:);
            % add leaving to leavingGroup
            leavingGroup(end+1:end+size(entriesLeaving,1),:) = entriesLeaving;
            % remove leaving from arrivingGroup
            arrivingGroup(any(arrivingGroup(:,1) == passingVehicles',2),:) = [];                
            
        end                
        
        
        function priorityGroup = buildPriorityGroup(obj, vehicles, arrivingGroup)
            % get and add priorities to the priority group, vehicles in this group are
            % considered by the algorithm
            
            % get priority group by looking at vehicles close to crossroad
            priorityGroup = obj.getPriorityGroup(vehicles, arrivingGroup);
            
            % calculate priorities for priority group
            priorityGroup = obj.calculatePriority(priorityGroup, vehicles); 
        end
        
        
        function priorityGroup = getPriorityGroup(obj, vehicles, arrivingGroup) % get first order vehicle group which only is considered in the algorithm
            % in this function we delimitate vehicles with close ETAs. The
            % parameter we are using is criticalETA (in our paper: deltaETA)
            
            % directions from that the vehicles enter the crossroad
            arrivingDirection = arrivingGroup(:,2);
            
            % we loop through all vehicles to get the estimated time of
            % arrival at the conflict zone (ETA)
            for i=1:size(arrivingGroup,1)                         
                
                % Find a vehicle in front of current vehicle.
                % Vehicle in front is the last vehicle added to arriving
                % queue before the current vehicle with same arriving
                % direction
                indexVehicleInFront = find(arrivingDirection(i) == arrivingDirection(1:i-1),1,'last');
                
                % Take ETA from vehicle in front.
                if ~isempty(indexVehicleInFront)
                    ETAcarInFront = arrivingGroup(indexVehicleInFront,4);
                else
                    ETAcarInFront = 0;
                end
                
                % get the current vehicle of arriving queue
                vehicle = vehicles(arrivingGroup(i,1));
                stoppingNode = obj.stoppingNodes(arrivingDirection(i));
                % the following line requests the ETA of the current
                % vehicle using the stopping node and the ETA of the
                % vehicle ahead (in case there is one)
                arrivingGroup(i,4) = obj.calculateEstimatedTimeOfArrival(vehicle, stoppingNode, ETAcarInFront, 3); % assume an average acceleration of 3             
                
            end
            
            % Build priority group with close vehicles         
            if obj.params.intelligentDecision == 1
                % add all vehicles to priority group that reach the
                % crossroad starting point in time critical estimated time
                % of arrival
                priorityGroup = arrivingGroup(arrivingGroup(:,4) <= obj.params.criticalETA,:);
                
                % if there is no car in the group take the nearest car
                if isempty(priorityGroup)
                    priorityGroup = sortrows(arrivingGroup,4);
                    priorityGroup = priorityGroup(1,:);
                end
            else
                % add all vehicles arriving if FCFS is selected
                priorityGroup = arrivingGroup;
            end
        end
        
        function priorityGroup = calculatePriority(obj, priorityGroup, vehicles)
            % the priority for each car in the first-order vehicle group (priorityGroup) is calculated
            
            if obj.params.intelligentDecision == 1
                for i=1:size(priorityGroup,1)
                    priorityGroupMember = priorityGroup(i,:);
                    vehicle = vehicles(priorityGroupMember(1));
                    if obj.params.energyEquation == 0
                        % time optimized priority formula
                        priority = 1 + vehicle.dynamics.speed * obj.params.alpha;
                    else
                        % energy optimized priority formula
                        priority = 1 + (vehicle.dynamics.speed)^2 * obj.params.alpha2 * vehicle.physics.mass;
                    end
                                        
                    % Vehicles that are blocked by other vehicles in front
                    % should not get a priority that allows them to pass
                    
                    % Check every vehicle that arrived earlier (higher in group)
                    vehiclesInFront = ones(i-1,1);
                    % ignore different starting points (not on same lane)
                    vehiclesInFront(priorityGroup(i,2) ~= priorityGroup(1:i-1,2)) = false;
                    % ignore when also same destination (can pass together)
                    vehiclesInFront(priorityGroup(i,3) == priorityGroup(1:i-1,3)) = false;
                    
                    % Set negative priority so that this vehicle is not
                    % considerd in passing the crossroad
                    if any(vehiclesInFront)
                        priority = -1;
                    end
                    
                    % Add priority to priority group
                    priorityGroup(i,5) = priority;
                end                
            else
                % first come, first serve FCFS
                % use the first entry of priority group, it is the first
                % one that reaches the crossroad
                
                % give every member a negative priority so that they are
                % not used in combinations because of bad priority
                priorityGroup(:,5) = -1;
                
                % only the first one that reached the crossroad should pass
                % when crossroad is empty
                priorityGroup(1,5) = 1;
            end
        end
        
        function stringDirections = convertNumToStringDirection(~, numDirections)
            % intermediate solution: convert direction in numberic numbers
            % (13) to string direction ("NS")
            stringDirections = strings(size(numDirections,1),1);
            for i=1:size(numDirections,1)
                for j=1:size(numDirections,2)
                    switch numDirections(i,j)
                        case 1
                            stringDirections(i) = stringDirections(i) + "N";
                        case 2
                            stringDirections(i) = stringDirections(i) + "E";
                        case 3
                            stringDirections(i) = stringDirections(i) + "S";
                        case 4
                            stringDirections(i) = stringDirections(i) + "W";
                    end
                end
            end
        end
        
%         function numDirections = convertStringToNumDirection(~, stringDirections)
%             % Convert direction in string ("NS") to numeric direction ("13")
%             
%             numDirections = zeros(size(stringDirections,1));
%             for i=1:length(stringDirections)
%                 for j=1:strlength(stringDirections(i))
%                     switch extract(stringDirections(i),j)
%                         case "N"
%                             numDirections(i,j) = 1;
%                         case "E"
%                             numDirections(i,j) = 2;
%                         case "S"
%                             numDirections(i,j) = 3;
%                         case "W"
%                             numDirections(i,j) = 4;
%                     end
%                 end
%             end
%         end

        function vehicleOrders = getVehicleOrders(obj, priorityGroup, leavingGroup)
             % find highest priority crossroad configuration
            % 1. start with the first car in priority group
            % 2. check if car can pass crossroad with current cars on it
            % 3. when yes, add car to a crossroad list and select an other
            % car
            % 4. when no, choose an other car and start with point 2
            % 5. also check next selected car and continue selecting until
            % no car is left in list.
            % 6. Save priority value
            % 7. Do this for every car-combination in list.
            
            % get directions from cars already on crossroad
            if ~isempty(leavingGroup)
                occupiedDirections = obj.convertNumToStringDirection(leavingGroup(:,2:3));
            else
                occupiedDirections = [];
            end
            
            % group of vehicles that want to pass the crossroad in string
            % directions
            % vehicle id | direction ("NS") | priority
            vehicleQueue = [priorityGroup(:,1) obj.convertNumToStringDirection(priorityGroup(:,2:3)) priorityGroup(:,5)];
            
            % priority list contains the vehicle priorities and the
            % matching directions
            priorityList = [vehicleQueue(:,2) vehicleQueue(:,3)];
            

            % get all possible directions that are not blocked by vehicles
            % at crossroad
            possibleDirections = obj.getAllPossibleDirections(vehicleQueue(:,2), occupiedDirections);
            
            % add all possible directions for crossroad to list of possible
            % combinations
            allCombinations = possibleDirections;
            
            % a combination is a row of directions that could applied to
            % the crossroad without overlapping or crossing
            


            % test every possible comibination and when there are more combinations,
            % add them to the array and test them too until there are no more new combinations

            % example combination array:
            % 1. NS
            % 2. NW
            % 3. NS-EW
            % 4. NS-ES
            % 5. NS-...
            % ...
            i=1;
            while i <=length(allCombinations)
                                
                currentCombination = allCombinations(i);
                
                newCombinations = obj.extendCombinations(currentCombination, possibleDirections, occupiedDirections);
                
                % add the new combinations to all combinations
                allCombinations(end+1:end+length(newCombinations)) = newCombinations;
                
                % select next combination entry
                i = i+1;
            end
            
            % Get priority for every combination
            combinationPriorities = obj.assignPriorities(allCombinations, priorityList);
            
            % Select combination with highest priority
            [~, indexHighestPriority] = max(combinationPriorities);
            selectedCombination = allCombinations(indexHighestPriority);
            
            % Define commands for every vehicle (stop, go)
            vehicleOrders = obj.defineVehicleOrders(vehicleQueue(:,1:2), selectedCombination);
            
            % FCFS
            if obj.params.intelligentDecision == 0
                % let no vehicles pass, when there is still one
                % on crossroad
                if ~isempty(occupiedDirections)
                    vehicleOrders(:,2) = 1;
                end
            end
        end
        
        function combinationPriorities = assignPriorities(~, allCombinations, priorityList)
            % assign priorities for every combination using the priority
            % values for the directions from the priority list
            
            combinationPriorities = zeros(size(allCombinations));
            
            for i=1:length(allCombinations)
                % Get all directions from this combination
                directions = split(allCombinations(i),'-');
                % Get priority list for this combination
                currentPriorityList = priorityList;
                
                % Find priority for every direction from one combination
                for j=1:length(directions)
                    priorityIndex = find(directions(j) == currentPriorityList(:,1));
                    combinationPriorities(i) = combinationPriorities(i) + str2double(currentPriorityList(priorityIndex(1),2));
                    % more than one car can drive in one direction
                    % so same directions could be in priority list multiple
                    % times.
                    % Order is not important, because same direction can
                    % every time cross or not cross crossroad and only
                    % highest priority sum is considered, so wrong partial
                    % assignment does not matter.
                    currentPriorityList(priorityIndex(1),:) = []; % delete entry for this combination
                end
            end
        end
        
        function vehicleOrders = defineVehicleOrders(~, vehicleQueue, allowedDirections)
            % define a order for every vehicle from vehicle queue
            %   vehicleQueue        list of vehicle ids and direction they are driving
            %   selectedCombination list of all allowed directions to drive on crossroad
            
            % no allowed direction exists
            if isempty(allowedDirections)
                allowedDirections = "";
            else
                allowedDirections = split(allowedDirections,"-");
            end
            
            % vehicle orders contains car id and stop/go command
            vehicleOrders = [str2double(vehicleQueue(:,1)) ones(size(vehicleQueue,1),1)];
            % make a list with stop and go signals for all cars from vehicleQueue
            for i=1:size(vehicleQueue,1)
                % 'go' when own direction one of allowed directions
                if any(vehicleQueue(i,2) == allowedDirections)
                    vehicleOrders(i,2) = 0; % 0 means 'go' for the car
                else
                    vehicleOrders(i,2) = 1; % 1 means 'stop' for the car
                end
            end
                
        end
        
        function extendedCombinations = extendCombinations(obj, combination, possibleDirections, occupiedDirections)
            % get all possible combinations that extend the input
            % combination with one of the possible directions but do not
            % conflict with the occupied directions
                        
            % split combination in directions to delete them from possible
            % directions
            combinationDirections = split(combination,'-');
            
            % assume that current combination is added to the occupied
            % directions
            assumedOccupied = [occupiedDirections;combinationDirections];
            
            % remove current combination directions from the still possible
            % directions
            for i=1:length(combinationDirections)
                % only remove the first direction there could be more
                % vehicles driving in the same direction
                directionIndex = find(combinationDirections(i) == possibleDirections);
                possibleDirections(directionIndex(1)) = [];
            end
            
            % return if there are no possible directions left
            if isempty(possibleDirections)
                extendedCombinations = [];
                return
            end
            
            % test which directions can still applied when the current
            % combination is choosen
            stillPossibleDirections = obj.getAllPossibleDirections(possibleDirections, assumedOccupied);
            
            % extend old combination with still possible directions
            extendedCombinations = combination + "-" + stillPossibleDirections;          
        end
        
        function possibleDirections = getAllPossibleDirections(obj, vehicleDirections, occupiedDirections)
            % check all vehicle directions if they are still possible
            % without conflicting with one of the occupied directions
            
            % when no direction is occupied, all vehicle directions can
            % work
            if isempty(occupiedDirections)
                possibleDirections = vehicleDirections;
                return
            end
            
            possibleDirections = strings(0,0);
            % check every vehicle direction if conflicting with occupied
            for i=1:length(vehicleDirections)               
                % Check if the direction a car wants to travel isnt
                % conflicting with a direction already occupied.
                if ~obj.checkDirectionsConflicting(vehicleDirections(i), occupiedDirections)
                    possibleDirections(end+1,1) = vehicleDirections(i); %#ok<AGROW>
                end
            end
        end
        
        function conflicting = checkDirectionsConflicting(obj, ownDirection, directions)
            % check if own direction is conflicting with one or more other
            % directions
            % There are two different types of conflicting directions:
            % 1. Direction is connected to own direction
            % 2. Direction is crossing own direction
            
            % get all directions with same start direction
            conflictingStarts = directions(ismember(extract(directions,1), extract(ownDirection,1)));
            
            % get all directions with same destination direction
            conflictingDestinations = directions(ismember(extract(directions,2), extract(ownDirection,2)));
            
            % check if own direction is crossing other lanes that are occupied
            if any(all(ownDirection == string(cat(1,obj.overlappingTable.Properties.VariableNames{:})),2))
                % own direction is part of the overlapping table
                
                % get all occupied overlapping lanes
                conflictingCrossings = directions(ismember(directions, obj.overlappingTable.(ownDirection)));
            else
                conflictingCrossings = [];
            end
            
            % add all conflicting path to one array
            conflictingDirections = [conflictingStarts; conflictingDestinations; conflictingCrossings];
            
            % remove directions that are the SAME as the own direction,
            % because more than one vehicle can travel in same direction
            % without colliding
            conflictingDirections(conflictingDirections == ownDirection,:) = [];
            
            % Check if there are conflicting directions
            if isempty(conflictingDirections)
                conflicting = false;
            else
                conflicting = true;
            end
        end
        
        
        function updateConventionalTrafficLightSystem(obj, currentTime)
            % this function regulate the current traffic for a
            % conventional traffic light system
            
            % 4 traffic states: 
            % 1.  NS direction with straight and right
            % 2.  NS direction with left
            % 3.  EW direction with straight and right
            % 4.  EW direction with left
            
            % Change to waiting
            if currentTime > (obj.stateStartingTime + obj.stateDuration)
                % stop all cars
                waitingState = true;
            else
                waitingState = false;
            end
            
            % Go to next state
            if currentTime > (obj.stateStartingTime + obj.stateDuration + obj.waitingBetweenStates)
                obj.currentState = mod(obj.currentState,4)+1; % go to next state
                obj.stateStartingTime = obj.stateStartingTime + obj.stateDuration + obj.waitingBetweenStates;
            end
            
            % Get orders for every vehicle according to the traffic light
            % state
            obj.vehicleOrders = obj.getConventionalVehicleOrders(obj.arrivingGroup, obj.currentState, waitingState);
            
            % Move all vehicles with GO order to leaving group
            [obj.arrivingGroup, obj.leavingGroup] = obj.moveFromArrivingToLeavingGroup(obj.arrivingGroup, obj.leavingGroup, obj.vehicleOrders);
                   
        end
        
        function vehicleOrders = getConventionalVehicleOrders(obj, arrivingGroup, currentState, waitingState)
            % set the vehicle orders for vehilces using a traffic light at
            % the crossroad
            
            % get vehicles that need orders
            if isempty(arrivingGroup)
                vehicleOrders = [];
                return
            else
                vehicleOrders = [arrivingGroup(:,1) ones(size(arrivingGroup,1),1)];
            end
            
            % stop all cars if it is waiting state
            if waitingState
                vehicleOrders(:,2) = 1;
                return
            end
            
            % convert to string directions
            directions = obj.convertNumToStringDirection(arrivingGroup(:,2:3));
            
            % Traffic State 1: NS direction with straight and right
            if currentState == 1 
                for i = 1:length(directions)
                    % Check if traffic state allows driving for cars
                    if any(directions(i) == ["NS";"NW";"SN";"SE"])
                        % vehicle can drive: 0 == 'go'
                        vehicleOrders(i,2) = 0;
                    else
                        % vehicle has to stop: 0 == 'stop'
                        vehicleOrders(i,2) = 1;
                    end
                end
                
            % Traffic State 2: NS direction with left    
            elseif currentState == 2
                for i = 1:length(directions)
                    % Check if traffic state allows driving for cars
                    if any(directions(i) == ["NE";"SW"])
                        % vehicle can drive: 0 == 'go'
                        vehicleOrders(i,2) = 0;
                    else
                        % vehicle has to stop: 0 == 'stop'
                        vehicleOrders(i,2) = 1;
                    end
                end
                
            % Traffic State 3: EW direction with straight and right
            elseif currentState == 3
                for i = 1:length(directions)
                    % Check if traffic state allows driving for cars
                    if any(directions(i) == ["EN";"EW";"WE";"WS"])
                        % vehicle can drive: 0 == 'go'
                        vehicleOrders(i,2) = 0;
                    else
                        % vehicle has to stop: 0 == 'stop'
                        vehicleOrders(i,2) = 1;
                    end
                end
                
            % Traffic State 4: EW direction with left
            elseif currentState == 4
                for i = 1:length(directions)
                    % Check if traffic state allows driving for cars
                    if any(directions(i) == ["ES";"WN"])
                        % vehicle can drive: 0 == 'go'
                        vehicleOrders(i,2) = 0;
                    else
                        % vehicle has to stop: 0 == 'stop'
                        vehicleOrders(i,2) = 1;
                    end
                end
            end
        end
        
        %% Estimation functions
        
        function timeToReach = calculateEstimatedTimeOfArrival(obj,vehicle,stoppingNode,ETAcarInFront, estimatedAcceleration)
            % calculate the time of arrival at the stopping node of the
            % crossroad for a specific vehicle
            
            maxSpeed = vehicle.dynamics.maxSpeed;
            currentSpeed = vehicle.dynamics.speed;
            
            distToConflictZone = norm(vehicle.dynamics.position -  vehicle.map.get_coordinates_from_waypoint(stoppingNode));
            % vehicle at max speed
            if abs(maxSpeed - currentSpeed)<1
                % if vehicle is not in accleration phase simple constant
                % speed phase calculation
                timeToReach = distToConflictZone/currentSpeed;
            else % vehicle in acceleration phase    
                
                [distanceAccPhase, timeAccPhase] = obj.getAccPhaseParameters(estimatedAcceleration, currentSpeed, maxSpeed); % distance and time to reach maximum speed                
                if distanceAccPhase < distToConflictZone
                    % if the acceleration distance is below the distance to
                    % the conflict zone we have a time accelerating and a
                    % and a time at constant speed
                    timeConstSpeed = (distToConflictZone-distanceAccPhase)/maxSpeed;
                    timeToReach = timeAccPhase + timeConstSpeed;
                else
                    % if the acceleration distance is above the distance to
                    % the conflict zone we can immediately calculate the
                    % time to reach
                    timeToReach = obj.timeToCoverDistanceInAccPhase(currentSpeed, estimatedAcceleration, distToConflictZone);
                    
                end       
            end           
                        
            if (ETAcarInFront ~= 0) && (ETAcarInFront > timeToReach)
                % if there is a car in front and it is slower, vehicle will
                % arrive after the car in front
                timeToReach = ETAcarInFront + obj.params.platooningTimeBehind; % platooninTimeBehind is time distance the vehicle is driving behind
            end
        end
        
        function [distance, time] = getAccPhaseParameters(~, acceleration, startingSpeed, topSpeed)
            % calculate the distance covered and the time needed to reach
            % top speed from starting speed with constant acceleration
            
            deltaV = topSpeed-startingSpeed;          
            t = deltaV/acceleration; % time to accelerate to top speed
            
            distance = startingSpeed*t + 1/2*acceleration*t^2; % distance = way at starting speed + way added through acceleration
            time = t; % time in acceleration phase
        end
        
        function time = timeToCoverDistanceInAccPhase(~, startingSpeed, acceleration, distance)
            % get time to cover a distance while constant accelerating from
            % start speed
            
            % reshape distance formular to get time (get eigenvalues):
            % Equation 8 in NecSys Paper
            % d = v0*t + 1/2*a*t^2
            v0 = startingSpeed;
            a = acceleration;
            
            time = (-v0 + sqrt(v0^2+2*distance*a))/a;
        end
    end
    
end



