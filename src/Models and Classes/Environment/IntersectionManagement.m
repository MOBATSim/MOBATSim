classdef IntersectionManagement < handle
    %IntersectionManagement Manages the intersection passing order of the incoming vehicles. "Intersection" is also referred to as "Crossroad" in other (usually older) parts of the code.
    %   Detailed explanation goes here
    
    % Contributors: Fabian Hart, Mustafa Saraoglu, Johannes Pintscher
    
    properties
        %static
        id                  (1,1) double            % ID of the intersection management unit
        mode                (1,1) IM_Algorithm_Enum % the type of intersection management algorithm     
        startingNodes
        brakingNodes
        stoppingNodes
        leavingNodes
        AIMparams
             % alpha
             % criticalETA
             % platooningTimeBehind
             % energyEquation                     
        overlappingTable    % each column contains the overlapping directions of direction in header
        vehicleOrders       % array with rows containing the vehicle id and the order [ vehicleId order] ( 0 = go, 1 = stop)
        arrivingGroup       % all vehicles arriving at the intersection
        passingGroup        % all vehicles at the intersection
        % Conventional traffic lights
        TLparams
            % stateStartingTime       = 0   % when the new traffic state starts
            % currentState            = 1   % current traffic light state
            % stateDuration           = 7   % the duration for a state to stay active
            % waitingBetweenStates    = 7   % the duration between to states
             
    end
    
    methods
        function obj = IntersectionManagement(id, startingNodes,brakingNodes,stoppingNodes,leavingNodes, mode)
            arguments
                id                                         (1,1) double
                startingNodes                              (1,4) double
                brakingNodes                               (1,4) double
                stoppingNodes                              (1,4) double
                leavingNodes                               (1,4) double
                mode                                       (1,1) IM_Algorithm_Enum = IM_Algorithm_Enum.IntelligentDecision % the mode in which the intersection management algorithm operates
            end
            
            obj.id = id;          
            obj.startingNodes           = startingNodes;
            obj.brakingNodes            = brakingNodes;
            obj.stoppingNodes           = stoppingNodes;
            obj.leavingNodes            = leavingNodes;                         
            obj.mode                    = mode; % the type of Intersection Management Algorithm
            
            %% Parameters for the Intelligent Decision AIM algorithm
            % edit the parameters below to tune the algorithm
            obj.AIMparams.energyEquation = false; % (false -> time optimized approach, true -> energy optimized approach)
            obj.AIMparams.criticalETA = 10; % delta_TTR in our paper
            obj.AIMparams.alpha = 0.072;
            obj.AIMparams.alpha2 = 0.001;
            %platooning control isn't working as expected, therefore a
            %constant platooning time behind the car ahead is set:
            obj.AIMparams.platooningTimeBehind = 0.2;
            
            %% Parameters for the Traffic Light algorithm
            obj.TLparams.stateStartingTime = 0; % when the new traffic state starts
            obj.TLparams.currentState = 1; % current traffic light state
            obj.TLparams.stateDuration = 7; % the duration for a state to stay active
            obj.TLparams.waitingBetweenStates = 7; % the duration between to states
            
            % Table that contains all overlaps paths at the intersection
            obj.overlappingTable = obj.generateOverlappingTable();
        end %Constructor
        
        function overlappingTable = generateOverlappingTable(~)
            % Generates the table that contains all overlaps paths at the intersection
            
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
        
        function vehicleReachesDecisionNode(obj,vehicle)
            % When a vehicle reaches the decision node of an intersection,
            % it has to be registered in the arriving queue
            
            % Select action according to selected mode
            switch(obj.mode)
                case IM_Algorithm_Enum.TrafficLight
                    
                    % PLACEHOLDER
                    
                case IM_Algorithm_Enum.FCFS
                    
                    % PLACEHOLDER
                    
                case IM_Algorithm_Enum.IntelligentDecision
                    
                    % Add vehicle to arriving queue
                    arrivingGroupEntry = obj.makeArrivingGroupEntry(vehicle, obj.startingNodes, obj.leavingNodes);
                    if ~isempty(arrivingGroupEntry)
                        obj.arrivingGroup(end+1,:) = arrivingGroupEntry;
                    end
                    
            end
            
        end
        
        function vehicleReachesIncomingNode(obj, vehicle, vehicles)
            % this function is executed when a vehicle reaches the incoming
            % node so that it has to either GO or STOP.
            
            % Select action according to selected mode
            switch(obj.mode)
                case IM_Algorithm_Enum.TrafficLight
                    
                    % Add vehicle to arriving queue
                    arrivingGroupEntry = obj.makeArrivingGroupEntry(vehicle, obj.brakingNodes, obj.leavingNodes);
                    if ~isempty(arrivingGroupEntry)
                        obj.arrivingGroup(end+1,:) = arrivingGroupEntry;
                    end
                    
                case IM_Algorithm_Enum.FCFS
                    
                    % Add vehicle to arriving queue
                    arrivingGroupEntry = obj.makeArrivingGroupEntry(vehicle, obj.brakingNodes, obj.leavingNodes);
                    if ~isempty(arrivingGroupEntry)
                        obj.arrivingGroup(end+1,:) = arrivingGroupEntry;
                    end
                    % Run the intersection management algorithm
                    [obj.vehicleOrders, obj.arrivingGroup, obj.passingGroup] = obj.run_AIM_Algorithm(vehicles, obj.arrivingGroup, obj.passingGroup);
                    
                case IM_Algorithm_Enum.IntelligentDecision
                    
                    % Run the intersection management algorithm
                    [obj.vehicleOrders, obj.arrivingGroup, obj.passingGroup] = obj.run_AIM_Algorithm(vehicles, obj.arrivingGroup, obj.passingGroup);
                    
            end            
        end
                
        function vehicleLeavesIntersection(obj,vehicle, vehicles)
            % When a vehicle reaches the outgoing node of an intersection, the
            % AIM algorithm has to be executed again and the vehicle has to be
            % deleted from the leaving queue
            
            % Select action according to selected mode
            switch(obj.mode)
                case IM_Algorithm_Enum.TrafficLight
                    
                    if ~isempty(obj.passingGroup)
                        % Remove vehicle from passingGroup
                        obj.passingGroup(obj.passingGroup(:,1)== vehicle.id,:) = [];
                    end
                    
                case IM_Algorithm_Enum.FCFS
                    
                    if ~isempty(obj.passingGroup)                       
                        % Remove vehicle from passingGroup
                        obj.passingGroup(obj.passingGroup(:,1)== vehicle.id,:) = [];
                        
                        % Run the intersection management algorithm
                        [obj.vehicleOrders, obj.arrivingGroup, obj.passingGroup] = obj.run_AIM_Algorithm(vehicles, obj.arrivingGroup, obj.passingGroup);
                    end
                    
                case IM_Algorithm_Enum.IntelligentDecision
                    
                    if ~isempty(obj.passingGroup)                        
                        % Remove vehicle from passingGroup
                        obj.passingGroup(obj.passingGroup(:,1)== vehicle.id,:) = [];
                        
                        % Run the intersection management algorithm
                        [obj.vehicleOrders, obj.arrivingGroup, obj.passingGroup] = obj.run_AIM_Algorithm(vehicles, obj.arrivingGroup, obj.passingGroup);
                    end
                    
            end
            
        end
        
        function [vehicleOrders, arrivingGroup, passingGroup] = run_AIM_Algorithm(obj, vehicles, arrivingGroup, passingGroup)
            % This is the main method for the AIM algorithm.
            
            % Only when a vehicle is arriving execute the algorithm
            if ~isempty(obj.arrivingGroup)
                % get all prioritized vehicles and their priorities
                priorityGroup = obj.buildPriorityGroup(vehicles, arrivingGroup);
                % give every priority vehicle a GO/STOP order
                vehicleOrders = obj.getVehicleOrders(priorityGroup, passingGroup);
                % transfer vehicles that will pass to leaving group
                [arrivingGroup, passingGroup] = obj.moveFromArrivingToPassingGroup(arrivingGroup, passingGroup, vehicleOrders);
            else
                vehicleOrders = [];
            end
        end
        
        function entry = makeArrivingGroupEntry(~, vehicle, startingNodes, leavingNodes)
            % add a vehicle to arriving group
                
                entry = [];
                % check that cars destination is not  before the intersection
                if length(vehicle.pathInfo.path)>3
                    arrivingDirection = find(vehicle.pathInfo.lastWaypoint == startingNodes); % from which direction the car is coming 1=N, 2=E, 3=S, 4=W
                    
                    
                    index = find(ismember(vehicle.pathInfo.path,leavingNodes));                   

                    try
                        leavingDirection = find(vehicle.pathInfo.path(index(1)) == leavingNodes); % in which direction the car is going 1=N, 2=E, 3=S, 4=W
                        
                    catch
                        
                        disp('Intersection management error');
                        
                    end
                    
                    % arring queue definition (one vehicle entry):
                    % | vehicle id | arriving direction | leaving direction |
                    entry = [vehicle.id arrivingDirection leavingDirection];
                end
        end

        function [arrivingGroup, passingGroup] = moveFromArrivingToPassingGroup(~, arrivingGroup, passingGroup, vehicleOrders)
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
            
            % move every passing vehicle from arrivingGroup to passingGroup
            entriesLeaving = arrivingGroup(any(arrivingGroup(:,1) == passingVehicles',2),:);
            % add leaving to passingGroup
            passingGroup(end+1:end+size(entriesLeaving,1),:) = entriesLeaving;
            % remove leaving from arrivingGroup
            arrivingGroup(any(arrivingGroup(:,1) == passingVehicles',2),:) = [];                
            
        end                
        
        
        function priorityGroup = buildPriorityGroup(obj, vehicles, arrivingGroup)
            % get and add priorities to the priority group, vehicles in this group are
            % considered by the algorithm
            
            % get priority group by looking at vehicles close to the intersection
            priorityGroup = obj.getPriorityGroup(vehicles, arrivingGroup);
            
            % calculate priorities for priority group
            priorityGroup = obj.calculatePriority(priorityGroup, vehicles); 
        end
        
        
        function priorityGroup = getPriorityGroup(obj, vehicles, arrivingGroup) % get first order vehicle group which only is considered in the algorithm
            % in this function we delimitate vehicles with close ETAs. The
            % parameter we are using is criticalETA (in our paper: deltaETA)
            
            % directions from that the vehicles enter the intersection
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
            
            % Build priority group in dependency of the selected mode
            switch(obj.mode)
                case IM_Algorithm_Enum.FCFS
                    
                    % add all arriving vehicles
                    priorityGroup = arrivingGroup;
                    
                case IM_Algorithm_Enum.IntelligentDecision
                    
                    % add all vehicles to priority group that reach the
                    % Intersection starting point in time critical estimated time
                    % of arrival
                    priorityGroup = arrivingGroup(arrivingGroup(:,4) <= obj.AIMparams.criticalETA,:);
                    
                    % if there is no car in the group take the nearest car
                    if isempty(priorityGroup)
                        priorityGroup = sortrows(arrivingGroup,4);
                        priorityGroup = priorityGroup(1,:);
                    end
                    
            end
        end
        
        function priorityGroup = calculatePriority(obj, priorityGroup, vehicles)
            % the priority for each car in the first-order vehicle group (priorityGroup) is calculated
            
            switch(obj.mode)
                
                case IM_Algorithm_Enum.FCFS
                    % use the first entry of priority group, it is the first
                    % one that reaches the intersection
                    
                    % give every member a negative priority so that they are
                    % not used in combinations because of bad priority
                    priorityGroup(:,5) = -1;
                    
                    % only the first one that reached the intersection should pass
                    % when the intersection is empty
                    priorityGroup(1,5) = 1;
                    
                case IM_Algorithm_Enum.IntelligentDecision
                    
                    for i=size(priorityGroup,1):-1:1
                        priorityGroupMember = priorityGroup(i,:);
                        vehicle = vehicles(priorityGroupMember(1));
                        if obj.AIMparams.energyEquation == 0
                            % time optimized priority formula
                            priority = 1 + vehicle.dynamics.speed * obj.AIMparams.alpha;
                        else
                            % energy optimized priority formula
                            priority = 1 + (vehicle.dynamics.speed)^2 * obj.AIMparams.alpha2 * vehicle.physics.mass;
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
                        % considerd in passing the intersection:
                        % Otherwise the front vehicle might get a "stop" command
                        % and the rear vehicle a "go" which blocks the intersection
                        if any(vehiclesInFront)
                            priority = -1;
                        end
                        
                        % Add priority to priority group
                        priorityGroup(i,5) = priority;
                    end
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

        function vehicleStopCommands = getVehicleOrders(obj, priorityGroup, passingGroup)
            % Find highest priority intersection passing order
            % 1. start with the first car in priority group
            % 2. check if the vehicle can pass the intersection without colliding with the current vehicles in the intersection
            % 3. If yes, add the vehicle to a intersection list and select another vehicle
            % 4. If no, choose another vehicle and re-evaluate point 2.
            % 5. Check the next selected vehicle and continue selecting until no vehicle is left in list.
            % 6. Save the priority value
            % 7. Do this for every vehicle-combination in the list.
            
            % Get directions from the vehicle already in the intersection conflict zone
            if ~isempty(passingGroup)
                occupiedDirections = obj.convertNumToStringDirection(passingGroup(:,2:3));
            else
                occupiedDirections = [];
            end
            
            % group of vehicles that want to pass the intersection in string
            % directions
            % vehicle id | direction ("NS") | priority
            vehicleQueue = [priorityGroup(:,1) obj.convertNumToStringDirection(priorityGroup(:,2:3)) priorityGroup(:,5)];
            
            % priority list contains the vehicle priorities and the
            % matching directions
            priorityList = [vehicleQueue(:,2) vehicleQueue(:,3)];
            
            % get all possible directions that are not blocked by vehicles
            % at the intersection (exclude rear vehicles as they cannot
            % move if the front vehicles' path directions are not allowed)
            idx_rearVehicles = vehicleQueue(:,3) == "-1";
            vehicleQueueWithoutRearVehicles = vehicleQueue;
            vehicleQueueWithoutRearVehicles(idx_rearVehicles,:) = [];
            possibleDirections = obj.getAllPossibleDirections(vehicleQueueWithoutRearVehicles(:,2), occupiedDirections);
            
            % add all possible directions for the intersection to list of possible
            % combinations
            allCombinations = possibleDirections;
            
            % a combination is a row of directions that could applied to
            % the intersection without overlapping or crossing
            


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

            %--------------------------------------------------------------
            % Verify the selected AIM "go" commands by generating a
            % verified Transition System for the allowed directions
            %--------------------------------------------------------------
            if obj.id == 1 && ~isempty(selectedCombination) % Verification only for the 4x4 intersection in the west with id:1 and if there is a new decision
                
                % Construct the current model for the safety verification
                if ~isempty(obj.arrivingGroup)
                    atInput = obj.convertNumToStringDirection(obj.arrivingGroup(:,2:3));                    
                else
                    atInput =[];
                end
                
                if ~isempty(obj.passingGroup)
                    atPassing = obj.convertNumToStringDirection(obj.passingGroup(:,2:3));
                else
                    atPassing =[];  
                end
                
                
                % The paths that overlap in intersection id:1
                pathConflicts = ["ne", "ew";
                    "ne", "wn";
                    "ne", "es";
                    "ne", "sn";
                    "ne", "we";
                    "ne", "se";
                    "ns", "ew";
                    "ns", "sw";
                    "ns", "wn";
                    "ns", "we";
                    "ns", "ws";
                    "ns", "es";
                    "nw", "ew";
                    "nw", "sw";
                    "es", "sn";
                    "es", "sw";
                    "es", "we";
                    "es", "ws";
                    "ew", "sn";
                    "ew", "wn";
                    "ew", "sw";
                    "en", "sn";
                    "en", "wn";
                    "sw", "we";
                    "sw", "wn";
                    "sn", "we";
                    "sn", "wn";
                    "se", "we"];
                             
                % To further test the safety filter (verification of the "go" actions taken by the AIM) one can
                % overwrite the values of "atPassing" and "atInput" with conflicting paths
                % in order to see if the "verified" variable results "false".
                [verified, currentSafeActions] = TScrossroad.verifyPlannedAIMActions(lower(atInput), lower(atPassing), pathConflicts, selectedCombination);
                
                if verified
                    % If safety of the planned AIM command is verified -> Apply AIM commands
                    
                elseif ~verified
                    % If not verified, choose a safe action available by the safety filter: 
                    disp("Intersection Management Algorithm at IM" +obj.id+ " wanted to take a forbidden action that could have caused a collision!");
                    disp("At time:" + get_param('MOBATSim','SimulationTime')+"s" );
                    disp("Vehicle incoming directions: " + sprintf('%s', atInput));
                    disp("Vehicle passing directions: " + sprintf('%s', atPassing));
                    disp("Forbidden Go Action(s):" + selectedCombination);
                    
                    safeDirections = strings(1,size(currentSafeActions,2));
                    safeActionByFilter = "";
                    
                    for k = 1:size(currentSafeActions,1)
                        safeDirections(k) = upper(eraseBetween(currentSafeActions(k),1,1));
                        if ~isempty(atPassing)
                            if any(ismember(safeDirections(k),atPassing))
                                safeActionByFilter = safeDirections(k);
                                break
                            end
                        
                        elseif ~isempty(atInput)
                            if any(ismember(safeDirections(k),atInput))
                                safeActionByFilter = safeDirections(k);
                                break
                            end
                                                        
                        end
                    end
                    
                    if isempty(safeActionByFilter)
                        safeActionByFilter = safeDirections(1);
                    end
                    
                    disp("Safety filter for AIM allows the following action(s): "+ safeActionByFilter);   
                    selectedCombination= safeActionByFilter;

                end
                
            end
            %--------------------------------------------------------------
            % Determine the vehicles to receive commands from the AIM
            vehiclesToReceiveCommands = []; 
            for v = 1:size(obj.arrivingGroup,1)
                vId = obj.arrivingGroup(v,1);
                vDir = obj.convertNumToStringDirection(obj.arrivingGroup(v,2:3));
                % Array of Vehicles: [ID, incoming-outgoing directions]
                vehiclesToReceiveCommands = [vehiclesToReceiveCommands; num2str(vId) vDir];
            end
            % Send "stop" commands for every vehicle (1 = stop, 0 = go in this section) 
            vehicleStopCommands = obj.defineVehicleOrders(vehiclesToReceiveCommands, selectedCombination);            
            

            %% FCFS
            if obj.mode == IM_Algorithm_Enum.FCFS
                % let no vehicles pass, when there is still one at the intersection
                if ~isempty(occupiedDirections)
                    vehicleStopCommands(:,2) = 1;
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
                    % every time cross or not cross the intersection and only
                    % highest priority sum is considered, so wrong partial
                    % assignment does not matter.
                    currentPriorityList(priorityIndex(1),:) = []; % delete entry for this combination
                end
            end
        end
        
        function vehicleOrders = defineVehicleOrders(~, vehicleQueue, allowedDirections)
            % define a order for every vehicle from vehicle queue
            %   vehicleQueue        list of vehicle ids and direction they are driving
            %   selectedCombination list of all allowed directions to drive at the intersection
            
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
                    vehicleOrders(i,2) = 0; % 0 means 'go' for the vehicle
                else
                    vehicleOrders(i,2) = 1; % 1 means 'stop' for the vehicle
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
            % (A very fast vehicle might a very slow vehicle that is in the next route at the intersection, 
            % safety first so the fast vehicle stops if passing from the same incoming node)
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
            
            % add all conflicting paths to one array
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
            if currentTime > (obj.TLparams.stateStartingTime + obj.TLparams.stateDuration)
                % stop all cars
                waitingState = true;
            else
                waitingState = false;
            end
            
            % Go to next state
            if currentTime > (obj.TLparams.stateStartingTime + obj.TLparams.stateDuration + obj.TLparams.waitingBetweenStates)
                obj.TLparams.currentState = mod(obj.TLparams.currentState,4)+1; % go to next state
                obj.TLparams.stateStartingTime = obj.TLparams.stateStartingTime + obj.TLparams.stateDuration + obj.TLparams.waitingBetweenStates;
            end
            
            % Get orders for every vehicle according to the traffic light
            % state
            obj.vehicleOrders = obj.getConventionalVehicleOrders(obj.arrivingGroup, obj.TLparams.currentState, waitingState);
            
            % Move all vehicles with GO order to leaving group
            [obj.arrivingGroup, obj.passingGroup] = obj.moveFromArrivingToPassingGroup(obj.arrivingGroup, obj.passingGroup, obj.vehicleOrders);
                   
        end
        
        function vehicleOrders = getConventionalVehicleOrders(obj, arrivingGroup, currentState, waitingState)
            % set the vehicle orders for vehilces using a traffic light at the intersection
            
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
            % intersection for a specific vehicle
            
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
                timeToReach = ETAcarInFront + obj.AIMparams.platooningTimeBehind; % platooninTimeBehind is time distance the vehicle is driving behind
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



