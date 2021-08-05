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
        trafficStateLookupTable
        params
             % deltaStatePriority
             % alpha
             % criticalDeltaTTR
             % intelligentDecision
             % platooningTimeBehind
             % conventionalTrafficLights
             % energyEquation
        arrivingQueue   = []    % this is named Waiting Queue in our  paper
        leavingQueue    = []
        priorityGroup           % this is named first-order vehicle group in our  paper
                
        trafficState    = 0     %this is named CSG in our paper   
        brakingFlagArray % Array with the following structure: [ carId brakingFlag]
                         %                                     [ carId brakingFlag]
                         % brakingFlag: 1 -> Stop!, 0 -> Go!
        dataLog
            % traffic state
        
        % Test new kind of checking:
        overlappingTable    % each column contains the overlapping directions of direction in header
        vehicleOrders       % array with rows containing the vehicle id and the order ( 0 = go, 1 = stop)
        arrivingGroup       % all vehicles arriving at the crossroad
        leavingGroup        % all vehicles at the crossroad
        % Conventional traffic lights
        stateStartingTime       = 0   % when the new traffic state has started
        currentState            = 1   % current traffic light state
        stateDuration           = 5   % the duration a state stays active TODO: fine tune this
        waitingBetweenStates    = 5  % the duration between to states TODO: fine tune this
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
                configurations.intelligentDecision         (1,1) logical   = true   % 0 for FCFS, 1 for our algorithm
                configurations.energyEquation              (1,1) logical   = false  % 0 for time optimized approach, 1 for energy optimized approach
            end
            obj.id = id;          
            obj.startingNodes = startingNodes;
            obj.brakingNodes = brakingNodes;
            obj.stoppingNodes = stoppingNodes;
            obj.leavingNodes = leavingNodes;
            
            obj.trafficStateLookupTable = obj.generateLookupTable();
            
            
            % edit all the parameters at this point to change the algorithm
            obj.params.deltaStatePriority = 2; % delta_p in our paper
            obj.params.criticalDeltaTTR = 1.5; % delta_TTR in our paper
            obj.params.alpha = 0.072;
            obj.params.alpha2 = 0.001;
            %platooning control isn't working as expected, therefore a
            %constant platooning time behind the car ahead is set:
            obj.params.platooningTimeBehind = 0.2;           
            obj.params.intelligentDecision          = configurations.intelligentDecision;
            obj.params.conventionalTrafficLights    = configurations.conventionalTrafficLights;
            obj.params.energyEquation               = configurations.energyEquation;
            
            
            if obj.params.intelligentDecision == 0 % TODO: think about usefullness in new algorithm
                obj.params.criticalDeltaTTR = 1000;  % for FCFS we set delta TTR on 1000 to consider all(!) vehicles in the surrounding of the crossroad
            end

            if obj.params.energyEquation == 1
                obj.params.deltaStatePriority = 9999; % for energy optimized approach we set delta_p to 9999 (see in diploma thesis for explanation)
            end
           
            
            obj.dataLog.trafficState = []; 
            % Test
            obj.overlappingTable = obj.generateOverlappingTable();
        end %Constructor
        
        function overlappingTable = generateOverlappingTable(~)
            % table containing all overlapping lanes on crossroad
            
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
        
        function trafficStateLookupTable = generateLookupTable(~)
            
                % Lookup Table for a 4-road-crossing with 7 different CSGs 
                
                % The definition is
                
                % to     N E S W
                % from N x x x x
                % from E x x x x
                % from S x x x x
                % from W x x x x
                
                % Vehicles can drive in 9 different scenarios according to
                % the table from row to column without collision
                
                trafficStateLookupTable={[
                    [0 0 0 0]
                    [1 0 0 1]
                    [0 0 0 0]
                    [0 1 1 0] ], ... E-N E-W W-S W-E
                    [
                    [0 0 1 1]
                    [0 0 0 0]
                    [1 1 0 0]
                    [0 0 0 0] ], ... N-W N-S S-E S-N
                    [
                    [0 0 0 1]
                    [0 0 0 0]
                    [0 0 0 0]
                    [1 1 1 0] ], ... W-N W-E W-S N-W
                    [
                    [0 1 1 1]
                    [1 0 0 0]
                    [0 0 0 0]
                    [0 0 0 0] ], ... N-E N-S N-W E-N
                    [
                    [0 0 0 0]
                    [1 0 1 1]
                    [0 1 0 0]
                    [0 0 0 0] ], ... E-S E-W E-N S-E
                    [
                    [0 0 0 0]
                    [0 0 0 0]
                    [1 1 0 1]
                    [0 0 1 0] ], ... S-W S-N S-E W-S
                    [    
                    [0 0 0 1]
                    [1 0 0 0]
                    [0 1 0 0]
                    [0 0 1 0] ], ... % N-W, E-N, S-E, W-S
                    ... % 2 new scenarios (2 cars from opposite roads turning
                    ... % left at the same time)
                    [    
                    [0 0 0 1]
                    [0 0 1 0]
                    [0 1 0 0]
                    [1 0 0 0] ], ... % N-W, E-S, S-E, W-N
                    [    
                    [0 1 0 0]
                    [1 0 0 0]
                    [0 0 0 1]
                    [0 0 1 0] ], ... % N-E, E-N, S-W, W-S
                    };
           
        end
        
        function carReachesCrossroad(obj,car,startingNode)
            % when a car is reaching the starting node of a crossroad, the
            % car has to be registrated in the arriving queue
            
            if obj.params.intelligentDecision == 1 % for normal algorithm
                
                % check that cars destination is not  before crossroad
                if length(car.pathInfo.path)>3
                    arrivingDirection = find(startingNode == obj.startingNodes); % from which direction the car is coming 1=N, 2=E, 3=S, 4=W
                    
                    
                    index = find(ismember(car.pathInfo.path,obj.leavingNodes));                   

                    try
                        leavingDirection = find(car.pathInfo.path(index(1)) == obj.leavingNodes); % in which direction the car is going 1=N, 2=E, 3=S, 4=W
                        
                    catch
                        
                        disp('crossroad error');
                        
                    end
                    
                    % arring queue definition:
                    % [car id; arrving direction; leaving direction; ETA]
                    % the cars will be added in this style to the arriving
                    % queue. One line is one car.
                    obj.arrivingQueue(end+1,:) = [car.id arrivingDirection leavingDirection 0]; % ETA = 0, not calculated jet
                    % Test for new algorithm TODO                   
                    %direction = obj.convertNumToStringDirection( [arrivingDirection leavingDirection]);
                    obj.arrivingGroup(end+1,:) = [car.id arrivingDirection leavingDirection 0]; % ETA = 0, not calculated jet
                end
            end
            
            
        end
        
        function carReachesBrakingPoint(obj, vehicle, vehicles, brakingNode, currentTime)
            % this function is executed when a car reaches the braking
            % point. Now the main algorithm has to be executed to derive
            % the optimal CSG
            
            % When we are in FCFS the car hasn't been added in the arriving
            % queue yet. This is done here.
            if obj.params.intelligentDecision == 0
                
                arrivingDirection = find(brakingNode == obj.brakingNodes);
                leavingDirection = find(vehicle.pathInfo.path(3) == obj.leavingNodes);
                TTR = 190/vehicle.dynamics.speed + currentTime;
                
                obj.arrivingQueue(end+1,:) = [vehicle.id arrivingDirection leavingDirection TTR];
                
            end
            
            if ~isempty(obj.arrivingQueue)
                if obj.params.conventionalTrafficLights == 0
                    % now the main part of the algorithm is executed. It is
                    % built up by the two following functions
                    [obj.priorityGroup, obj.arrivingQueue] = obj.buildPriorityGroup(vehicles, obj.arrivingQueue); % get and calculate priorities for vehicles near to crossroad
                    obj.deriveMPCSG(currentTime); % derive the optimal CSG for this group of cars
                end
            end
            
            % Test: new algorithm
            if ~isempty(obj.arrivingGroup)
                if obj.params.conventionalTrafficLights == 0
                    % now the main part of the algorithm is executed. It is
                    % built up by the two following functions
                    [obj.priorityGroup, obj.arrivingGroup] = obj.buildPriorityGroup(vehicles, obj.arrivingGroup); % get and calculate priorities for vehicles near to crossroad
                    obj.vehicleOrders = obj.getVehicleOrders(obj.priorityGroup, obj.leavingGroup); % get a order for every vehicle in priority group
                    [obj.arrivingGroup, obj.leavingGroup] = obj.moveFromArrivingToLeavingGroup(obj.arrivingGroup, obj.leavingGroup, obj.vehicleOrders); % transfer vehicles that will pass crossroad
                end
            end
            % the next step is to update the braking flag array according
            % to the new CSG
            obj.brakingFlagArray(end+1,:) = [vehicle.id 1]; % this is just to add the new car to the braking flag array
            
            if obj.trafficState ~=0
                obj.updateBrakingFlagArray;  % this function updates the braking flag array if the current CSG is not zero
            end
            
            % now we have to check what the new braking flag for the
            % new car is. If it is zero, we have to remove the car from
            % the arriving queue
            brakingFlag = obj.brakingFlagArray(obj.brakingFlagArray(:,1)==vehicle.id,2);          
            
            if brakingFlag == 0
                
                % remove car from waiting queue for specific cardinal direction
                obj.arrivingQueue(obj.arrivingQueue(:,1)==vehicle.id,:)=[];
                
            end
                        
        end
        
        function carLeavesCrossroad(obj,vehicle, vehicles, currentTime)
            % when a vehicle reaches the leaving node of a crossroad, the
            % main algorithm has to be executed again and the car has to be
            % deleted from the leaving queue
            
            
 
            if ~isempty(obj.leavingQueue)
                                                           
                obj.leavingQueue(obj.leavingQueue(:,1)==vehicle.id,:)=[]; % remove car from leaving queue
                obj.brakingFlagArray(obj.brakingFlagArray(:,1)==vehicle.id,:)=[]; % remove car from braking flag array
                
                
                % when there are still cars in the arriving queue then the
                % main algorithm has to be executed again
                if ~isempty(obj.arrivingQueue)
                    if obj.params.conventionalTrafficLights == 0
                        [obj.priorityGroup, obj.arrivingQueue] = obj.buildPriorityGroup(vehicles, obj.arrivingQueue); % get and calculate priorities for vehicles near the crossroad
                        obj.deriveMPCSG(currentTime); % derive the optimal CSG for this group of cars
                    end
                end
            end
            
            if ~isempty(obj.leavingGroup)
                
                % Remove vehicle from leavingGroup: Test TODO
                obj.leavingGroup(obj.leavingGroup(:,1)== vehicle.id,:) = [];
                if obj.params.conventionalTrafficLights == 0
                    % Remove vehicle from arrivingGroup, because there is
                    % no leaving group
                    obj.arrivingGroup(obj.arrivingGroup(:,1)== vehicle.id,:) = [];
                end
                % Test TODO
                % when there are still cars in the arriving queue then the
                % main algorithm has to be executed again
                if ~isempty(obj.arrivingGroup)
                    if obj.params.conventionalTrafficLights == 0
                        [obj.priorityGroup, obj.arrivingGroup] = obj.buildPriorityGroup(vehicles, obj.arrivingGroup); % get and calculate priorities for vehicles near the crossroad
                        obj.vehicleOrders = obj.getVehicleOrders(obj.priorityGroup, obj.leavingGroup); % get a order for every vehicle in priority group
                        [obj.arrivingGroup, obj.leavingGroup] = obj.moveFromArrivingToLeavingGroup(obj.arrivingGroup, obj.leavingGroup, obj.vehicleOrders); % transfer vehicles that will pass crossroad
                    end
                end
            end
        end

        function [arrivingGroup, leavingGroup] = moveFromArrivingToLeavingGroup(~, arrivingGroup, leavingGroup, vehicleOrders)
            % Move all vehicles that got a 'go' command from the arriving
            % group to the leaving group
            
            % get all vehicles that have a 'go' command to pass the
            % crossroad
            passingVehicles = vehicleOrders(vehicleOrders(:,2) == 0,1);
            
            % nothing changed
            if isempty(passingVehicles)
                return
            end
            
            % move every passing vehicle from arrivingGroup to leavingGroup
            % TODO: maybe use vectorization
            for vehicle = passingVehicles'
                % move entry
                leavingGroup(end+1,:) = arrivingGroup(arrivingGroup(:,1) == vehicle,:);
                % delete from arriving group
                arrivingGroup(vehicle==arrivingGroup(:,1),:) = [];
            end
                
            
        end
                
        function updateBrakingFlagArray(obj)
            % this function updates the brakingflag Array due to a change
            % of the current CSG
            
            if obj.params.intelligentDecision == 1
                if ~isempty(obj.brakingFlagArray)
                    for k=1: size(obj.brakingFlagArray,1)
                        if obj.brakingFlagArray(k,2) == 1
                            carId = obj.brakingFlagArray(k,1);
                            carDirection =  obj.arrivingQueue(obj.arrivingQueue(:,1)==carId,2:3);

                            % check if theres a car ahead who has to stop
                            % and is thus blocking the street ahead
                            try
                                index = find(obj.arrivingQueue(:,2)==carDirection(1));
                                Err_expression = index(1) >= find(obj.arrivingQueue(:,1)==carId);
                            catch
                                disp('crossroad error 2');
                                Err_expression = false;
                            end
                            

                            % if no blocking car ahead check traffic state
                            if Err_expression
                                % now we can get the braking flag according
                                % to the CSGs we have defined in the lookup
                                % table
                                newBrakingFlag = ~obj.trafficStateLookupTable{obj.trafficState}(carDirection(1),carDirection(2));
                                if newBrakingFlag == 0
                                    obj.brakingFlagArray(k,2) = newBrakingFlag;
                                    obj.leavingQueue(end+1,:) = [carId carDirection];

                                    % when braking flag is zero we can remove car from waiting queue for specific cardinal direction
                                    obj.arrivingQueue(obj.arrivingQueue(:,1)==carId,:)=[];                            

                                end
                            end
                        end
                    end

                end
            else
                % this routine is for FCFS
                if ~isempty(obj.brakingFlagArray)
                    for k=1: size(obj.brakingFlagArray,1)
                        if obj.brakingFlagArray(k,2) == 1
                             carId = obj.brakingFlagArray(k,1);
                             carDirection =  obj.arrivingQueue(obj.arrivingQueue(:,1)==carId,2:3);
                             newBrakingFlag = ~obj.trafficStateLookupTable{obj.trafficState}(carDirection(1),carDirection(2));
                                if newBrakingFlag == 0
                                    obj.brakingFlagArray(k,2) = newBrakingFlag;
                                    obj.leavingQueue(end+1,:) = [carId carDirection];

                                    % remove car from waiting queue for specific cardinal direction
                                    obj.arrivingQueue(obj.arrivingQueue(:,1)==carId,:)=[];                            

                                else
                                    break;
                                end
                        end
                    end
                end
            end
        end
        
        function updateCustomBrakingFlagArray(obj, customBrakingFlags)
            % this function is used to upadte the braking Flag Array with
            % custom CSGs. This function is used in the traffic light
            % system function.
            % the function is pretty similar to the updateBrakingFlagArray
            % function

            if ~isempty(obj.brakingFlagArray)
                for k=1: size(obj.brakingFlagArray,1)
                    if obj.brakingFlagArray(k,2) == 1
                        carId = obj.brakingFlagArray(k,1);
                        carDirection =  obj.arrivingQueue(obj.arrivingQueue(:,1)==carId,2:3);
                        
                        % check if theres a blocking car ahead
                        index = find(obj.arrivingQueue(:,2)==carDirection(1));
                        
                        % if no blocking car ahead check traffic state
                        if index(1) >= find(obj.arrivingQueue(:,1)==carId)
                            newBrakingFlag = ~customBrakingFlags(carDirection(1),carDirection(2));
                            if newBrakingFlag == 0
                                obj.brakingFlagArray(k,2) = newBrakingFlag;
                                obj.leavingQueue = [obj.leavingQueue;
                                    [carId carDirection]];
                                
                                % remove car from waiting queue for specific cardinal direction
                                obj.arrivingQueue(obj.arrivingQueue(:,1)==carId,:)=[];
                                
                            end
                        end
                    end
                end
                
            end
        end
        
        function [priorityGroup, arrivingQueue] = buildPriorityGroup(obj, vehicles, arrivingQueue)
            % get and update the priority group, vehicles in this group are
            % considered by the algorithm
            
            % get priority group by looking at vehicles close to crossroad
            [priorityGroup, arrivingQueue] = obj.getPriorityGroup(vehicles, arrivingQueue);
            
            % calculate priorities for priority group
            priorityGroup = obj.calculatePriority(priorityGroup, vehicles); 
        end
        
        function [priorityGroup, arrivingQueue] = getPriorityGroup(obj, vehicles, arrivingQueue) % get first order vehicle group which only is considered in the algorithm
            % in this function we delimitate vehicles with close ETAs. The
            % parameter we are using is criticalDeltaTTR (in our paper: deltaETA)
            
            % we loop through all vehicles to get the estimated time of
            % arrival at the conflict zone (ETA)
            for i=1:size(arrivingQueue,1)          
                
                % to get exact estimations of all vehicles in the arriving
                % queue, it has to be checked if there is another vehicle
                % ahead. The next lines evaluate if there is another car
                % ahead and if so then the ETA of the car ahead is saved in
                % 'ETAcarInFront'. If there is no other car ahead
                % ETAcarInFront = 0.                
                tempArrivingQueue = arrivingQueue(1:i-1,:);
                
                if any(tempArrivingQueue(:,2)== arrivingQueue(i,2))
                    rowIndexCarInFront = find(tempArrivingQueue(:,2)== arrivingQueue(i,2));
                    ETAcarInFront = arrivingQueue(rowIndexCarInFront(end),4);
                else
                    ETAcarInFront = 0;
                end
                
                % get the current vehicle of arriving queue
                vehicle = vehicles(arrivingQueue(i,1));
                stoppingNode = obj.stoppingNodes(arrivingQueue(i,2));
                % the following line requests the ETA of the current
                % vehicle using the stopping node and the ETA of the
                % vehicle ahead (in case there is one)
                arrivingQueue(i,4) = obj.calculateEstimatedTimeOfArrival(vehicle, stoppingNode, ETAcarInFront, 3); % assume an average acceleration of 3             
                
            end
            
            % now we have updated the estimated time of arrival for all
            % vehicles in the arriving queue. The next step is to
            % delimitate vehicles with close ETA by using the parameter
            % criticalDeltaTTR
            
            priorityGroup = sortrows( arrivingQueue,4); % priority queue has the same structure as the arriving queue
            difference = diff(priorityGroup(:,4)) < obj.params.criticalDeltaTTR; % TODO: Error when vehicle is too close behind an other,
            % but without triggering criticalDeltaTTR is can make it over
            % the crossroad without getting the brake flag
            
%             % find first order priority group seperated by critical
%             % delta ttr from other vehicles
%             if ~isempty(find( difference == 0, 1, 'first'))
%                 carIds = priorityGroup(1:find( difference == 0, 1, 'first'),1);
%                 % find the lines with matching carIds
%                 priorityGroup = priorityGroup(ismember(priorityGroup(:,1),carIds),:);
%             end
            
            priorityGroup = priorityGroup(priorityGroup(:,4) <= 25,:); % TODO: quickfix
            % add least one arriving car should be in priority group
            if isempty(priorityGroup)
                priorityGroup = sortrows( obj.arrivingQueue,4);
                priorityGroup = priorityGroup(1,:);
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
                occupiedDirections = obj.convertNumToStringDirection(leavingGroup(:,2:3)); % TODO: make this conversion unneccessaty
            else
                occupiedDirections = [];
            end
            
            % group of vehicles that want to pass the crossroad in string
            % directions TODO: dont convert, make priority group also in
            % string
            % vehicle id | direction ("NS") | priority
            vehicleQueue = [priorityGroup(:,1) obj.convertNumToStringDirection(priorityGroup(:,2:3)) priorityGroup(:,5)]; % TODO: check for non empty
            
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
            % 3. NS | EW
            % 4. NS | ES
            % 5. NW | ...
            % ...
            i=1;
            while i <=size(allCombinations,1)
                                
                currentCombination = allCombinations(i,:);
                
                newCombinations = obj.extendCombinations(currentCombination, possibleDirections, occupiedDirections);
                
                % add the new combinations to all combinations
                allCombinations(end+1:end+size(newCombinations,1),1:size(newCombinations,2)) = newCombinations;
                
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
                for direction = directions'
                    priorityIndex = find(direction == currentPriorityList(:,1)); % TODO: maybe use only one priority value per direction
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
            assumedOccupied = [occupiedDirections;combinationDirections'];
            
            % remove current combination directions from the still possible
            % directions TODO: when the number of same direction does not
            % matter, this could be a one liner.
            for direction = combinationDirections'
                % only remove the first direction there could be more
                % vehicles driving in the same direction
                directionIndex = find(direction == possibleDirections);
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
            for direction = vehicleDirections'               
                % Check if the direction a car wants to travel isnt
                % conflicting with a direction already occupied.
                if ~obj.checkDirectionsConflicting(direction, occupiedDirections)
                    possibleDirections(end+1) = direction; %#ok<AGROW>
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
                conflictingCrossings = directions(ismember(string(directions), obj.overlappingTable.(ownDirection)),:);
            else
                conflictingCrossings = [];
            end
            
            % add all conflicting path to one array
            conflictingDirections = [conflictingStarts; conflictingDestinations; conflictingCrossings];
            
            % remove directions that are the SAME as the own direction,
            % because more than one vehicle can travel in same direction
            % without colliding
            conflictingDirections(string(conflictingDirections) == ownDirection,:) = [];
            
            % Check if there are conflicting directions
            if isempty(conflictingDirections)
                conflicting = false;
            else
                conflicting = true;
            end
        end
        
        function deriveMPCSG(obj, currentTime)
            % this function dervies the maximum priority CSG by using the
            % first-order vehicle group                       
            
            % finding the optimal traffic state for the priority car group
            vector0 = zeros(length(obj.trafficStateLookupTable),1);
            vector1000 = 1000*ones(length(obj.trafficStateLookupTable),1);
            
            
            trafficStatePriority = [vector0, vector1000, vector0]; 
            % this array has the following strucutre for seven CSGs. Each
            % line defines one CSG.
            % [0 1000 0]
            % [0 1000 0]
            % [0 1000 0]
            % [0 1000 0]
            % [0 1000 0]
            % [0 1000 0]
            % this array is getting filled in the following
            % The first element of the vector is the added up priority for the current CSG
            % The second element is the time from when the intersection is occupied
            % The third element is the time until when the interesetion is occupied 
            % (But the times are never used in the algorithms, so
            % you don't have to care about them, important is just the priority for each CSG)
            
            
            % in the following we loop through all CSGs and we add up the
            % priority for each vehicles that suits to the current CSG
            for i=1:length(obj.trafficStateLookupTable)
                entry = cell2mat(obj.trafficStateLookupTable(i));
                tempPriorityGroup = obj.priorityGroup;
                for k=1:size(obj.priorityGroup,1)
                    priorityGroupMember = tempPriorityGroup(k,:);
                    if entry(priorityGroupMember(2),priorityGroupMember(3))==1
                        % check if there's no blocking car ahead
                        if ~any(tempPriorityGroup(1:k-1,2)==priorityGroupMember(2))
                            
                            % adding priority for current state
                            trafficStatePriority(i,1) =  trafficStatePriority(i,1) + priorityGroupMember(5);

                                % add the times when intersection is occupied
                                if priorityGroupMember(4)< trafficStatePriority(i,2)
                                    trafficStatePriority(i,2) = priorityGroupMember(4);
                                end
                                if priorityGroupMember(4)>trafficStatePriority(i,3)
                                    trafficStatePriority(i,3) = priorityGroupMember(4);
                                end

                                % deactivate this priorityGroupMember
                            tempPriorityGroup(tempPriorityGroup(:,1)==priorityGroupMember(1),2)=0;
                            
                            % hack for FCFS based order -> add priority 100 for first vehicle in waiting queue
                            if ~obj.params.intelligentDecision && (priorityGroupMember(1)== obj.arrivingQueue(1))
                                trafficStatePriority(i,1) =  trafficStatePriority(i,1) + 100;
                                
                            end
                        end
                    end
                end
            end
            
            optimalTrafficStates = find(trafficStatePriority(:,1) ==max(trafficStatePriority(:,1))); % this is the optimal CSG (can be more than one if two or more CSGs have the same priority)
            
            
            % if the leaving queue is empty we can set the maximum priority CSG immediately
            if isempty(obj.leavingQueue)
                obj.trafficState = optimalTrafficStates(1);
                
                obj.updateBrakingFlagArray; % after setting a new CSG the brakingFlagArray has to be updated
                
                % if leaving queue is not empty and old and new CSG are not equal
            elseif ~ismember(obj.trafficState,optimalTrafficStates)
                
           
                % find MPCSG-matching and MPCSG-nonmatching
                isMatching = ones(size(trafficStatePriority,1),1);
                for k = 1:size(trafficStatePriority,1)
                    for i = 1:size(obj.leavingQueue,1)
                        leavingQueueMember = obj.leavingQueue(i,:);
                        isMatching(k) = isMatching(k) && obj.trafficStateLookupTable{k}(leavingQueueMember(2),leavingQueueMember(3));
                    end
                    if isMatching(k) == 0
                        trafficStatePriority(k) = trafficStatePriority(k) - obj.params.deltaStatePriority; % the priority of MPCSG-nonmatching is reduced by delta_p
                    end
                end
                
                
                
                [~,TS]=max(trafficStatePriority(:,1));
                
                if isMatching(TS) == 1
                    % if the new CSG is matching to the old one we can set
                    % the new CSG immediately
                    
                    obj.trafficState = TS;
                  
                    obj.updateBrakingFlagArray;
                    
                else
                    % but if the new CSG does not macht with the old one we
                    % have to set CSG = 0 to evacuate the conflict zone.
                    % Therefore all vehicles has to stop
                    obj.trafficState = 0;
                    for k=1: size(obj.brakingFlagArray,1)
                        if ~any(obj.leavingQueue(:,1)== obj.brakingFlagArray(k,1))
                            obj.brakingFlagArray(k,2) = 1;
                        end
                        
                    end
                end
                
            end
            obj.dataLog.trafficState(:,end+1) = [obj.trafficState;currentTime];            
        end
        
        function priorityGroup = calculatePriority(obj, priorityGroup, vehicles)
            % the priority for each car in the first-order vehicle group (priorityGroup) is calculated
            
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
                priorityGroup(i,5) = priority; % add priority to priority group
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
                obj.stateStartingTime = currentTime;
            end
            
            % Get orders for every vehicle according to the traffic light
            % state
            obj.vehicleOrders = obj.getConventionalVehicleOrders(obj.arrivingGroup, obj.currentState, waitingState);
            
        end
        
        function vehicleOrders = getConventionalVehicleOrders(obj, arrivingGroup, currentState, waitingState)
            % set the vehicle orders for vehilces using a traffic light at
            % the crossroad
            
            % get vehicles that need orders
            vehicleOrders = [arrivingGroup(:,1) ones(size(arrivingGroup,1),1)];
            
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
 
        function updateTrafficStateFromConventionalSystem(obj ,currentTime)
            % this function calculates the current traffic state for a
            % conventional traffic light system
            
            % the following line calculates a floating number x by using the
            % current time. This number x goes up from 1 to 6 in a loop
            x = (mod(round((currentTime + 14)*20),600))/100+1; 
            
            % the number before the comma is the CSG
            newTrafficState = floor(x);
            
            % the number after the comma represents the time we already
            % have passed in the current CSG
            % For example: durationOfTrafficState = 0.34 -> 34% of the
            % duration of the current CSG has passed
            durationOfTrafficState = 10*rem(x,1)/10;
            
            % durationYellowPhase defines the yellow phase for each CSG
            durationYellowPhase = 0.4; % 40% of the current CSG is the yellow phase
            
            % select special braking flag array when in a yellow phase
            if durationOfTrafficState < durationYellowPhase
                switch newTrafficState
                    case 2
                        % for the simulation of traffic lights we need more
                        % CSGs than the seven we have defined. The function
                        % updateCustomBrakingFlagArray can set custom
                        % CSGs. The argument of the method is a CSG in the
                        % common structure
                        customBrakingFlags = [0 0 1 1;
                                              0 0 0 0;
                                              0 0 0 0;
                                              0 0 0 0];
                        
                    case 3
                        
                        customBrakingFlags = [0 0 0 0;
                                              0 0 0 0;
                                              1 1 0 0;
                                              0 0 0 0];
                        
                    case 5
                        customBrakingFlags = [0 0 0 0;
                                              1 0 0 1;
                                              0 0 0 0;
                                              0 0 0 0];
                        
                    case 6
                        customBrakingFlags = [0 0 0 0;
                                              0 0 0 0;
                                              0 0 0 0;
                                              0 1 1 0];
                          
                    otherwise
                        customBrakingFlags = [0 0 0 0;
                                              0 0 0 0;
                                              0 0 0 0;
                                              0 0 0 0];
                        
                end
                % update with this special braking flag array
                obj.updateCustomBrakingFlagArray(customBrakingFlags);
            else
                % if no yellow phase use the normal CSG
                obj.trafficState  = newTrafficState;
                obj.updateBrakingFlagArray;
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



