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
        arrivingQueue   = []% this is named Waiting Queue in our  paper
        leavingQueue    = []
        priorityGroup % this is named first-order vehicle group in our  paper
                
        trafficState %this is named CSG in our paper   
        breakingFlagArray % Array with the following structure: [ carId breakingFlag]
                          %                                     [ carId breakingFlag]
                          % breakingFlag: 1 -> Stop!, 0 -> Go!
        dataLog
            % traffic state
    end
    
    methods
        function obj = CrossroadUnit(id, startingNodes,brakingNodes,stoppingNodes,leavingNodes)
            obj.id = id;          
            obj.startingNodes = startingNodes;
            obj.brakingNodes = brakingNodes;
            obj.stoppingNodes = stoppingNodes;
            obj.leavingNodes = leavingNodes;
            
            obj.trafficStateLookupTable = obj.generateLookupTable();

            %platooning control isn't working as expected, therefore a
            %constant platooning time behind the car ahead is set:
            obj.params.platooningTimeBehind = 0.2;
            
            % edit all the parameters at this point to change the algorithm
            obj.params.deltaStatePriority = 2; % delta_p in our paper
            obj.params.criticalDeltaTTR = 1.5; % delta_TTR in our paper
            obj.params.alpha = 0.072;
            obj.params.alpha2 = 0.001;
            
            obj.params.intelligentDecision = 1; % 0 for FCFS, 1 for our algorithm
            obj.params.conventionalTrafficLights = 0; % 1 for conventional traffic light system
            obj.params.energyEquation = 0; % 0 for time optimized approach, 1 for energy optimized approach
            
            
            if obj.params.intelligentDecision == 0
                obj.params.criticalDeltaTTR = 1000;  % for FCFS we set delta TTR on 1000 to consider all(!) vehicles in the surrounding of the crossroad
            end

            if obj.params.energyEquation == 1
                obj.params.deltaStatePriority = 9999; % for energy optimized approach we set delta_p to 9999 (see in diploma thesis for explanation)
            end
            obj.trafficState = 0; 
           
            
            obj.dataLog.trafficState = []; 
            
        end %Constructor
        
        function trafficStateLookupTable = generateLookupTable(~)
            
                % Lookup Table for a 4-road-crossing with 7 different CSGs 
                
                % The definition is
                
                % to     N E S W
                % from N x x x x
                % from E x x x x
                % from S x x x x
                % from W x x x x
                
                % Vehicles can drive in 7 different scenarios according to
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
                    [0 0 1 0] ]}; % N-W, E-N, S-E, W-S
           
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
                end
            end
            
            
        end
        
        function carReachesBreakingPoint(obj, vehicle, vehicles, brakingNode, currentTime)
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
                    obj.getPriorityGroup(vehicles, currentTime); % get first-order vehicle group
                    obj.deriveMPCSG(vehicles, currentTime); % derive the optimal CSG for this group of cars
                end
            end
            
            % the next step is to update the braking flag array according
            % to the new CSG
            obj.breakingFlagArray = [obj.breakingFlagArray; [vehicle.id 1 ]]; % this is just to add the new car to the braking flag array
            
            if obj.trafficState ~=0
                obj.updateBreakingFlagArray;  % this function updates the braking flag array if the current CSG is not zero
            end
            
            % now we have to check what the new braking flag for the
            % new car is. If it is zero, we have to remove the car from
            % the arriving queue
            breakingFlag = obj.breakingFlagArray(obj.breakingFlagArray(:,1)==vehicle.id,2);
            
            if breakingFlag == 0
                
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
                obj.breakingFlagArray(obj.breakingFlagArray(:,1)==vehicle.id,:)=[]; % remove car from braking flag array
                
                
                % when there are still cars in the arriving queue then the
                % main algorithm has to be executed again
                if ~isempty(obj.arrivingQueue)
                    if obj.params.conventionalTrafficLights == 0
                        obj.getPriorityGroup(vehicles,currentTime); % get first order vehicle group
                        obj.deriveMPCSG(vehicles, currentTime); % derive the optimal CSG for this group of cars
                    end
                end
            end
        end
                
        function updateBreakingFlagArray(obj)
            % this function updates the breakingflag Array due to a change
            % of the current CSG
            
            if obj.params.intelligentDecision == 1
                if ~isempty(obj.breakingFlagArray)
                    for k=1: size(obj.breakingFlagArray,1)
                        if obj.breakingFlagArray(k,2) == 1
                            carId = obj.breakingFlagArray(k,1);
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
                                newBreakingFlag = ~obj.trafficStateLookupTable{obj.trafficState}(carDirection(1),carDirection(2));
                                if newBreakingFlag == 0
                                    obj.breakingFlagArray(k,2) = newBreakingFlag;
                                    obj.leavingQueue = [obj.leavingQueue;
                                        [carId carDirection]];

                                    % when braking flag is zero we can remove car from waiting queue for specific cardinal direction
                                    obj.arrivingQueue(obj.arrivingQueue(:,1)==carId,:)=[];                            

                                end
                            end
                        end
                    end

                end
            else
                % this routine is for FCFS
                if ~isempty(obj.breakingFlagArray)
                    for k=1: size(obj.breakingFlagArray,1)
                        if obj.breakingFlagArray(k,2) == 1
                             carId = obj.breakingFlagArray(k,1);
                             carDirection =  obj.arrivingQueue(obj.arrivingQueue(:,1)==carId,2:3);
                             newBreakingFlag = ~obj.trafficStateLookupTable{obj.trafficState}(carDirection(1),carDirection(2));
                                if newBreakingFlag == 0
                                    obj.breakingFlagArray(k,2) = newBreakingFlag;
                                    obj.leavingQueue = [obj.leavingQueue;
                                        [carId carDirection]];

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
        
        function updateCustomBreakingFlagArray(obj, customBreakingFlags)
            % this function is used to upadte the braking Flag Array with
            % custom CSGs. This function is used in the traffic light
            % system function.
            % the function is pretty similar to the updateBreakingFlagArray
            % function

            if ~isempty(obj.breakingFlagArray)
                for k=1: size(obj.breakingFlagArray,1)
                    if obj.breakingFlagArray(k,2) == 1
                        carId = obj.breakingFlagArray(k,1);
                        carDirection =  obj.arrivingQueue(obj.arrivingQueue(:,1)==carId,2:3);
                        
                        % check if theres a blocking car ahead
                        index = find(obj.arrivingQueue(:,2)==carDirection(1));
                        
                        % if no blocking car ahead check traffic state
                        if index(1) >= find(obj.arrivingQueue(:,1)==carId)
                            newBreakingFlag = ~customBreakingFlags(carDirection(1),carDirection(2));
                            if newBreakingFlag == 0
                                obj.breakingFlagArray(k,2) = newBreakingFlag;
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
        
        function getPriorityGroup(obj, vehicles, currentTime) % get first order vehicle group which only is considered in the algorithm
            % in this function we delimitate vehicles with close ETAs. The
            % parameter we are using is criticalDeltaTTR (in our paper: deltaETA)
            
            % we loop through all vehicles to get the estimated time of
            % arrival at the conflict zone (ETA)
            for i=1:size(obj.arrivingQueue,1)          
                
                % to get exact estimations of all vehicles in the arriving
                % queue, it has to be checked if there is another vehicle
                % ahead. The next lines evaluate if there is another car
                % ahead and if so then the ETA of the car ahead is saved in
                % 'ETAcarInFront'. If there is no other car ahead
                % ETAcarInFront = 0.                
                tempArrivingQueue = obj.arrivingQueue(1:i-1,:);
                
                if any(tempArrivingQueue(:,2)== obj.arrivingQueue(i,2))
                    rowIndexCarInFront = find(tempArrivingQueue(:,2)== obj.arrivingQueue(i,2));
                    ETAcarInFront = obj.arrivingQueue(rowIndexCarInFront(end),4);
                else
                    ETAcarInFront = 0;
                end
                
                % get the current vehicle of arriving queue
                vehicle = vehicles(obj.arrivingQueue(i,1));
                stoppingNode = obj.stoppingNodes(obj.arrivingQueue(i,2));
                % the following line requests the ETA of the current
                % vehicle using the stopping node and the ETA of the
                % vehicle ahead (in case there is one)
                obj.arrivingQueue(i,4) = obj.calculateEstimatedTimeOfArrival(vehicle,stoppingNode,ETAcarInFront, currentTime,3); % assume an average acceleration of 3             
                
            end
            
            % now we have updated the estimated time of arrival for all
            % vehicles in the arriving queue. The next step is to
            % delimitate vehicles with close ETA by using the parameter
            % criticalDeltaTTR
            
            priorityQueue = sortrows( obj.arrivingQueue,4); % priority queue has the same structure as the arriving queue
            difference = diff(priorityQueue(:,4)) < obj.params.criticalDeltaTTR;
            
            % find first order priority group seperated by critical
            % delta ttr from other vehicles
            if ~isempty(find( difference == 0, 1, 'first'))
                carIds = priorityQueue(1:find( difference == 0, 1, 'first'),1);
                % find the lines with matching carIds
                priorityQueue = priorityQueue(ismember(priorityQueue(:,1),carIds),:);
            end
            obj.priorityGroup = priorityQueue;
            
        end
        
        function deriveMPCSG(obj, vehicles, currentTime)
            % this function dervies the maximum priority CSG by using the
            % first-order vehicle group

            % in the following the priority for each car in the first-order
            % vehicle group (obj.priorityGroup) is calculated
            for i=1:size(obj.priorityGroup,1)
                priorityGroupMember = obj.priorityGroup(i,:);
                vehicle = vehicles(priorityGroupMember(1));
                if obj.params.energyEquation == 0
                    priority = 1 + vehicle.dynamics.speed * obj.params.alpha;   % time optimized priority formula
                else
                    priority = 1 + (vehicle.dynamics.speed)^2 * obj.params.alpha2 * vehicle.physics.mass;   % energy optimized priority formula
                end
                obj.priorityGroup(i,5) = priority; % add priority to priority group
            end
            
            
            
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
                
                obj.updateBreakingFlagArray; % after setting a new CSG the breakingFlagArray has to be updated
                
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
                  
                    obj.updateBreakingFlagArray;
                    
                else
                    % but if the new CSG does not macht with the old one we
                    % have to set CSG = 0 to evacuate the conflict zone.
                    % Therefore all vehicles has to stop
                    obj.trafficState = 0;
                    for k=1: size(obj.breakingFlagArray,1)
                        if ~any(obj.leavingQueue(:,1)== obj.breakingFlagArray(k,1))
                            obj.breakingFlagArray(k,2) = 1;
                        end
                        
                    end
                end
                
            end
        obj.dataLog.trafficState = [obj.dataLog.trafficState [obj.trafficState;currentTime]]; 

            
        end      
 
        function updateTrafficStateFromConventionalSystem(obj,global_timesteps)
            % this function calculates the current traffic state for a
            % conventional traffic light system
            
            % the following line calculates a floating number x by using the
            % global timesteps. This number x goes up from 1 to 6 in a loop
            x = (mod(round((global_timesteps + 14)*20),600))/100+1; 
            
            % the number before the comma is the CSG
            newTrafficState = floor(x);
            
            % the number after the comma represents the time we already
            % have passed in the current CSG
            % For example: durationOfTrafficState = 0.34 -> 34% of the
            % duration of the current CSG has passed
            durationOfTrafficState = 10*rem(x,1)/10;
            
            % durationYellowPhase defines the yellow phase for each CSG
            durationYellowPhase = 0.4; % 40% of the current CSG is the yellow phase
            
            switch newTrafficState
                case 2
                    if durationOfTrafficState < durationYellowPhase
                        % for the simulation of traffic lights we need more
                        % CSGs than the seven we have defined. The function
                        % updateCustomBreakingFlagArray can set custom
                        % CSGs. The argument of the method is a CSG in the
                        % common structure
                        obj.updateCustomBreakingFlagArray([0 0 1 1;
                                                           0 0 0 0;
                                                           0 0 0 0;
                                                           0 0 0 0]);
                    else
                        obj.trafficState  = newTrafficState;
                        obj.updateBreakingFlagArray;
                    end
                    
                case 3
                    if durationOfTrafficState < durationYellowPhase
                        obj.updateCustomBreakingFlagArray([0 0 0 0;
                                                           0 0 0 0;
                                                           1 1 0 0;
                                                           0 0 0 0]);
                    else
                        obj.trafficState  = newTrafficState;
                        obj.updateBreakingFlagArray;
                    end
                    
                case 5
                    if durationOfTrafficState < durationYellowPhase
                        obj.updateCustomBreakingFlagArray([0 0 0 0;
                                                           1 0 0 1;
                                                           0 0 0 0;
                                                           0 0 0 0]);
                    else
                        obj.trafficState  = newTrafficState;
                        obj.updateBreakingFlagArray;
                    end
                    
                case 6
                    if durationOfTrafficState < durationYellowPhase
                        obj.updateCustomBreakingFlagArray([0 0 0 0;
                                                           0 0 0 0;
                                                           0 0 0 0;
                                                           0 1 1 0]);
                    else
                        obj.trafficState  = newTrafficState;
                        obj.updateBreakingFlagArray;
                    end
                otherwise
                    if durationOfTrafficState < durationYellowPhase
                       obj.updateCustomBreakingFlagArray([0 0 0 0;
                                                          0 0 0 0;
                                                          0 0 0 0;
                                                          0 0 0 0]);
                    else
                        obj.trafficState  = newTrafficState;
                        obj.updateBreakingFlagArray;
                    end
            end
            
           
        end
        
        %% Estimation functions
        
        function timeReachingCrossroad = calculateEstimatedTimeOfArrival(obj,vehicle,stoppingNode,ETAcarInFront, currentTime, estimatedAcceleration)
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
            
            % add global time
            timeReachingCrossroad = timeToReach + currentTime;
                        
            if (ETAcarInFront ~= 0) && (ETAcarInFront > timeReachingCrossroad)
                % if there is a car in front and it is slower, vehicle will
                % arrive after the car in front
                timeReachingCrossroad = ETAcarInFront + obj.params.platooningTimeBehind; % platooninTimeBehind is time distance the vehicle is driving behind
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



