classdef CrossroadUnit < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %static
        V2I
        id
        startingNodes
        breakingNodes
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
             % platooning (parameters of implemented platooning control)
                   % tau
                   % d_s
        
        %dynamic
        arrivingQueue % this is named Waiting Queue in our  paper
        priorityGroup % this is named first-order vehicle group in our  paper
        leavingQueue        
        trafficState %this is named CSG in our paper   
        breakingFlagArray % Array with the following structure: [ carId breakingFlag]
                          %                                     [ carId breakingFlag]
                          % breakingFlag: 1 -> Stop!, 0 -> Go!
         dataLog
        
        
    end
    
    methods
        function obj = CrossroadUnit(id,startingNodes,breakingNodes,stoppingNodes,leavingNodes)
            obj.id = id;
            obj.trafficStateLookupTable = obj.generateLookupTable(startingNodes);
            obj.startingNodes = startingNodes;
            obj.breakingNodes = breakingNodes;
            obj.stoppingNodes = stoppingNodes;
            obj.leavingNodes = leavingNodes;
            
            %obj.params.platooning.tau = 1.4;
            %obj.params.platooning.d_s = 25;
            
            %platooning control isn't working as expected, therefore a
            %constant platooning time behind the car ahead is set:
            obj.params.platooning = 0.2;
            
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
        
        function trafficStateLookupTable = generateLookupTable(~,crossroad)
            
                % Lookup Table for a 4-road-crossing with 7 different CSGs 
                
                % The definition is
                
                % from N W S E
                % to N x x x x
                % to W x x x x
                % to S x x x x
                % to E x x x x
                
                trafficStateLookupTable={[
                    [0 1 1 1]
                    [1 0 0 0]
                    [0 0 0 0]
                    [0 0 0 0] ],
                    [
                    [0 0 1 1]
                    [0 0 0 0]
                    [1 1 0 0]
                    [0 0 0 0] ],
                    [
                    [0 0 0 0]
                    [0 0 0 0]
                    [1 1 0 1]
                    [0 0 1 0] ],
                    [
                    [0 0 0 0]
                    [1 0 1 1]
                    [0 1 0 0]
                    [0 0 0 0] ],
                    [
                    [0 0 0 0]
                    [1 0 0 1]
                    [0 0 0 0]
                    [0 1 1 0] ],
                    [
                    [0 0 0 1]
                    [0 0 0 0]
                    [0 0 0 0]
                    [1 1 1 0] ],
                    [    
                    [0 0 0 1]
                    [1 0 0 0]
                    [0 1 0 0]
                    [0 0 1 0] ]};
           
        end
        
        function carReachesCrossroad(obj,car,startingNode)
            % when a car is reaching the starting node of a crossroad, the
            % car has to be registrated in the arriving queue
            
%             if ~isempty(obj.arrivingQueue)
%                 if any(obj.arrivingQueue(:,1) == car.id) 
%                     return % hack for a crazy fault that a car is added twice to arriving queue
%                 end
%             end
            
            if obj.params.intelligentDecision == 1 % for normal algorithm
                
                % check if cars destination is before crossroad
                if length(car.pathInfo.path)>3
                    arrivingDirection = find(startingNode == obj.startingNodes); % from which direction the car is coming 1=N, 2=E, 3=S, 4=W
                    
                    
                    index = find(ismember(car.pathInfo.path,obj.leavingNodes));
                    

                    try
                        leavingDirection = find(car.pathInfo.path(index(1)) == obj.leavingNodes); % in which direction the car is going 1=N, 2=E, 3=S, 4=W
                        
                    catch ME
                        
                        disp('crossroad error');
                        
                    end
                    
                    
                    TTR = 0; %equal to ETA in our paper. First simple calculation of TTR, but is being replaced later anyway by the estimator of the car 
                    
                    % arring queue definition:
                    % [car id; arrving direction; leaving direction; ETA]
                    % the cars will be added in this style to the arriving
                    % queue. One line is one car.
                    obj.arrivingQueue = [obj.arrivingQueue ;[car.id arrivingDirection leavingDirection TTR]];
                end
            end
            
            
        end
        
        function carReachesBreakingPoint(obj,car,breakingNode, global_timesteps)
            % this function is executed when a car reaches the braking point. Now the main algorithm has to be executed to derive the optimal CSG             
            
            % When we are in FCFS the car hasn't been added in the arriving
            % queue yet. This is done here.
            if obj.params.intelligentDecision == 0
                    arrivingDirection = find(breakingNode == obj.breakingNodes);
                    leavingDirection = find(car.pathInfo.path(3) == obj.leavingNodes);
                    TTR = 190/car.dynamics.speed + global_timesteps;

                    obj.arrivingQueue = [obj.arrivingQueue ;[car.id arrivingDirection leavingDirection TTR]];
             
            end
           
            if ~isempty(obj.arrivingQueue)
                if obj.params.conventionalTrafficLights == 0
                    % now the main part of the algorithm is executed. It is
                    % built up by the two following functions
                    getPriorityGroup(obj,car,global_timesteps); % get first-order vehicle group
                    deriveMPCSG(obj,global_timesteps); % derive the optimal CSG for this group of cars
                end
            end
            
            % the next step is to update the braking flag array according
            % to the new CSG 
              obj.breakingFlagArray = [obj.breakingFlagArray; [car.id 1 ]]; % this is just to add the new car to the braking flag array
              
              if obj.trafficState ~=0
              obj.updateBreakingFlagArray;  % this function updates the braking flag array if the current CSG is not zero 
              end
            
                % now we have to check what the new braking flag for the
                % new car is. If it is zero, we have to remove the car from
                % the arriving queue 
              breakingFlag = obj.breakingFlagArray(obj.breakingFlagArray(:,1)==car.id,2);
            
            if breakingFlag == 0 
                
                % remove car from waiting queue for specific cardinal direction
                obj.arrivingQueue(obj.arrivingQueue(:,1)==car.id,:)=[];
                
            end
            
        end
        
        function carLeavesCrossroad(obj,car,global_timesteps)
            % when a vehicle reaches the leaving node of a crossroad, the
            % main algorithm has to be executed again and the car has to be
            % deleted from the leaving queue
            
            
 
            if ~isempty(obj.leavingQueue)
                
                
                obj.leavingQueue(obj.leavingQueue(:,1)==car.id,:)=[]; % remove car from leaving queue
                obj.breakingFlagArray(obj.breakingFlagArray(:,1)==car.id,:)=[]; % remove car from braking flag array
                
                
                % when there are still cars in the arriving queue then the
                % main algorithm has to be executed again
                if ~isempty(obj.arrivingQueue)
                    if obj.params.conventionalTrafficLights == 0
                        getPriorityGroup(obj,car,global_timesteps); % get first order vehicle group
                        deriveMPCSG(obj,global_timesteps); % derive the optimal CSG for this group of cars
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
                            catch ME
                                disp('crossroad error 2');
                                Err_expression = false;
                            end
                            

                            % if no blocking car ahead check traffic state
                            if Err_expression
                                % now we can get the braking flag according
                                % to the CSGs we have defined in the lookup
                                % table
                                newBreakingFlag = ~obj.trafficStateLookupTable{obj.trafficState,1}(carDirection(1),carDirection(2));
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
                             newBreakingFlag = ~obj.trafficStateLookupTable{obj.trafficState,1}(carDirection(1),carDirection(2));
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
        
        function getPriorityGroup(obj,~,global_timesteps) % get first order vehicle group which only is considered in the algorithm
            
            % in this function we delimitate vehicles with close ETAs. The
            % parameter we are using is criticalDeltaTTR (in our paper: deltaETA)
            
            Vehicles = evalin('base','Vehicles');
            % we loop through all vehicles to get the estimated time of
            % arrival at the conflict zone (ETA)
            for i=1:size(obj.arrivingQueue,1)
                vehicle = Vehicles(obj.arrivingQueue(i,1));
                
                tempArrivingQueue = obj.arrivingQueue(1:i-1,:);
                
                % to get exact estimations of all vehicles in the arriving
                % queue, it has to be checked if there is another vehicle
                % ahead. The next lines evaluate if there is another car
                % ahead and if so then the ETA of the car ahead is saved in
                % 'ETAcarInFront'. If there is no other car ahead
                % ETAcarInFront = 0. 
                if any(tempArrivingQueue(:,2)== obj.arrivingQueue(i,2))
                    rowIndexCarInFront = find(tempArrivingQueue(:,2)== obj.arrivingQueue(i,2));
                    ETAcarInFront = obj.arrivingQueue(rowIndexCarInFront(end),4);
                else
                    ETAcarInFront = 0;
                end
                stoppingNode = obj.stoppingNodes(obj.arrivingQueue(i,2));
                % the following line requests the ETA of the current
                % vehicle using the stopping node and the ETA of the
                % vehicle ahead (in case there is one)
                timeToReach = obj.calculateEstimatedTimeOfArrival(vehicle,stoppingNode,ETAcarInFront,obj.params.platooning,global_timesteps);
                
                
                obj.arrivingQueue(i,4) = timeToReach;                
                
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
        
        function deriveMPCSG(obj, global_timesteps)
            % this function dervies the maximum priority CSG by using the
            % first-order vehicle group
            Vehicles = evalin('base','Vehicles');
            priority = [];
            % in the following the priority for each car in the first-order
            % vehicle group (obj.priorityGroup) is calculated
            for i=1:size(obj.priorityGroup,1)
                priorityGroupMember = obj.priorityGroup(i,:);
                vehicle = findobj(Vehicles,'id',priorityGroupMember(1));
                if obj.params.energyEquation == 0
                    priority = [priority; 1 + vehicle.dynamics.speed * obj.params.alpha];   % time optimized priority formula
                else
                    priority = [priority; 1 + (vehicle.dynamics.speed)^2 * obj.params.alpha2 * vehicle.physics.mass];   % energy optimized priority formula
                end
            end
            
            obj.priorityGroup(:,5) = priority;
            
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
                entry = obj.trafficStateLookupTable(i);
                tempPriorityGroup = obj.priorityGroup;
                for k=1:size(obj.priorityGroup,1)
                    priorityGroupMember = tempPriorityGroup(k,:);
                    if entry{1}(priorityGroupMember(2),priorityGroupMember(3))==1
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
                        isMatching(k) = isMatching(k) && obj.trafficStateLookupTable{k,1}(leavingQueueMember(2),leavingQueueMember(3));
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
        obj.dataLog.trafficState = [obj.dataLog.trafficState [obj.trafficState;global_timesteps]]; 

            
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
        
        function timeToReach = calculateEstimatedTimeOfArrival(obj,vehicle,stoppingNode,ETAcarInFront,platooningTimeBehind,global_timesteps)
            
            distToConflictZone = norm(vehicle.dynamics.position -  vehicle.map.get_coordinates_from_waypoint(stoppingNode)) + 100;
            % if vehicle is in acceleration phase
            if abs(vehicle.dynamics.speed - vehicle.dynamics.maxSpeed)>1
                averageAcceleration = vehicle.dynamics.maxSpeed - vehicle.dynamics.speed; % average acceleration to reach top speed
                accelerationDistance = obj.getAccelerationDistance(averageAcceleration, vehicle.dynamics.speed, vehicle.dynamics.maxSpeed);
                
                if accelerationDistance < distToConflictZone
                    % if the acceleration distance is below the distance to
                    % the conflict zone we have to calculate t1 (time in
                    % accleration phase) and t2 (time in constant speed
                    % phase)
                    t1 = obj.timeToReachNextWaypointInAccelerationPhase(vehicle.dynamics.speed, averageAcceleration, accelerationDistance);
                    t2 = (distToConflictZone-accelerationDistance)/vehicle.dynamics.maxSpeed;
                    timeToReach = t1+t2+global_timesteps;
                else
                    % if the acceleration distance is above the distance to
                    % the conflict zone we can immediately calculate the
                    % time to reach
                    timeToReach = obj.timeToReachNextWaypointInAccelerationPhase(vehicle.dynamics.speed, averageAcceleration, distToConflictZone)+global_timesteps;
                    
                end
            else
                % if vehicle is not in accleration phase simple constant
                % speed phase calculation
                timeToReach = distToConflictZone/vehicle.dynamics.speed + global_timesteps;
            end
            
            
            if ETAcarInFront ~= 0
                % if there is a car in front
                if timeToReach < ETAcarInFront
                    
                    timeToReach = ETAcarInFront + platooningTimeBehind; % platooninTimeBehind is a constant, set in the constructor of the crossroad unit TODO (has to be changed)
                end
            end
        end
        
        function accelerationDistance = getAccelerationDistance(~, averageAcceleration, currentSpeed, speedTo)
            delta_v = speedTo-currentSpeed;
            accelerationDistance = delta_v^2/(2*averageAcceleration)+currentSpeed*delta_v/averageAcceleration;
        end
        
        %% Eigenvalues of Equation 8 in NecSys Paper
        function timeToReach = timeToReachNextWaypointInAccelerationPhase(~, currentSpeed, averageAcceleration, distance)
            timeToReach = -currentSpeed/averageAcceleration + sqrt((currentSpeed/averageAcceleration)^2+2*distance/averageAcceleration);
        end
    end
    
end



