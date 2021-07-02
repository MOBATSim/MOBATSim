classdef Vehicle < handle
    %VEHICLE Vehicle Class is used to generate Vehicle instances for each vehicle in the simulation.
    %
    % VEHICLE Methods:
    %    Vehicle - Constructer Method
    %    checkCollision - Checks if a collision has happened with another vehicle
    %    getHitbox - Gets the hitbox of vehicles to check for collision
    %    vehiclesCollide - Stops the vehicles if a collision has happened
    %    checkIntersection - Checks if hitboxes intersect to raise the collision flag
    %    checkifDestinationReached - Checks if the vehicle has reached its destination waypoint
    %    calculateEstimatedTimeOfArrival - (TODO: Check in detail)
    %    getAverageAcceleration - (TODO: Check in detail)
    %    getAccelerationDistance - (TODO: Check in detail)
    %    timeToReachNextWaypointInAccelerationPhase - (TODO: Check in detail)
    % SET Methods:
    %    setStopStatus - Stops the vehicle
    %    setCurrentRoute
    %    setPath
    %    setVehicleSensorDetection
    %    setEmergencyCase
    %    setDrivingMode
    %    setCollided
    %    setCurrentTrajectory
    %    updateActualSpeed
    %    updateVehicleFrenetPosition
    %    setLastWaypoint
    %    setRouteCompleted
    %    setDestinationReached
    %    setYawAngle
    %    setPosition

    properties
        id              %    id - Unique id of a vehicle
        name            %    name - A custom name for a vehicle
        drivingBehavior % Check where they are set and get
        physics
        %         size
        %         mass
        dynamics
        %         position
        %         speed
        %         maxSpeed
        %         orientation
        %         directionVector
        %         cornering
        %         minDeceleration
        %         acceleration                  acceleration applied to model        
        %         steeringAngle                 steering angle applied to model
        sensors
        %         frontSensorRange
        %         AEBdistance
        %         leadingVehicleId
        %         distanceToLeadingVehicle
        %         leadingVehicleSpeed
        status
        %         emergencyCase
        %         drivingMode
        %         stop
        %         collided
        %         ttc                           time to collision
        pathInfo
        %         startingTime
        %         currentRoute
        %         destinationReached
        %         stopAt
        %         startingPoint
        %         destinationPoint
        %         lastWaypoint
        %         referencePath
        %         routeCompleted
        %         calculateNewPathFlag
        %         path
        %         BOGPath
        dataLog
        %         timeStamps
        %         totalTravelTime
        %         speed
        %         emergencyCase
        %         speedInCrossroad        Speed from braking to leaving point
        %         speedInCrossroad2       Speed from starting to leaving point
        %         platooning
        decisionUnit
        %         Map
        %         accelerationPhase
        %         initialFutureData
        %         futureData
        %         breakingFlag
        %         inCrossroad
        V2I
        V2VdataLink
        V2IdataLink
        map
    end
    
    methods
        function obj = Vehicle(id, car_name,startingPoint,destinationPoint,startingTime,maxSpeed,size,dataLinkV2V,dataLinkV2I,mass,frontSensorRange,AEBdistance,minDeceleration, map)
            obj.id = id;
            obj.name = car_name;
            
            obj.drivingBehavior.safetyTime = 2; % Check where they are set and get
            
            obj.physics.size = size; %Should be edited according to the vehicle
            obj.physics.mass = mass; %kg Should be edited according to the vehicle
            
            obj.dynamics.position = [0 0 0];
            obj.dynamics.speed = 0;
            obj.dynamics.maxSpeed =maxSpeed;
            obj.dynamics.orientation = [0 1 0 0];
            obj.dynamics.minDeceleration = minDeceleration;
            obj.dynamics.acceleration = 0;
            obj.dynamics.steeringAngle = 0;
            
            
            obj.sensors.frontSensorRange            = frontSensorRange;
            obj.sensors.AEBdistance                 = AEBdistance;
            obj.sensors.leadingVehicleId            = 0;
            obj.sensors.distanceToLeadingVehicle    = 1000;
            %obj.sensors.leadingVehicleSpeed          TODO: implement this variable
            % TODO: check following, if they are needed
            obj.sensors.leadingVehicle = []; % Check where they are set and get
            obj.sensors.behindVehicle = []; % Check where they are set and get % TODO: name it to rearVehicle
            obj.sensors.behindVehicleSafetyMargin = 1000; % Check where they are set and get
            obj.sensors.behindVehicleDistance = 1000; % Check where they are set and get
            
            
            
            obj.setEmergencyCase(0); % no emergency case appears
            obj.setDrivingMode(0);
            obj.setStopStatus(true);
            obj.setCollided(false); % vehicle has no collision
            obj.status.canLaneSwitch = 0; % Check where they are set and get
            obj.status.laneSwitchFinish = 0; % Check where they are set and get
            obj.status.ttc = 1000;
            
            obj.pathInfo.startingTime = startingTime;
            obj.pathInfo.currentTrajectory = [];
            obj.pathInfo.currentRoute =0;
            obj.pathInfo.destinationReached = false;
            obj.pathInfo.stopAt = 0;
            obj.pathInfo.startingPoint = startingPoint;
            obj.pathInfo.destinationPoint = destinationPoint;
            obj.pathInfo.lastWaypoint = startingPoint;
            obj.pathInfo.referencePath = [];
            obj.pathInfo.routeCompleted = true;
            obj.pathInfo.calculateNewPathFlag = true;
            obj.pathInfo.path = 0;
            obj.pathInfo.staticPath = 0; % Check where they are set and get
            obj.pathInfo.BOGPath = [];
            obj.pathInfo.laneId = 0;  % Check where they are set and get
            obj.pathInfo.s = 0; % Check where they are set and get
            obj.pathInfo.d = 0; % Check where they are set and get
            obj.pathInfo.routeEndDistance = []; % Check where they are set and get
            
            obj.dataLog.totalTravelTime = 0;
            obj.dataLog.speedInCrossroad = [];
            obj.dataLog.speedInCrossroad2 = [];
            obj.dataLog.speed = [];
            obj.dataLog.emergencyCase = [];
            obj.dataLog.platooning = [];
            obj.dataLog.laneSwitchStartTime = []; % Check where they are set and get
            obj.dataLog.laneSwitchEndTime = []; % Check where they are set and get
            obj.dataLog.MinJerkTrajPolynom = {};% Check where they are set and get
            
            
            obj.map = map;
            
            %obj.decisionUnit = DecisionUnit;
            obj.decisionUnit.Map = map;
            obj.decisionUnit.accelerationPhase = zeros(1,5);
            obj.decisionUnit.initialFutureData = [];
            obj.decisionUnit.futureData = [];
            obj.decisionUnit.breakingFlag = 0;
            obj.decisionUnit.inCrossroad = [0 0];
            obj.decisionUnit.LaneSwitchTime = 4; % Check where they are set and get
            
            obj.V2VdataLink = dataLinkV2V;
            obj.V2IdataLink = dataLinkV2I;
            obj.V2I = V2I(id, dataLinkV2I, map);
            
            obj.setPosition(obj.map.get_coordinates_from_waypoint(startingPoint));
            obj.setYawAngle(obj.map.getInitialYawAnglefromWaypoint(startingPoint));
        end %Constructor
        
        function bool = checkWaypointReached(car,Destination)
            if car.pathInfo.routeEndDistance < 1 % consider to reach the endpoint when distance smaller than a threshold. Threshold defined by the user
                car.pathInfo.s = 0;%reset s at the end of road
                
                lastWaypoint = car.map.get_waypoint_from_coordinates(Destination);
                car.setLastWaypoint(lastWaypoint); % Vehicle Set
                
                if ~(car.checkifDestinationReached()) % Check if destination reached
                    % If not -> find the next route and calculate the trajectory
                    nextRoute = car.generateCurrentRoute(car.pathInfo.path,lastWaypoint);
                    car.setCurrentRoute(nextRoute);
                    car.pathInfo.path(1) = [];
                    currentTrajectory = car.generateTrajectoryFromPath(car.pathInfo.path);
                    car.setCurrentTrajectory(currentTrajectory); % Vehicle - Set Functions
                    car.pathInfo.calculateNewPathFlag = 1; % TODO: check
                end
                bool = true;
            else
                bool = false;
            end
        end
        
        function currentTrajectory = generateTrajectoryFromPath(car,path)
            % Format of route for vehicle dynamics (translation)
            if (isempty(find((car.map.connections.translation(:,1) == path(1) )&(car.map.connections.translation(:,2)== path(2)), 1 )) == false)
                index = find((car.map.connections.translation(:,1) == path(1) )&(car.map.connections.translation(:,2)== path(2)) );
                currentTrajectory = [car.map.waypoints(car.map.connections.translation(index,1),:);
                    car.map.waypoints(car.map.connections.translation(index,2),:);
                    zeros(1,3);
                    zeros(1,3)];
            end
            % Format of route for vehicle dynamics (curves)
            if (isempty(find((car.map.connections.circle(:,1) == path(1) )&(car.map.connections.circle(:,2)== path(2)), 1 )) == false)
                index = find((car.map.connections.circle(:,1) == path(1) )&(car.map.connections.circle(:,2)== path(2)) );
                currentTrajectory = [car.map.waypoints(car.map.connections.circle(index,1),:);
                    car.map.waypoints(car.map.connections.circle(index,2),:);
                    abs(car.map.connections.circle(index,3)),car.map.connections.circle(index,4),car.map.connections.circle(index,6);
                    -sign(car.map.connections.circle(index,3))*ones(1,3)];
            end
        end
        
        function currentRoute = generateCurrentRoute(car, path, lastWaypoint)
            
            idx = find(path==lastWaypoint);
            if idx+1<=length(path) % Next Route
                currentRoute = car.map.getRouteIDfromPath([path(idx) path(idx+1)]);
            else % Destination Reached // CurrentRoute stays the same
                currentRoute = car.pathInfo.currentRoute;
            end
        end
        
        function checkCollision(vehicle,car)
            %this function should be seperate from the sensor because sensor can be faulty but collision will appear as
            %an accident so to create the conditions for instant stop we need to keep it as a seperate function
            Box1 = getHitbox(vehicle);
            Box2 = getHitbox(car);
            hasCollided = checkIntersection(vehicle,Box1,Box2);
            if hasCollided == true
                % Collision occurs
                vehicle.vehiclesCollide(car);
            end
            
        end
        
        function Hitbox = getHitbox(car)
            
            centerP = [car.dynamics.position(1); -car.dynamics.position(3)];
            angle = car.dynamics.orientation(4); % it isn't always correct on corners
            
            a = car.physics.size(3)/2; % Half length of a vehicle
            b = car.physics.size(2)/2; % Half width of a vehicle
            
            p1 =[a; b];
            p2 =[a; -b];
            p3 =[-a; -b];
            p4 =[-a; b];
            
            Rmatrix = [cos(angle) -sin(angle); sin(angle) cos(angle)];
            
            p1r = centerP+Rmatrix*p1;
            p2r = centerP+Rmatrix*p2;
            p3r = centerP+Rmatrix*p3;
            p4r = centerP+Rmatrix*p4;
            
            Hitbox = [p1r p2r p3r p4r];
            %Hitbox = [p1r p2r p3r p4r p1r]; %use this if we want to plot the collision scene like X-Ray
            
            
        end
        
        function vehiclesCollide(car1,car2)
            % set collision on both vehicles and stop them
            car1.setCollided(true);
            car1.setStopStatus(true);
            
            car2.setCollided(true);
            car2.setStopStatus(true);
        end
              
        function CollisionFlag = checkIntersection(~,BoxA,BoxB)
            
            CornersAx = transpose(BoxA(1,:));
            CornersAy = transpose(BoxA(2,:));
            CornersBx = transpose(BoxB(1,:));
            CornersBy = transpose(BoxB(2,:));
            
            in = inpolygon(CornersAx,CornersAy,CornersBx,CornersBy);
            
            if max(in) > 0
                CollisionFlag = true;
                return;
            else
                in = inpolygon(CornersBx,CornersBy,CornersAx,CornersAy);
                if max(in) > 0
                    CollisionFlag = true;
                    % To plot the collision scene
                    %plot(CornersAx,CornersAy,CornersBx,CornersBy)
                    return;
                else
                    CollisionFlag = false;
                    return;
                end
                
            end
            
            
        end
        
        function reached = checkifDestinationReached(car)
            reached = false;
            if car.dataLog.totalTravelTime ==0 % The vehicle hasn't reached before so it is equal to zero
                if car.pathInfo.lastWaypoint == car.pathInfo.destinationPoint
                    % Vehicle has reached its destination
                    reached = car.setDestinationReached(true);
                    % Stop the vehicle
                    car.setStopStatus(true);
                    % Close the V2V connection
                    car.V2VdataLink(car.V2VdataLink==1) =0;
                end
            else
                if car.pathInfo.destinationReached
                    reached = true;
                else
                    reached = false; %This case should never happen
                end
            end
        end
        
        
        %% Estimator of the vehicle to calculate the ETA at the crossroad
        
        function timeToReach = calculateEstimatedTimeOfArrival(obj,stoppingNode,ETAcarInFront,platooningTimeBehind,global_timesteps)
            distToConflictZone = norm(obj.dynamics.position -  obj.map.get_coordinates_from_waypoint(stoppingNode)) + 100;
            % if vehicle is in acceleration phase
            if abs(obj.dynamics.speed - obj.dynamics.maxSpeed)>1
                averageAcceleration = NN_acceleration([obj.dynamics.speed; obj.dynamics.maxSpeed-obj.dynamics.speed]); % NN_acceleration -> can be replaced by getAverageAcceleration(speedFrom, speedTo)
                accelerationDistance = getAccelerationDistance(obj, averageAcceleration, obj.dynamics.speed, obj.dynamics.maxSpeed);
                
                if accelerationDistance < distToConflictZone
                    % if the acceleration distance is below the distance to
                    % the conflict zone we have to calculate t1 (time in
                    % accleration phase) and t2 (time in constant speed
                    % phase)
                    t1 = timeToReachNextWaypointInAccelerationPhase(obj, obj.dynamics.speed, averageAcceleration, accelerationDistance);
                    t2 = (distToConflictZone-accelerationDistance)/obj.dynamics.maxSpeed;
                    timeToReach = t1+t2+global_timesteps;
                else
                    % if the acceleration distance is above the distance to
                    % the conflict zone we can immediately calculate the
                    % time to reach
                    timeToReach = timeToReachNextWaypointInAccelerationPhase(obj, obj.dynamics.speed, averageAcceleration, distToConflictZone)+global_timesteps;
                    
                end
            else
                % if vehicle is not in accleration phase simple constant
                % speed phase calculation
                timeToReach = distToConflictZone/obj.dynamics.speed + global_timesteps;
            end
            
            
            if ETAcarInFront ~= 0
                % if there is a car in front
                if timeToReach < ETAcarInFront
                    
                    timeToReach = ETAcarInFront + platooningTimeBehind; % platooninTimeBehind is a constant, set in the constructor of the crossroad unit (has to be changed)
                end
            end
        end
        
        function averageAcceleration = getAverageAcceleration(obj, speedFrom, speedTo)
            
            % this is the alternative to the neural network for the
            % acceleration phase. This calculation is based on a polynom,
            % derived by curve fitting on acceleration measurements.
            x = speedFrom;
            y = abs(speedTo-speedFrom);
            p00 =      0.1091  ;
            p10 =     -0.1124  ;
            p01 =    -0.02811  ;
            p20 =     0.02018  ;
            p11 =     0.04362  ;
            p02 =     0.01583  ;
            p30 =   -0.001595  ;
            p21 =   -0.004716  ;
            p12 =     -0.0054  ;
            p03 =   -0.001514  ;
            p40 =   5.917e-05  ;
            p31 =   0.0001982  ;
            p22 =   0.0003198  ;
            p13 =    0.000251  ;
            p04 =   5.423e-05  ;
            p50 =  -8.408e-07  ;
            p41 =  -2.908e-06  ;
            p32 =  -6.004e-06  ;
            p23 =   -6.48e-06  ;
            p14 =  -3.938e-06  ;
            p05 =  -6.736e-07  ;
            averageAcceleration = sign(speedTo-speedFrom)*(p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2 + p30*x^3 + p21*x^2*y+ p12*x*y^2 + p03*y^3 + p40*x^4 + p31*x^3*y + p22*x^2*y^2 + p13*x*y^3 + p04*y^4 + p50*x^5 + p41*x^4*y + p32*x^3*y^2 + p23*x^2*y^3 + p14*x*y^4 + p05*y^5) ;
            
        end
        
        function accelerationDistance = getAccelerationDistance(~, averageAcceleration, currentSpeed, speedTo)
            delta_v = speedTo-currentSpeed;
            accelerationDistance = delta_v^2/(2*averageAcceleration)+currentSpeed*delta_v/averageAcceleration;
        end
        %% Eigenvalues of Equation 8 in NecSys Paper
        function timeToReach = timeToReachNextWaypointInAccelerationPhase(~, currentSpeed, averageAcceleration, distance)
            timeToReach = -currentSpeed/averageAcceleration + sqrt((currentSpeed/averageAcceleration)^2+2*distance/averageAcceleration);
        end
        
        %% SET/GET Functions to control the changes in the properties
        
        function setStopStatus(car, stop)
            car.status.stop = stop;
            if stop % If Status Stop then the speed should be zero instantly (slowing down is not factored in yet but might come with the next update)
                car.updateActualSpeed(0);
            end
        end
        
        function logInitialFuturePlan(car,newFutureData,global_timesteps)
            if global_timesteps == 0 % save the future data at the beginning of the simulation for validation after simulation
                car.decisionUnit.initialFutureData = newFutureData;
            end
        end
        
        function setCurrentRoute(car, RouteID)
            car.pathInfo.currentRoute = RouteID;
        end
        
        function setPath(car, newPath)
            car.pathInfo.path = newPath;
        end
        
        function setVehicleSensorDetection(car,leadingVehicleID, distanceToLeading, rearVehicleID, distanceToRear)
            car.sensors.leadingVehicleId = leadingVehicleID;
            car.sensors.distanceToLeadingVehicle = distanceToLeading;
            %% Temp -> TODO: Carry these into Situation Awareness Block
            if ~(leadingVehicleID==-1) % Register Front Vehicle
                car.sensors.leadingVehicle = car.map.Vehicles(leadingVehicleID);
                relSpeed = car.dynamics.speed - car.sensors.leadingVehicle.dynamics.speed;
                car.status.ttc = distanceToLeading/relSpeed;
            else
                car.sensors.leadingVehicle = [];
                car.status.ttc = inf;
                
            end
            
            if ~(rearVehicleID==-1) % Register Behind Vehicle if exists
                car.sensors.behindVehicle = car.map.Vehicles(rearVehicleID);
                relSpeed = car.sensors.behindVehicle.dynamics.speed-car.dynamics.speed;
                car.sensors.behindVehicleSafetyMargin = distanceToRear/relSpeed;
            else
                car.sensors.behindVehicle = [];
                car.sensors.behindVehicleSafetyMargin = inf;
            end
            
        end
        
        function setEmergencyCase(car, EmergencyCase)
            car.status.emergencyCase = EmergencyCase;
        end
        
        function setDrivingMode(car, DrivingMode)
            car.status.drivingMode = DrivingMode;
        end
        
        function setCollided(car, collided)
            car.status.collided = collided;
        end
        
        function setCurrentTrajectory(car, currentTrajectory)
            car.pathInfo.currentTrajectory = currentTrajectory;
        end
        
        function updateActualSpeed(car,speed)
            car.dynamics.speed = speed;
        end
        
        function updateVehicleFrenetPosition(car, s,vehicle_d,routeLength)
            car.pathInfo.s = s;% driven length
            car.pathInfo.d = vehicle_d; % lateral offset
            car.pathInfo.routeEndDistance = routeLength-s; %distance to the current routes endpoint
        end
        
        function setLastWaypoint(car,lastWaypoint)
            car.pathInfo.lastWaypoint = lastWaypoint;
        end
        
        function bool = setRouteCompleted(car,bool)
            car.pathInfo.routeCompleted = bool;  
        end
        
        function bool = setDestinationReached(car,bool)
            car.pathInfo.destinationReached = bool;
            car.setRouteCompleted(bool);
            
            if bool
                car.dataLog.totalTravelTime = get_param('MOBATSim','SimulationTime'); % TODO: Check how we can send the time into this function
            else
                car.dataLog.totalTravelTime = 0;
            end
            
        end
               
        function setOrientation(car,newOrientation) % Might be obsolete after implementing setYawAngle function
            car.dynamics.orientation = newOrientation;
        end
        
        function setYawAngle(car,YawAngle) % Might replace set Orientation after careful analysis
            % Set the yaw angle in rad
            car.dynamics.orientation = [0 1 0 YawAngle];
        end
        
        function setPosition(car,newPosition)
            car.dynamics.position = newPosition;
        end
        
        
    end
end

