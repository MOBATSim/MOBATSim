classdef Vehicle < handle
    %VEHÝCLE Summary of this class goes here
    
    %   Detailed explanation goes here
    
    properties
        id
        simSpeed
        name
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
        sensors
        %         frontSensorRange
        %         AEBdistance
        %         frontDistance
        %         vehicleInFrontId
        status
        %         emergencyCase
        %         stop
        %         collided
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
        modelName
    end
    
    methods
        function obj = Vehicle(id, car_name,startingPoint,destinationPoint,startingTime,maxSpeed,size,dataLinkV2V,dataLinkV2I,mass,simSpeed,frontSensorRange,AEBdistance,minDeceleration)
            obj.id = id;
            obj.simSpeed = simSpeed;
            obj.name = car_name;
            
            obj.physics.size = size; %Should be edited according to the vehicle
            obj.physics.mass = mass; %kg Should be edited according to the vehicle
            
            obj.dynamics.position = [0 0 0];
            obj.dynamics.speed = 0;
            obj.dynamics.maxSpeed =maxSpeed;
            obj.dynamics.directionVector=[0 0 0];
            obj.dynamics.cornering.angles = 0;
            obj.dynamics.cornering.iterator = 1;
            obj.dynamics.orientation = [0 1 0 0];
            obj.dynamics.minDeceleration = minDeceleration;
            
            obj.sensors.frontSensorRange= frontSensorRange;
            obj.sensors.AEBdistance = AEBdistance;
            obj.sensors.frontDistance = 1000;
            obj.sensors.vehicleInFrontId =0;
            
            obj.status.emergencyCase = 0;
            obj.setStopStatus(true);
            obj.status.collided = 0;
            
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
            obj.pathInfo.path = 0;
            obj.pathInfo.BOGPath = [];
            
            obj.dataLog.timeStamps =[];
            obj.dataLog.totalTravelTime = 0;
            obj.dataLog.speedInCrossroad = [];
            obj.dataLog.speedInCrossroad2 = [];
            obj.dataLog.speed = [];
            obj.dataLog.emergencyCase = [];
            obj.dataLog.platooning = [];
            
            obj.map = evalin('base','Map');
            
            %obj.decisionUnit = DecisionUnit;
            obj.decisionUnit.Map = evalin('base','Map');
            obj.decisionUnit.simSpeed = evalin('base','simSpeed');
            obj.decisionUnit.accelerationPhase = zeros(1,5);
            obj.decisionUnit.initialFutureData = [];
            obj.decisionUnit.futureData = [];
            obj.decisionUnit.breakingFlag = 0;
            obj.decisionUnit.inCrossroad = [0 0];
            
            obj.V2VdataLink = dataLinkV2V;
            obj.V2IdataLink = dataLinkV2I;
            obj.V2I = V2I(id, dataLinkV2I);
            
            obj.modelName = evalin('base','modelName');
            
            
        end %Constructor
        
        function car = initVehicle(car)
            car.dynamics.position = car.map.get_coordinates_from_waypoint(car.pathInfo.lastWaypoint);
            
            
        end
        
        function car = setDestination(car, destination, global_timesteps)
            
            car.pathInfo.destinationPoint = destination;
            car.dataLogger.setDestinationDataLogger(global_timesteps, car.pathInfo.lastWaypoint);
            car.setStopStatus(false);
            car.destinationReached = false;
        end
   
        function checkCollision(car,Vehicles)
            %this function should be seperate from the sensor because sensor can be faulty but collision will appear as
            %an accident so to create the conditions for instant stop we need to keep it as a seperate function
            
            % Car's hitbox can be calculated once.
            Box1 = getHitbox(car);
            
            for i=1:length(Vehicles)  
                if car.id == Vehicles(i).id
                    break; 
                    %This "break" prevents double checks, for example: V1 doesn't check at all, 
                    %V4 checks all before until itself, V10 checks with everyone before it.
                else                           
                    Box2 = getHitbox(Vehicles(i));
                    hasCollided = checkIntersection(car,Box1,Box2);  
                    if hasCollided == true
                        % Collision occurs
                        car.status.emergencyCase = 3;
                        car.status.collided = 1;
                        car.setStopStatus(true);
                        car.dynamics.speed = 0;
                        
                        Vehicles(i).status.emergencyCase =3;
                        Vehicles(i).status.collided =1;
                        Vehicles(i).setStopStatus(true);
                        Vehicles(i).dynamics.speed = 0;
                    end
                    
                    
                end
                
                
                % here we need to use car.main.translation values, not the information of the Decision Unit. In a
                % fault-free simulation both data would be the same.
                
                % if collision happens, car.status.emergencyCase is set to "3"
            end
            
            
        end
        
        function Hitbox = getHitbox(car)
                    
            centerP = [car.dynamics.position(1); -car.dynamics.position(3)];
            angle = car.dynamics.orientation(4); % it isn't always correct on corners
            a = car.physics.size(1)/2;
            b = car.physics.size(3)/2;
            
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
                    car.pathInfo.destinationReached = true;
                    reached = true;
                    car.setStopStatus(true);
                    car.pathInfo.routeCompleted = true;
                    car.dynamics.speed = 0;
                    car.dataLog.totalTravelTime = get_param(car.modelName,'SimulationTime');
                    car.V2VdataLink(car.V2VdataLink==1) =0;                    
                end
            end            
        end
        
        %% Estimator of the vehicle to calculate the ETA at the crossroad
        
        function timeToReach = calculateEstimatedTimeOfArrival(obj,stoppingNode,ETAcarInFront,platooningTimeBehind,global_timesteps)
            distToConflictZone = norm(obj.dynamics.position -  obj.map.get_coordinates_from_waypoint(stoppingNode)) + 100;
            % if vehicle is in acceleration phase
            if abs(obj.dynamics.speed - obj.dynamics.maxSpeed)>1 
                averageAcceleration = obj.simSpeed^2 * NN_acceleration([obj.dynamics.speed; obj.dynamics.maxSpeed-obj.dynamics.speed]); % NN_acceleration -> can be replaced by getAverageAcceleration(speedFrom, speedTo) 
                accelerationDistance =  obj.simSpeed * getAccelerationDistance(obj, averageAcceleration, obj.dynamics.speed, obj.dynamics.maxSpeed);
                
                if accelerationDistance < distToConflictZone
                    % if the acceleration distance is below the distance to
                    % the conflict zone we have to calculate t1 (time in
                    % accleration phase) and t2 (time in constant speed
                    % phase)
                    t1 = timeToReachNextWaypointInAccelerationPhase(obj, obj.dynamics.speed, averageAcceleration, accelerationDistance/obj.simSpeed);
                    t2 = 1/obj.simSpeed * (distToConflictZone-accelerationDistance)/obj.dynamics.maxSpeed;
                    timeToReach = t1+t2+global_timesteps;
                else
                    % if the acceleration distance is above the distance to
                    % the conflict zone we can immediately calculate the
                    % time to reach
                    timeToReach = timeToReachNextWaypointInAccelerationPhase(obj, obj.dynamics.speed, averageAcceleration, distToConflictZone/obj.simSpeed)+global_timesteps;
                    
                end
            else
                % if vehicle is not in accleration phase simple constant
                % speed phase calculation
                timeToReach = 1/obj.simSpeed * distToConflictZone/obj.dynamics.speed + global_timesteps;
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
        
        function accelerationDistance = getAccelerationDistance(obj, averageAcceleration, currentSpeed, speedTo)
            delta_v = speedTo-currentSpeed;
            accelerationDistance = delta_v^2/(2*averageAcceleration)+currentSpeed*delta_v/averageAcceleration;
        end
        %% Eigenvalues of Equation 8 in NecSys Paper
        function timeToReach = timeToReachNextWaypointInAccelerationPhase(obj, currentSpeed, averageAcceleration, distance)
            timeToReach = -currentSpeed/averageAcceleration + sqrt((currentSpeed/averageAcceleration)^2+2*distance/averageAcceleration);
        end
        
        % To control the change in the status because it happens in so many different places
        function setStopStatus(car, binary)
            car.status.stop = binary;   
        end
        
        function setCurrentRoute(car, RouteID)
            car.pathInfo.currentRoute = RouteID;
        end
    end
end

