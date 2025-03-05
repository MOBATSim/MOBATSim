classdef Vehicle < handle
    %VEHICLE Vehicle Class is used to generate Vehicle instances for each vehicle in the simulation.
    %
    
    % Contributors: Mustafa Saraoglu, Fabian Hart, Qihang Shi, Johannes Pintscher, Laura Slabon
    
    % VEHICLE Methods:
    %    Vehicle - Constructor Method   - Generates the vehicle with the initial values as inputs
    %    checkWaypointReached           - Checks if the vehicle has reached its next waypoint
    %    getRouteFromPath               - Derives the route information from the path
    %    getCurrentRouteID              - Finds the current route ID from the path and the last waypoint information
    %    checkCollisionBetweenVehicles  - Checks if a collision has happened with another vehicle
    %    getHitbox                      - Gets the hitbox of vehicles to check for collision
    %    vehiclesCollide                - Stops the vehicles if a collision has happened
    %    checkIntersection              - Checks if hitboxes intersect to raise the collision flag
    %    checkifDestinationReached      - Checks if the vehicle has reached its destination waypoint
    
    % SET Methods:
    %    setStopAtNextWaypoint          - Makes the vehicle stop at the next waypoint
    %    setStopStatus                  - Stops the vehicle
    %    setCurrentRouteID              - Sets the current route ID
    %    setPath                        - Sets the current path
    %    setVehicleSensorDetection      - Registers the front and rear vehicle detections
    %    setDrivingMode                 - Sets the vehicle's driving mode
    %    setCollided                    - Changes the vehicle's collided status
    %    setCurrentRoute                - Sets the current route for the vehicle according to the format the route data
    %    updateActualSpeed              - Sets the vehicle dynamics speed value
    %    updateVehicleFrenetPosition    - Updates vehicle's <s,d> values according to its own Frenet Frame
    %    setLastWaypoint                - Sets the last waypoint
    %    setRouteCompleted              - Sets the value for when a route is completed
    %    setDestinationReached          - Sets the value for when the destination is reached
    %    setYawAngle                    - Sets the yaw angle value
    %    setPosition                    - Sets the position value
    %    setInitialRoute                - Sets the initial route for a vehicle to start
    
    properties
        id              %    id - Unique id of a vehicle
        name            %    name - A custom name for a vehicle
        physics
        %         size
        %         mass
        dynamics
        %         position
        %         speed
        %         maxSpeed
        %         orientation
        %         minDeceleration
        %         acceleration                  acceleration applied to model
        %         steeringAngle                 steering angle applied to model
        sensors
        %         safeDistance
        %         frontSensorRange
        %         AEBdistance
        %         leadingVehicleId
        %         distanceToLeadingVehicle
        %         leadingVehicleSpeed
        %         leadingVehicle
        %         rearVehicleSafetyMargin
        status
        %         drivingMode
        %         stop
        %         collided
        %         ttc                           time to collision
        %         brakingFlag                   vehicle stops at next waypoint
        %         atIntersection                at which Intersection and in which state
        pathInfo
        %         currentRoute
        %         currentRouteID
        %         destinationReached
        %         stopAt
        %         startingPoint
        %         destinationPoint
        %         lastWaypoint
        %         routeCompleted
        %         calculateNewPathFlag
        %         path
        %         pathChangedCounter
        %         futureData
        %         BOGPath
        %         routeLength
        %         laneId                        Left Lane = 1, in between = 0.5, Right Lane = 0
        %         s
        %         d
        %         routeEndDistance
        V2I
        V2VdataLink
        V2IdataLink
        map             %    map - Every vehicle takes an instance of the map
        collisionReport % Command Window output: 0 -> off, 1 -> on, 2-> report only collisions at intersections
    end
    
    methods
        function obj = Vehicle(id, car_name,startingPoint,destinationPoint,maxSpeed,size,dataLinkV2V,dataLinkV2I,mass,frontSensorRange,AEBdistance,minDeceleration, map, collisionReport)
            obj.id = id;
            obj.name = car_name;
            
            obj.physics.size = size; %Should be edited according to the vehicle
            obj.physics.mass = mass; %kg Should be edited according to the vehicle
            
            obj.dynamics.position = [0 0 0];
            obj.dynamics.speed = 0;
            obj.dynamics.maxSpeed =maxSpeed;
            obj.dynamics.orientation = [0 1 0 0];
            obj.dynamics.minDeceleration = minDeceleration;
            obj.dynamics.acceleration = 0;
            obj.dynamics.steeringAngle = 0;
            
            obj.sensors.safeDistance = 5;
            obj.sensors.frontSensorRange            = frontSensorRange;
            obj.sensors.AEBdistance                 = AEBdistance;
            obj.sensors.leadingVehicleId            = 0;
            obj.sensors.distanceToLeadingVehicle    = Inf;
            obj.sensors.leadingVehicleSpeed         = -1;
            obj.sensors.leadingVehicle = [];
            obj.sensors.rearVehicleSafetyMargin = 1000; % Only used by the Ego Vehicle at the moment
            
            obj.setDrivingMode(0);
            obj.setStopStatus(true);
            obj.setCollided(false);
            obj.status.ttc = 1000;
            obj.status.brakingFlag = 0;
            obj.status.atIntersection = [0 "x"];
            
            obj.pathInfo.currentRoute = [];
            obj.pathInfo.currentRouteID =0;
            obj.pathInfo.destinationReached = false;
            obj.pathInfo.stopAt = 0;
            obj.pathInfo.startingPoint = startingPoint;
            obj.pathInfo.destinationPoint = destinationPoint;
            obj.pathInfo.lastWaypoint = startingPoint;
            obj.pathInfo.routeCompleted = true;
            obj.pathInfo.calculateNewPathFlag = true;
            obj.pathInfo.path = 0;
            obj.pathInfo.pathChangedCounter =0;
            obj.pathInfo.futureData = [];
            obj.pathInfo.BOGPath = [];
            obj.pathInfo.routeLength = 0;
            obj.pathInfo.laneId = 0;  % Left Lane = 1, in between = 0.5, Right Lane = 0
            obj.pathInfo.s = 0;
            obj.pathInfo.d = 0;
            obj.pathInfo.routeEndDistance = [];
            
            obj.map = map;
            
            obj.collisionReport =collisionReport;
            
            
            obj.V2VdataLink = dataLinkV2V;
            obj.V2IdataLink = dataLinkV2I;
            
            obj.setPosition(obj.map.get_coordinates_from_waypoint(startingPoint));
            obj.setYawAngle(obj.map.getInitialYawAnglefromWaypoint(startingPoint));
        end %Constructor
        
        function bool = checkWaypointReached(car,Destination)
            % Consider waypoint/destination reached when the distance is smaller than a threshold (1 meter).
            if (car.pathInfo.routeEndDistance<1) || (norm(Destination-car.dynamics.position)<1)
                lastWaypoint = car.map.get_waypoint_from_coordinates(Destination);
                car.setLastWaypoint(lastWaypoint); % Vehicle Set
                
                if ~(car.checkifDestinationReached()) % Check if destination reached
                    % If not -> find the next route ID and set the current route for the vehicle
                    car.pathInfo.s = 0;%reset s at the end of road
                    nextRouteID = car.getCurrentRouteID(car.pathInfo.path,lastWaypoint);
                    car.setCurrentRouteID(nextRouteID);
                    car.pathInfo.path(1) = [];
                    currentRoute = car.getRouteFromPath(car.pathInfo.path);
                    car.setCurrentRoute(currentRoute); % Vehicle - Set Functions
                    car.pathInfo.calculateNewPathFlag = 1; % Path Planner should calculate a new path to account for road changes
                end
                bool = true;
            else
                bool = false;
            end
        end
        
        function currentRoute = getRouteFromPath(car,path)
            % Straight route segment
            if (isempty(find((car.map.connections.translation(:,1) == path(1) )&(car.map.connections.translation(:,2)== path(2)), 1 )) == false)
                index = find((car.map.connections.translation(:,1) == path(1) )&(car.map.connections.translation(:,2)== path(2)) );
                currentRoute = [car.map.waypoints(car.map.connections.translation(index,1),:);
                    car.map.waypoints(car.map.connections.translation(index,2),:);
                    zeros(1,3);
                    zeros(1,3)];
            end
            % Curved route segment
            if (isempty(find((car.map.connections.circle(:,1) == path(1) )&(car.map.connections.circle(:,2)== path(2)), 1 )) == false)
                index = find((car.map.connections.circle(:,1) == path(1) )&(car.map.connections.circle(:,2)== path(2)) );
                currentRoute = [car.map.waypoints(car.map.connections.circle(index,1),:);
                    car.map.waypoints(car.map.connections.circle(index,2),:);
                    abs(car.map.connections.circle(index,3)),car.map.connections.circle(index,4),car.map.connections.circle(index,6);
                    -sign(car.map.connections.circle(index,3))*ones(1,3)];
            end
        end
        
        function currentRouteID = getCurrentRouteID(car, path, lastWaypoint)
            
            idx = find(path==lastWaypoint);
            if idx+1<=length(path) % Next Route
                currentRouteID = car.map.getRouteIDfromPath([path(idx) path(idx+1)]);
            else % Destination Reached // CurrentRouteID stays the same
                currentRouteID = car.pathInfo.currentRouteID;
            end
        end
        
        function reached = checkifDestinationReached(car)
            reached = false;
            if ~car.pathInfo.destinationReached % If the vehicle hasn't already reached
                if car.pathInfo.lastWaypoint == car.pathInfo.destinationPoint
                    % Vehicle has reached its destination now
                    reached = car.setDestinationReached(true);
                    % Stop the vehicle
                    car.setStopStatus(true);
                    % Close the V2V connection
                    car.V2VdataLink(car.V2VdataLink==1) =0;
                end
            end
        end
        
        %% Collision Detection Functions
        function checkCollisionBetweenVehicles(vehicle,car)
            %this function should be seperate from the sensor because sensor can be faulty but collision will appear as
            %an accident so to create the conditions for instant stop we need to keep it as a seperate function
            Box1 = getHitbox(vehicle);
            Box2 = getHitbox(car);
            hasCollided = checkIntersection(vehicle,Box1,Box2);
            
            %% If collision has happened
            if hasCollided == true
                %% Collision report output -> Command Window
                if vehicle.collisionReport == 1
                    if ~vehicle.status.collided % If the vehicle just collided (not already collided before)
                        disp("Collision Between Vehicles:")
                        disp([vehicle.id car.id]);
                        % First vehicle
                        disp("Collided Vehicle:");
                        disp([vehicle.id]);
                        disp("Position:")
                        disp([vehicle.dynamics.position(1) -vehicle.dynamics.position(3) vehicle.dynamics.speed]);
                        disp("Driving Mode:")
                        disp(vehicle.status.drivingMode);
                        disp("Current Route")
                        disp(vehicle.pathInfo.currentRouteID)
                        disp("Last waypoint")
                        disp(vehicle.pathInfo.lastWaypoint)
                        if ~(vehicle.status.atIntersection(2) == "x")
                            disp(["Vehicle collided at intersection id:" num2str(vehicle.status.atIntersection(1))])
                            disp(["Vehicle's state at intersection was: " vehicle.status.atIntersection(2)])
                        end
                    end
                    
                    %% Collision report output (Only intersection) -> Command Window
                elseif vehicle.collisionReport == 2
                    if ~vehicle.status.collided % If the vehicle just collided (not already collided before)
                        if ~(vehicle.status.atIntersection(2) == "x")

                            disp("Collision Between Vehicles:")
                            disp([vehicle.id car.id]);
                            disp("Collided Vehicle:");
                            disp([vehicle.id])
                            disp(["Vehicle collided at intersection id:" num2str(vehicle.status.atIntersection(1))])
                            disp(["Vehicle's state at intersection was: " vehicle.status.atIntersection(2)])
                        end
                    end
                    
                end
                
                %% The other car has also recently collided (was not already in collision state before)
                if ~car.status.collided
                    %% Collision report output -> Command Window
                    if car.collisionReport == 1
                        disp("Collision Between Vehicles:")
                        disp([car.id vehicle.id]);
                        % Second vehicle
                        disp("Collided Vehicle:");
                        disp([car.id]);
                        disp("Position:")
                        disp([car.dynamics.position(1) -car.dynamics.position(3) car.dynamics.speed]);
                        disp("Driving Mode:")
                        disp(car.status.drivingMode);
                        disp("Current Route")
                        disp(car.pathInfo.currentRouteID)
                        disp("Last waypoint")
                        disp(car.pathInfo.lastWaypoint)
                        
                        if ~(car.status.atIntersection(2) == "x")             
                            disp(["Vehicle collided at intersection id:" num2str(car.status.atIntersection(1))])
                            disp(["Vehicle's state at intersection was: " car.status.atIntersection(2)])
                        end
                        
                        %% Collision report output (Only intersection) -> Command Window
                    elseif car.collisionReport == 2
                        if ~car.status.collided % If the vehicle just collided (not already collided before)
                            if ~(car.status.atIntersection(2) == "x")
                                
                                disp("Collision Between Vehicles:")
                                disp([car.id vehicle.id]);
                                disp("Collided Vehicle:");
                                disp([car.id]);
                                disp(["Vehicle collided at intersection id:" num2str(car.status.atIntersection(1))])
                                disp(["Vehicle's state at intersection was: " car.status.atIntersection(2)])
                            end
                        end
                        
                    end
                end
                
                %% Collision occurs
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
            
            % Uncomment this for plotting the collision scene in details
            %Hitbox = [p1r p2r p3r p4r p1r]; 
            
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
        
        %% SET/GET Functions to control the value changes in the properties
        function setStopAtNextWaypoint(car, brakingFlag)
            % set stopAt if the vehicle should stop at next waypoint
            
            if brakingFlag
                car.pathInfo.stopAt = car.pathInfo.path(2);
            else
                car.pathInfo.stopAt = 0;
            end
        end
        
        function setStopStatus(car, stop)
            car.status.stop = stop;
            if stop % If Status Stop then the speed should be zero instantly (slowing down is not factored in yet but might come with the next update)
                car.updateActualSpeed(0);
            end
        end
        
        function setCurrentRouteID(car, RouteID)
            car.pathInfo.currentRouteID = RouteID;
        end
        
        function setPath(car, newPath)
            car.pathInfo.path = newPath;
        end
        
        function setVehicleSensorDetection(car,leadingVehicle, distanceToLeading, rearVehicle, distanceToRear)
            % set the data about leading and rear vehicle
            
            %% Leading vehicle
            
            car.sensors.leadingVehicle = leadingVehicle;
            car.sensors.distanceToLeadingVehicle = distanceToLeading;
            if ~isempty(leadingVehicle) % if there is a leading vehicle
                car.sensors.leadingVehicleId = leadingVehicle.id;
                car.sensors.leadingVehicleSpeed = car.sensors.leadingVehicle.dynamics.speed;
                relSpeed = car.dynamics.speed - car.sensors.leadingVehicle.dynamics.speed;
            else
                car.sensors.leadingVehicleId = 0;
                car.sensors.leadingVehicleSpeed = -1;
                relSpeed = 0; % to get a infinite time to collision
            end
            car.status.ttc = distanceToLeading/relSpeed; % time to collision
            
            %% Rear vehicle
            
            car.sensors.rearVehicle = rearVehicle;
            if ~isempty(rearVehicle) % if there is a rear vehicle
                relSpeed = car.sensors.rearVehicle.dynamics.speed - car.dynamics.speed;
            else
                relSpeed = 0; % to get an infinite rear safety margin
            end
            car.sensors.rearVehicleSafetyMargin = distanceToRear/relSpeed;
            
        end
        
        function setDrivingMode(car, DrivingMode)
            car.status.drivingMode = DrivingMode;
        end
        
        function setCollided(car, collided)
            car.status.collided = collided;
        end
        
        function setCurrentRoute(car, currentRoute)
            car.pathInfo.currentRoute = currentRoute;
            
            % Calculate the current route length
            if currentRoute(3,1) % if curved
                startingPoint = currentRoute(1,[1 3]).*[1 -1]; % Get the starting point
                rotationCenter = currentRoute(3,[2 3]).*[1 -1]; % Get the rotation center
                R = norm(startingPoint-rotationCenter); % Get the radius of the rotation
                
                car.pathInfo.routeLength = currentRoute(3,1)*R; % Arc length := theta*Radius
            else % if straight
                car.pathInfo.routeLength = norm(currentRoute(2,:)-currentRoute(1,:)); % Straight road length
            end
        end
        
        function updateActualSpeed(car,speed)
            car.dynamics.speed = speed;
        end
        
        function updateVehicleFrenetPosition(car, s,d)
            car.pathInfo.s = s; % Driven longitudinal length in Frenet frame
            car.pathInfo.d = d; % Lateral offset in Frenet frame
            car.pathInfo.routeEndDistance = car.pathInfo.routeLength-s; % Distance until the end of the current route
        end
        
        function setLastWaypoint(car,lastWaypoint)
            car.pathInfo.lastWaypoint = lastWaypoint;
        end
        
        function setBrakingFlag(car,stopFlag)
            car.status.brakingFlag = stopFlag;
        end
        
        function bool = setRouteCompleted(car,bool)
            car.pathInfo.routeCompleted = bool;
        end
        
        function bool = setDestinationReached(car,bool)
            car.pathInfo.destinationReached = bool;
            car.setRouteCompleted(bool);
            
        end
        
        function setYawAngle(car,YawAngle)
            % Set the yaw angle in rad
            car.dynamics.orientation = [0 1 0 YawAngle];
        end
        
        function setPosition(car,newPosition)
            car.dynamics.position = newPosition;
        end
        
        function setInitialRoute(car)
            % The Vehicle determines the initial route
            nextRoute = car.getCurrentRouteID(car.pathInfo.path,car.pathInfo.lastWaypoint);
            currentRoute = car.getRouteFromPath(car.pathInfo.path);
            
            car.setCurrentRouteID(nextRoute);           % Vehicle - Set Functions
            car.setCurrentRoute(currentRoute);          % Vehicle - Set Functions
            car.setRouteCompleted(false);               % Vehicle - Set Functions
        end
        
        
    end
end

