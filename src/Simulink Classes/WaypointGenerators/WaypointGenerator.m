classdef WaypointGenerator < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime & matlab.system.mixin.CustomIcon
    %WAYPOINTGENERATOR Superclass for waypoint generators
    %   Detailed explanation goes here
    
    properties
        Vehicle_id
    end
    
    % Pre-computed constants
    properties(Access = protected)
        vehicle
        map = evalin('base','Map');
        simSpeed = evalin('base','simSpeed');
        modelName = evalin('base','modelName');
        
        laneWidth = 3.7; % Standard road width
        curvature = 0;%curvature of the current road
        laneSwitchStartPoint = [];
        laneSwitchTargetPoint = [];
        laneSwitchStartTime = [];
        
        safetyGain = 80;%k_2 in cost function(FKFS)
        comfortGain = 0.05;%k_1 in cost function(FKFS)
        laneSwitchTime = 3;%delta_T, choosen by cost function
        latOffset = 0;%variable to save reference delta_d in Frenet coordinate
        trajPolynom_candidates = [];% condidate trajectories
        trajPolynom = [];% Trajectory choosen
        %velocityPolynom = {};% reference velocity for minimum jerk trajectory
        %accPolynom = {};%reference acc for minimum jerk trajectory
        %jerkPolynom = {};%reference jerk for minimum jerk trajectory
        
        referencePose = [0; 0; 0];
    end
    
    methods
        function obj = WaypointGenerator(varargin)
            %WAYPOINTGENERATOR Construct an instance of this class
            %   Detailed explanation goes here
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.vehicle = evalin('base',strcat('Vehicle',int2str(obj.Vehicle_id)));
        end
        
                function move_straight(obj,car,speed,Destination)
            %% Reference Waypoint Generation
            obj.generateStraightWaypoints(car)
            %%
            
            if car.pathInfo.routeEndDistance <1
                
                car.pathInfo.s = 0;
                
                lastWaypoint = car.map.get_waypoint_from_coordinates(Destination);
                
                car.setRouteCompleted(true); % Vehicle Set
                car.setLastWaypoint(lastWaypoint); % Vehicle Set
                
                nextRoute = obj.generateCurrentRoute(car,car.pathInfo.path,lastWaypoint);
                car.setCurrentRoute(nextRoute); % Vehicle Set
            end

        end
        
        function rotate_left(obj ,car, speed,Destination)
            %% Reference Waypoint Generation
            obj.generateLeftRotationWaypoints(car);
            %%

            if car.pathInfo.routeEndDistance <1% consider to reach the endpoint when distance smaller than a threshold. Threshold defined by the user
                car.pathInfo.s = 0;%reset s at the end of road
                
                lastWaypoint = car.map.get_waypoint_from_coordinates(Destination);
                
                car.setRouteCompleted(true);% Vehicle Set
                car.setLastWaypoint(lastWaypoint); % Vehicle Set
                
                nextRoute = obj.generateCurrentRoute(car,car.pathInfo.path,lastWaypoint);
                car.setCurrentRoute(nextRoute); % Vehicle Set
                
                
            end
            
        end
        
        function rotate_right(obj ,car,speed,Destination)
            %% Reference Waypoint Generation
            obj.generateRightRotationWaypoints(car);
            %%
            
            if car.pathInfo.routeEndDistance <1% consider to reach the endpoint when distance smaller than a threshold. Threshold defined by the user
                car.pathInfo.s = 0;%reset s at the end of road
                
                lastWaypoint = car.map.get_waypoint_from_coordinates(Destination);
                
                car.setRouteCompleted(true);% Vehicle Set
                car.setLastWaypoint(lastWaypoint); % Vehicle Set
                
                nextRoute = obj.generateCurrentRoute(car,car.pathInfo.path,lastWaypoint);
                car.setCurrentRoute(nextRoute);
                

            end
            
            
        end
        
        
        function currentRoute = generateCurrentRoute(~,car, path, lastWaypoint)
            
            idx = find(path==lastWaypoint);
            if idx+1<=length(path) % Next Route
                currentRoute = car.map.getRouteIDfromPath([path(idx) path(idx+1)]);
            else % Destination Reached // CurrentRoute stays the same
                currentRoute = car.pathInfo.currentRoute;
            end
        end
        
        function currentTrajectory = generateTrajectoryFromPath(~,car,path)
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
        
              
        function [position_Cart,orientation_Cart] = Frenet2Cartesian(obj,route,s,d,radian)
            
            %this function transfer a position in Frenet coordinate into Cartesian coordinate
            %input:
            %route is a 2x2 array [x_s y_s;x_e y_e]contains the startpoint and the endpoint of the road
            %s is the journey on the reference roadline(d=0)
            %d is the vertical offset distance to the reference roadline,positive d means away from center
            %radian is the radian of the whole curved road,is positive when
            %counterclockwise turns
            %output:
            %position_Cart is the 1x2 array [x y] in Cartesian coordinate
            %orientation_Cart is the angle of the tangent vector on the reference roadline and the x axis of cartesian
            %detail information check Frenet.mlx
            startPoint = route(1,:);
            endPoint = route(2,:);
            if radian == 0%straight road
                route_Vector = endPoint-startPoint;
                local_route_Vector_i = route_Vector/norm(route_Vector);% unit vector of the route_vector
                orientation_Cart = atan2(local_route_Vector_i(2),local_route_Vector_i(1)); % reverse tangent of unit vector
                sideVector = [cos(orientation_Cart+pi/2) sin(orientation_Cart+pi/2)];%vector of the tangent line of reference line
                position_Cart = s*local_route_Vector_i+d*sideVector+startPoint;% position= start point + length of journey
            else %curved road
                r = sqrt((norm(endPoint-startPoint))^2/(1-cos(radian))/2);%The radius of the road segment， according to the law of the cosines
                targetVector = (endPoint-startPoint)/norm(endPoint-startPoint); %Unit vector of route vector (p in Frenet.xml)
                beta = atan2(targetVector(2),targetVector(1)); %the angle of target vector and x axis in cartesian coordinate (theta 1 in Frenet.xml)
                plumbLength = cos(radian/2)*r; % the distance from circle center to targetVector (OG in Frenet.xml)
                plumbVector = [cos(beta+sign(radian)*pi/2) sin(beta+sign(radian)*pi/2)]*plumbLength;
                center = startPoint + targetVector*norm(endPoint-startPoint)/2 + plumbVector;%rotation center of the road in Cartesian coordinate
                startPointVector = startPoint-center;%OP1 in Frenet.xml
                startPointVectorAng = atan2(startPointVector(2),startPointVector(1));
                l = r+d;%current distance from rotation center to position
                lAng = sign(radian)*s/r+startPointVectorAng;% the angle of vector l
                position_Cart = l*[cos(lAng) sin(lAng)]+center;% the position in Cartesion coordinate
                orientation_Cart = lAng+sign(radian)*pi/2;
                orientation_Cart = mod(orientation_Cart,2*pi);
                orientation_Cart = orientation_Cart.*(0<=orientation_Cart & orientation_Cart <= pi) + (orientation_Cart - 2*pi).*(pi<orientation_Cart & orientation_Cart<2*2*pi);   % angle in (-pi,pi]
            end
        end
        
        function [s,d,orientation_Cart,routeLength] = Cartesian2Frenet(obj,route,position_C,radian)
            
            %this function transform a position in Cartesian coordinate into Frenet coordinate
            
            %input:
            %route is a 2x2 array [x_s y_s;x_e y_e]contains the startpoint and the endpoint of the road
            %position_C is the 1x2 array [x y] in Cartesian coordinate
            %radian is the radian of the whole curved road,is positive when
            %counterclockwise turn
            
            %output:
            %orientation_C is the angle of the tangent vector on the reference roadline(d=0)
            %s is the journey on the reference roadline
            %d is the vertical offset distance to the reference roadline,positive d means away from center
            %this function follows the similar logic with function Frenet2Cartesian(obj,route,s,d,radian)
            startPoint = route(1,:);
            endPoint = route(2,:);
            if radian == 0%straight road
                route_Vector = endPoint-startPoint;
                local_route_Vector_i = route_Vector/norm(route_Vector);
                orientation_Cart = atan2(local_route_Vector_i(2),local_route_Vector_i(1));% orientation angle
                posVector = position_C-startPoint;
                s = dot(posVector,local_route_Vector_i);% the projection of posVector on local_route_vector_i
                sideVector = [cos(orientation_Cart+pi/2) sin(orientation_Cart+pi/2)];% side vector is perpendicular to the route
                d = dot(posVector,sideVector);% the projection of posVector on sideVector
                routeLength = norm(endPoint-startPoint);% the length of the route_Vector
                obj.curvature = 0;
            else
                r = sqrt((norm(endPoint-startPoint))^2/(1-cos(radian))/2);%The radius of the road segment， according to the law of the cosines
                targetVector = (endPoint-startPoint)/norm(endPoint-startPoint);%Unit vector of route vector (p in Frenet.xml)
                beta = atan2(targetVector(2),targetVector(1));%the angle of target vector and x axis in cartesian coordinate (theta 1 in Frenet.xml)
                plumbLength = cos(radian/2)*r;% the distance from circle center to targetVector (OG in Frenet.xml)
                plumbVector = [cos(beta+sign(radian)*pi/2) sin(beta+sign(radian)*pi/2)]*plumbLength;
                center = startPoint + targetVector*norm(endPoint-startPoint)/2 + plumbVector;%rotation center of the road in Cartesian coordinate
                startPointVector = startPoint-center;% vector OP_1 in Frenet.xml

                l = position_C-center;% the vector from rotation center to position
                d = norm(l)-r;
                lAng = atan2(l(2),l(1)); % the angle of vector l with x axis (phi 3 in Frenet.xml)

                start_dot_l = dot(startPointVector,l);% |startPointVetor|*|l|*sin(angle)
                start_cross_l = sign(radian)*(startPointVector(1)*l(2)-startPointVector(2)*l(1));% |startPointVetor|*|l|*cos(angle)
                angle = atan2(start_cross_l,start_dot_l);% the angle between startPointVector and vector l, tan(angle) = start_dot_l/start_cross_1
                if mod(angle,2*pi) > abs(radian)% judge if the radian of the angle bigger than the radian of the road
                    start_cross_l = -(startPointVector(1)*l(2)-startPointVector(2)*l(1));
                    angle = -atan2(start_cross_l,start_dot_l);
                end
                s = angle*r;
                routeLength = abs(radian)*r;
                orientation_Cart = lAng+sign(radian)*pi/2;% the orientation of the current point of the road(phi 4 in Frenet.xml) in cartesian coordinate
                orientation_Cart = mod(orientation_Cart,2*pi);% orientation can not bigger than 2pi
                orientation_Cart = orientation_Cart.*(0<=orientation_Cart & orientation_Cart <= pi) + (orientation_Cart - 2*pi).*(pi<orientation_Cart & orientation_Cart<2*2*pi);   % angle in (-pi,pi]
                obj.curvature = 1/r;
            end
        end
        

    end
    methods(Static,Access = protected)
        
    end
    
        %% Abstract Methods / Must be implemented by Subclasses
    methods (Abstract, Access = protected)

        % Every Waypoint generator should generate Waypoints in their own way
        generateStraightWaypoints(obj,car)
        generateLeftRotationWaypoints(obj,car)
        generateRightRotationWaypoints(obj,car)
        
    end
end

