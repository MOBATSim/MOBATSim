classdef WaypointGenerator < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime & matlab.system.mixin.CustomIcon
    %WAYPOINTGENERATOR Superclass for waypoint generators
    %   Detailed explanation goes here
    
    properties
        Vehicle_id
    end
    
    % Pre-computed constants
    properties(Access = protected)
        vehicle
        
        laneWidth = 3.7; % Standard road width
        curvature = 0;%curvature of the current road
        laneSwitchStartPoint = [];
        laneSwitchTargetPoint = [];
        laneSwitchStartTime = [];
        
        %% TODO: Some of these variables will be deleted
        safetyGain = 80;%k_2 in cost function(FKFS)
        comfortGain = 0.05;%k_1 in cost function(FKFS)
        laneSwitchTime = 4;%delta_T, choosen by cost function
        latOffset = 0;%variable to save reference delta_d in Frenet coordinate
        trajPolynom_candidates = [];% candidate trajectories
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
            obj.vehicle = evalin('base', "Vehicles(" + obj.Vehicle_id + ")");
        end
        
                 
        function registerVehiclePoseAndSpeed(~,car,pose,speed)
            car.setPosition(Map.transformPoseTo3DAnim(pose));   % Sets the vehicle position
            car.setYawAngle(pose(3));                           % Sets the vehicle yaw angle (4th column of orientation)
            car.updateActualSpeed(speed);                       % Sets the vehicle speed
        end
                
        function [position_Cart,orientation_Cart] = Frenet2Cartesian(~,route,s,d,radian)
            
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
        
        function [s,d,yawAngle_in_Cartesian,routeLength] = Cartesian2Frenet(obj,route,vehiclePos_Cartesian,radian)
            
            %this function transform a position in Cartesian coordinate into Frenet coordinate
            
            %input:
            %route is a 2x2 array [x_s y_s;x_e y_e]contains the startpoint and the endpoint of the road
            %position_C is the 1x2 array [x y] in Cartesian coordinate
            %radian is the radian of the whole curved road,is positive when
            %counterclockwise turn
            
            %output:
            %yawAngle_in_Cartesian is the angle of the tangent vector on the reference roadline(d=0)
            %s is the journey on the reference roadline
            %d is the vertical offset distance to the reference roadline,positive d means away from center
            %this function follows the similar logic with function Frenet2Cartesian(obj,route,s,d,radian)
            Route_StartPoint = route(1,:);
            Route_endPoint = route(2,:);
            
            
            if radian == 0%straight road
                obj.curvature = 0;
                
                route_Vector = Route_endPoint-Route_StartPoint;
                route_UnitVector = route_Vector/norm(route_Vector);
                yawAngle_in_Cartesian = atan2(route_UnitVector(2),route_UnitVector(1));% orientation angle of the vehicle in Cartesian Coordinate
                posVector = vehiclePos_Cartesian-Route_StartPoint;
                sideVector = [cos(yawAngle_in_Cartesian+pi/2) sin(yawAngle_in_Cartesian+pi/2)];% side vector is perpendicular to the route
                
                s = dot(posVector,route_UnitVector);% the projection of posVector on local_route_vector_i
                
                d = dot(posVector,sideVector);% the projection of posVector on sideVector
                routeLength = norm(Route_endPoint-Route_StartPoint);% the length of the route_Vector
                
            else
                r = sqrt((norm(Route_endPoint-Route_StartPoint))^2/(1-cos(radian))/2);%The radius of the road segment， according to the law of the cosines
                targetVector = (Route_endPoint-Route_StartPoint)/norm(Route_endPoint-Route_StartPoint);%Unit vector of route vector (p in Frenet.xml)
                beta = atan2(targetVector(2),targetVector(1));%the angle of target vector and x axis in cartesian coordinate (theta 1 in Frenet.xml)
                plumbLength = cos(radian/2)*r;% the distance from circle center to targetVector (OG in Frenet.xml)
                plumbVector = [cos(beta+sign(radian)*pi/2) sin(beta+sign(radian)*pi/2)]*plumbLength;
                center = Route_StartPoint + targetVector*norm(Route_endPoint-Route_StartPoint)/2 + plumbVector;%rotation center of the road in Cartesian coordinate
                startPointVector = Route_StartPoint-center;% vector OP_1 in Frenet.xml
                
                l = vehiclePos_Cartesian-center;% the vector from rotation center to position
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
                yawAngle_in_Cartesian = lAng+sign(radian)*pi/2;% the orientation of the current point of the road(phi 4 in Frenet.xml) in cartesian coordinate
                yawAngle_in_Cartesian = mod(yawAngle_in_Cartesian,2*pi);% orientation can not bigger than 2pi
                yawAngle_in_Cartesian = yawAngle_in_Cartesian.*(0<=yawAngle_in_Cartesian & yawAngle_in_Cartesian <= pi) + (yawAngle_in_Cartesian - 2*pi).*(pi<yawAngle_in_Cartesian & yawAngle_in_Cartesian<2*2*pi);   % angle in (-pi,pi]
                obj.curvature = 1/r;
            end
        end
        
        
    end
    
    %% Abstract Methods / Must be implemented by Subclasses
    methods (Abstract, Access = protected)
        
        % Every Waypoint generator should generate Waypoints in their own way
        %generateStraightWaypoints(obj,car)
        %generateLeftRotationWaypoints(obj,car)
        %generateRightRotationWaypoints(obj,car)
        
    end
end

