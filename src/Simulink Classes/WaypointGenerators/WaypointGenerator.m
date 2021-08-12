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
                
        function [refPos,refOrientation] = Frenet2Cartesian(~,currentTrajectory,s,d,radian)
            
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
            route = currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];
            cclockwise = currentTrajectory(4,1);
            startPoint = route(1,:);
            endPoint = route(2,:);
            %d = -d;
            if radian == 0%straight road
                route_Vector = endPoint-startPoint;
                local_route_Vector_i = route_Vector/norm(route_Vector);% unit vector of the route_vector
                refOrientation = atan2d(local_route_Vector_i(2),local_route_Vector_i(1)); % reverse tangent of unit vector
                %sideVector = [cosd(refOrientation+90) sind(refOrientation+90)];%vector of the tangent line of reference line
                %refPos = s*local_route_Vector_i+d*sideVector+startPoint;% position= start point + length of journey
                refPos = (s+0.01)*local_route_Vector_i+startPoint;
            else %curved road
                rotationCenter = currentTrajectory(3,[2 3]).*[1 -1]; % Get the rotation center
                r = norm(startPoint-rotationCenter); % Get the radius of the rotation
                
                %targetVector = (endPoint-startPoint)/norm(endPoint-startPoint); %Unit vector of route vector (p in Frenet.xml)
                %beta = atan2(targetVector(2),targetVector(1)); %the angle of target vector and x axis in cartesian coordinate (theta 1 in Frenet.xml)
                %plumbLength = cos(radian/2)*r; % the distance from circle center to targetVector (OG in Frenet.xml)
                %plumbVector = [cos(beta+sign(radian)*pi/2) sin(beta+sign(radian)*pi/2)]*plumbLength;
                %center = startPoint + targetVector*norm(endPoint-startPoint)/2 + plumbVector;%rotation center of the road in Cartesian coordinate
                startPointVector = startPoint-rotationCenter;%OP1 in Frenet.xml
                startPointVectorAng = atan2(startPointVector(2),startPointVector(1));
                l = r-(d*cclockwise);%current distance from rotation center to position
                lAng = sign(radian)*s/r+startPointVectorAng;% the angle of vector l
                refPos = l*[cos(lAng) sin(lAng)]+rotationCenter;% the position in Cartesion coordinate
                refOrientation = lAng+sign(radian)*pi/2;
                refOrientation = mod(refOrientation,2*pi);
                refOrientation = refOrientation.*(0<=refOrientation & refOrientation <= pi) + (refOrientation - 2*pi).*(pi<refOrientation & refOrientation<2*2*pi);   % angle in (-pi,pi]
                refOrientation = rad2deg(refOrientation);
            end
        end
        
        function [s,d] = Cartesian2Frenet(obj,currentTrajectory,Vpos_C)
            %Transform a position in Cartesian coordinate into Frenet coordinate
            
            %Function Inputs:
            %route:                 2x2 array [x_s y_s;x_e y_e] the starting point and the ending point of the road
            %vehiclePos_Cartesian:  1x2 array [x y] in Cartesian coordinate
            %radian:                The angle of the whole curved road, positive for counterclockwise turn
            
            %Function Output:
            %yawAngle_in_Cartesian: The angle of the tangent vector on the reference roadline(d=0)
            %s:                     Traversed length along the reference roadline
            %d:                     Lateral offset - positive d means to the left of the reference road 
            
            route = currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];%Start- and endpoint of the current route
            radian = currentTrajectory(3,1);%radian of the curved road, is 0 for straight road
            Route_StartPoint = route(1,:);
            Route_endPoint = route(2,:);
            
            
            if radian == 0%straight road
                route_Vector = Route_endPoint-Route_StartPoint;
                route_UnitVector = route_Vector/norm(route_Vector);
                posVector = Vpos_C-Route_StartPoint; % Vector pointing from the route start point to the vehicle
                
                % Calculate "s" the longitudinal traversed distance
                s = dot(posVector,route_UnitVector);% the projection of posVector on route_UnitVector
                
                % Calculate "d" the lateral distance to the reference road frame
                yawAngle_in_Cartesian = atan2d(route_UnitVector(2),route_UnitVector(1));% orientation angle of the vehicle in Cartesian Coordinate
                sideVector = [cosd(yawAngle_in_Cartesian+90) sind(yawAngle_in_Cartesian+90)];% side vector is perpendicular to the route
                
                d = dot(posVector,sideVector);% the projection of posVector on sideVector - positive d value means to the left
                
            else % Curved Road
                
                rotationCenter = currentTrajectory(3,[2 3]).*[1 -1]; % Get the rotation center
                r = norm(Route_StartPoint-rotationCenter); % Get the radius of the rotation
                startPointVector = Route_StartPoint-rotationCenter;% vector OP_1 in Frenet.xml
                
                
                posVector = Vpos_C-rotationCenter;% the vector from rotation center to position
                % currentTrajectory(4,1) == -1 for counterclockwise, +1 for clockwise route
                d=(norm(posVector)-r)*currentTrajectory(4,1); % Previously: d = abs(r-norm(posVector));
                
                
                %start_dot_l = dot(startPointVector,posVector);% |startPointVetor|*|l|*sin(angle)
                %start_cross_l = sign(radian)*(startPointVector(1)*posVector(2)-startPointVector(2)*posVector(1));% |startPointVetor|*|l|*cos(angle)
                % TODO: Check + - according to radian and
                % obj.vehicle.pathInfo.currentTrajectory values
                angle = real(acos(dot(posVector,startPointVector)/(norm(posVector)*norm(startPointVector))));
%                 angle = -atan2(start_cross_l,start_dot_l);% the angle between startPointVector and vector l, tan(angle) = start_dot_l/start_cross_1
%                 if mod(angle,2*pi) > abs(radian)% judge if the radian of the angle bigger than the radian of the road
%                     start_cross_l = -(startPointVector(1)*posVector(2)-startPointVector(2)*posVector(1));
%                     angle = -atan2(start_cross_l,start_dot_l);
%                 end
                s = angle*r;
            end
        end
        
        function [s,d,yawAngle_in_Cartesian] = OLD_Cartesian2Frenet(obj,route,vehiclePos_Cartesian,radian)
            
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
                
                route_Vector = Route_endPoint-Route_StartPoint;
                route_UnitVector = route_Vector/norm(route_Vector);
                yawAngle_in_Cartesian = atan2(route_UnitVector(2),route_UnitVector(1));% orientation angle of the vehicle in Cartesian Coordinate
                posVector = vehiclePos_Cartesian-Route_StartPoint;
                sideVector = [cos(yawAngle_in_Cartesian+pi/2) sin(yawAngle_in_Cartesian+pi/2)];% side vector is perpendicular to the route
                
                s = dot(posVector,route_UnitVector);% the projection of posVector on local_route_vector_i
                
                d = dot(posVector,sideVector);% the projection of posVector on sideVector                
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
                yawAngle_in_Cartesian = lAng+sign(radian)*pi/2;% the orientation of the current point of the road(phi 4 in Frenet.xml) in cartesian coordinate
                yawAngle_in_Cartesian = mod(yawAngle_in_Cartesian,2*pi);% orientation can not bigger than 2pi
                yawAngle_in_Cartesian = yawAngle_in_Cartesian.*(0<=yawAngle_in_Cartesian & yawAngle_in_Cartesian <= pi) + (yawAngle_in_Cartesian - 2*pi).*(pi<yawAngle_in_Cartesian & yawAngle_in_Cartesian<2*2*pi);   % angle in (-pi,pi]
            end
        end
        
        
    end
end

