classdef LocalTrajectoryPlanner < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime & matlab.system.mixin.CustomIcon
    %LocalTrajectoryPlanner Superclass for generating necessary inputs for the lateral controllers.
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
        function obj = LocalTrajectoryPlanner(varargin)
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
            
            if radian == 0%straight road
                route_Vector = endPoint-startPoint;
                routeUnitVector = route_Vector/norm(route_Vector);% unit vector of the route_vector
                refOrientation = atan2d(routeUnitVector(2),routeUnitVector(1)); % reverse tangent of unit vector
                refPos = s*routeUnitVector+startPoint; % The ref position on the path with 0.01 "s" ahead
            else %curved road
                rotationCenter = currentTrajectory(3,[2 3]).*[1 -1]; % Get the rotation center
                startPointVector = startPoint-rotationCenter;% Vector pointing from the rotation point to the start
                
                r = norm(startPointVector); % Get the radius of the rotation
                
                startPointVectorAng = atan2(startPointVector(2),startPointVector(1)); % The initial angle of the starting point
                l = r-(d*cclockwise);% Vehicle reference lateral distance from the rotation center (d=0 for the right lane)
                lAng = (-cclockwise)*s/r+startPointVectorAng;% the angle of vector l
                refPos = rotationCenter+ l*[cos(lAng) sin(lAng)];% the reference position in Cartesian coordinates

                refOrientation = rad2deg(lAng+(-cclockwise)*pi/2);
                
                % refOrientation = lAng+(-cclockwise)*pi/2;
                % refOrientation = mod(refOrientation,2*pi);
                % refOrientation = refOrientation.*(0<=refOrientation & refOrientation <= pi) + (refOrientation - 2*pi).*(pi<refOrientation & refOrientation<2*2*pi);   % angle in (-pi,pi]
                
            end
        end
        
        function [s,d] = Cartesian2Frenet(~,currentTrajectory,Vpos_C)
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
            
            if currentTrajectory(3,1) == 0  % Straight road := radian value equals to zero
                route_Vector = route(2,:)-route(1,:);
                route_UnitVector = route_Vector/norm(route_Vector);
                posVector = Vpos_C-route(1,:); % Vector pointing from the route start point to the vehicle
                
                % Calculate "s" the longitudinal traversed distance
                s = dot(posVector,route_UnitVector);% the projection of posVector on route_UnitVector
                
                % Calculate "d" the lateral distance to the reference road frame
                %yawAngle_in_Cartesian = atan2d(route_UnitVector(2),route_UnitVector(1));% orientation angle of the vehicle in Cartesian Coordinate
                %sideVector = [cosd(yawAngle_in_Cartesian+90) sind(yawAngle_in_Cartesian+90)];% side vector is perpendicular to the route
                
                normalVector = [-route_UnitVector(2),route_UnitVector(1)];% Fast rotation by 90 degrees to find the normal vector  
                
                d = dot(posVector,normalVector);% the projection of posVector on sideVector - positive d value means to the left
                
            else % Curved Road
                
                rotationCenter = currentTrajectory(3,[2 3]).*[1 -1]; % Get the rotation center
                startPointVector = route(1,:)-rotationCenter;% Vector pointing to the start of the route from the rotation center
                r = norm(startPointVector); % Get the radius of the rotation
                
                posVector = Vpos_C-rotationCenter;% the vector from rotation center pointing the vehicle
                l = norm(posVector);
                
                % currentTrajectory(4,1) == +1 for CounterClockwise direction 
                d=(l-r)*currentTrajectory(4,1); % lateral displacement of the vehicle from the d=0 reference road (arc)
                              
                angle = real(acos(dot(posVector,startPointVector)/(l*r))); % Angle between vectors
                % One possible issue is that arccos doesn't distinguish the vector being ahead or behind, therefore when the 
                % vehicle starts the new route, it might get a positive angle and a positive "s" value instead of a negative 
                % one. This can be resolved using a third reference, the vector from the rotation point to the end of the 
                % route but for the performance this is not implemented here.
                
                s = angle*r; % Traversed distance along the reference arc
            end
        end
        
    end
end

