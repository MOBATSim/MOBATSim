classdef V_WPGenerator_Stanley < WaypointGenerator
    % This blocks generates waypoints.
    %
    
    % Pre-computed constants
    properties(Access = private)
        referencePose = [0; 0; 0];
        latOffset = 0;%variable to save reference delta_d in Frenet coordinate
    end
    
    methods
        % Constructor
        function obj = V_WPGenerator_Stanley(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@WaypointGenerator(obj);  % Inherit the setupImpl function of the Superclass @WaypointGenerator
        end
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon("WaypointGenerator.png");
        end
        
        
        
        
        function [poseOut, referencePose] = stepImpl(obj,pose,speed,changeLane)
            
            obj.registerVehiclePoseAndSpeed(obj.vehicle,pose,speed); % Sets/Registers vehicle current Pose and speed
            
            pose(3)=pose(3)*180/pi; % rad to deg
            
            %This block shouldn't run if the ego vehicle: (destinationReached or Collided)
            if obj.vehicle.status.collided || obj.vehicle.pathInfo.destinationReached
                
                poseOut=pose';                      % Output1: vehicle's actual pose
                referencePose = obj.referencePose'; % Output2: Reference pose
                
                return;
            elseif obj.vehicle.pathInfo.routeCompleted % The Vehicle determines the initial trajectory and route
                obj.vehicle.setInitialRouteAndTrajectory();
            end
            
            % Calculate helping variables for the reference path calculation
            currentTrajectory = obj.vehicle.pathInfo.currentTrajectory;
            Vpos_C = [pose(1) pose(2)];
            
            if false %(obj.vehicle.id == 10) && (obj.getCurrentTime)>6
                [s,d,routeLength,referencePose] = obj.generateReferencePose(Vpos_C,currentTrajectory);
                obj.vehicle.updateVehicleFrenetPosition(s,d,routeLength);
                obj.referencePose = referencePose;
            else
                obj.referencePose = obj.takeRoute(obj.vehicle,obj.vehicle.pathInfo.currentTrajectory);
            end
            
            % Check if the Waypoint is Reached
            obj.vehicle.checkWaypointReached(currentTrajectory(2,:));
            %Output 1: Position of the vehicle
            %Output 2: Rotation angle of the vehicle
            
            
            
            referencePose = obj.referencePose';
            poseOut=pose';
            
        end
        
        function [s,d,routeLength,referencePose] = generateReferencePose(obj,Vpos_C,currentTrajectory)
            
            route = currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];%Start- and endpoint of the current route
            radian = currentTrajectory(3,1);%radian of the curved road, is 0 for straight road
            left = currentTrajectory(4,1);
            radian = sign(left)*radian;
            
            Route_StartPoint = route(1,:);
            Route_endPoint = route(2,:);
            
            if radian == 0%straight road
                route_Vector = Route_endPoint-Route_StartPoint;
                route_UnitVector = route_Vector/norm(route_Vector);
                yawAngle_in_Cartesian = atan2(route_UnitVector(2),route_UnitVector(1));% orientation angle of the vehicle in Cartesian Coordinate
                posVector = Vpos_C-Route_StartPoint;
                
                sideVector = [cos(yawAngle_in_Cartesian+pi/2) sin(yawAngle_in_Cartesian+pi/2)];% side vector is perpendicular to the route
                sideVector = round(sideVector,5); % Round is used to eliminate very small machine errors from calculation
                
                s = dot(posVector,route_UnitVector);% the projection of posVector on route_UnitVector
                
                d = dot(posVector,sideVector);% the projection of posVector on sideVector
                routeLength = norm(Route_endPoint-Route_StartPoint);% the length of the route_Vector
                
            else % Curved Road
                
                rotationCenter = obj.vehicle.pathInfo.currentTrajectory(3,[2 3]); % Get the rotation center
                rotationCenter(2) = -rotationCenter(2); % Transform the coordinate
                r = norm(Route_StartPoint-rotationCenter); % Get the radius of the rotation
                startPointVector = Route_StartPoint-rotationCenter;% vector OP_1 in Frenet.xml
                
                
                l = Vpos_C-rotationCenter;% the vector from rotation center to position
                d = abs(r-norm(l)); % Previously: d = norm(l)-r;
                
                start_dot_l = dot(startPointVector,l);% |startPointVetor|*|l|*sin(angle)
                start_cross_l = sign(radian)*(startPointVector(1)*l(2)-startPointVector(2)*l(1));% |startPointVetor|*|l|*cos(angle)
                % TODO: Check + - according to radian and
                % obj.vehicle.pathInfo.currentTrajectory values
                angle = -atan2(start_cross_l,start_dot_l);% the angle between startPointVector and vector l, tan(angle) = start_dot_l/start_cross_1
                if mod(angle,2*pi) > abs(radian)% judge if the radian of the angle bigger than the radian of the road
                    start_cross_l = -(startPointVector(1)*l(2)-startPointVector(2)*l(1));
                    angle = -atan2(start_cross_l,start_dot_l);
                end
                s = angle*r;
                routeLength = abs(radian)*r;
                
                yawAngle_in_Cartesian = angle+sign(radian)*pi/2;% the orientation of the current point of the road(phi 4 in Frenet.xml) in cartesian coordinate
                yawAngle_in_Cartesian = mod(yawAngle_in_Cartesian,2*pi);% orientation can not bigger than 2pi
                yawAngle_in_Cartesian = yawAngle_in_Cartesian.*(0<=yawAngle_in_Cartesian & yawAngle_in_Cartesian <= pi) + (yawAngle_in_Cartesian - 2*pi).*(pi<yawAngle_in_Cartesian & yawAngle_in_Cartesian<2*2*pi);   % angle in (-pi,pi]
            end
            
            
            [targetPosition_C,roadOrientation] = obj.Frenet2Cartesian(route,s,d,radian);%Coordinate Conversion function
            referencePose = [targetPosition_C(1); targetPosition_C(2); yawAngle_in_Cartesian*180/pi];%Required format for the Stanley controller
            
            
        end
        
        
        function referencePose = takeRoute(obj,car,refRoute)
            RotationVector = refRoute(3,:);
            
            if (RotationVector(1) == 0) %Straight motion
                %this function generates waypoints according to reference
                %trajectory. Waypoints are used as input signal for the Stanley
                %controller.
                route = car.pathInfo.currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];%Start- and endpoint of the current route
                position_Cart = car.dynamics.position([1,3]).*[1 -1];%Position of the vehicle in Cartesian coordinate
                radian = car.pathInfo.currentTrajectory(3,1);%radian of the curved road, is 0 for straight road
                [s,vehicle_d,orientation_C,routeLength] = obj.Cartesian2Frenet(route,position_Cart,radian);%Coordinate Conversion function
                car.pathInfo.s = s;% drived length
                car.pathInfo.routeEndDistance = routeLength-s; %distance to the current route's endpoint
                
                
                % ISSUE: Doesn't have meaning with LaneId-0.5
                %d=obj.laneWidth*(car.pathInfo.laneId-0.5)+obj.latOffset;
                d = obj.latOffset;
                latOffsetError = d-vehicle_d;%lateral offset error
                
                [targetPosition_C,roadOrientation] = obj.Frenet2Cartesian(route,s,latOffsetError+d,radian);%Coordinate Conversion function
                referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];%Required format for the Stanley controller
                
            else %Rotational motion
                
                %Determine rotation direction: left or right
                if car.pathInfo.currentTrajectory(4,:) == -ones(1,3) % -1 means turn left
                    %this function generates waypoints according to reference
                    %trajectory. Waypoints are used as input signal for the Stanley
                    %controller.
                    route = car.pathInfo.currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];%Start- and endpoint of the current route
                    position_Cart = car.dynamics.position([1,3]).*[1 -1];%Position of the vehicle in Cartesian coordinate
                    radian = abs(car.pathInfo.currentTrajectory(3,1));%radian of the curved road, is positive for left rotating curved road
                    %% transfer Cartesian coordinate into Frenet coordinate
                    [s,vehicle_d,orientation_C,routeLength] = obj.Cartesian2Frenet(route,position_Cart,radian); %Coordinate Conversion function
                    car.pathInfo.s = s;% Arc length
                    car.pathInfo.routeEndDistance = routeLength-s; %distance to the current route's endpoint
                    
                    % ISSUE: Doesn't have meaning with LaneId-0.5
                    d=-obj.latOffset;%negative is only for left rotating vehicle
                    
                    latOffsetError = d-vehicle_d;%lateral offset error
                    [targetPosition_C,roadOrientation] = obj.Frenet2Cartesian(route,s,latOffsetError+d,radian);%Coordinate Conversion function
                    referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];%Required format for the Stanley controller
                    
                elseif car.pathInfo.currentTrajectory(4,:) == ones(1,3) % 1 means turn right
                    %this function generates waypoints according to reference
                    %trajectory. Waypoints are used as input signal for the Stanley
                    %controller.
                    route = car.pathInfo.currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];%Start- and endpoint of the current route
                    position_Cart = car.dynamics.position([1,3]).*[1 -1];%Position of the vehicle in Cartesian coordinate
                    radian = -abs(car.pathInfo.currentTrajectory(3,1));%radian of the curved road, is negative for left rotating curved road
                    [s,vehicle_d,orientation_C,routeLength] = obj.Cartesian2Frenet(route,position_Cart,radian);%Coordinate Conversion function
                    car.pathInfo.s = s;% Arc length
                    car.pathInfo.routeEndDistance = routeLength-s;%distance to the current route's endpoint
                    
                    % ISSUE: Doesn't have meaning with LaneId-0.5
                    d=obj.latOffset;%for right rotating vehicle
                    
                    latOffsetError = d-vehicle_d;%lateral offset error
                    
                    [targetPosition_C,roadOrientation] = obj.Frenet2Cartesian(route,s,latOffsetError+d,radian);%Coordinate Conversion function
                    referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];%Required format for the Stanley controller
                    
                end
            end
            
        end
        

    end
    %% Standard Simulink Output functions
    methods(Static,Access = protected)
        
        
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
        
        function [out, out2] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 3];
            out2 = [1 3];
            
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
        
        function [out,out2] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = 'double';
            out2 = 'double';
            
            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end
        
        function [out, out2] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
        
        function [out, out2] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            
            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
        
        function sts = getSampleTimeImpl(obj)
            % Define sample time type and parameters
            sts = obj.createSampleTime("Type", "Inherited");
        end
    end
    
end
