classdef New_WPGenerator_Stanley < WaypointGenerator
    % This blocks generates waypoints.
    %
    
    % Pre-computed constants
    properties(Access = private)     
        latOffsetError = 0;
        changeLane = 0;
        
        % Temp variable, later should be created in the function
        refLatSpeed =0;
        RouteOrientation = 0;
    end
    
    methods
        % Constructor
        function obj = New_WPGenerator_Stanley(varargin)
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

 
        
        function [ReferenceYaw, referencePose, refLatSpeed] = stepImpl(obj,pose,speed,changeLane)
            % Lane changing signal
            obj.changeLane = changeLane;
                        
            %transfer from local coordinate obj.vehicle.dynamics.speed = v_pos(4);
            
            
            obj.vehicle.setPosition(obj.map.transformPoseTo3DAnim(pose));   % Sets the vehicle position
            obj.vehicle.setYawAngle(pose(3));                               % Sets the vehicle yaw angle (4th column of orientation)
            
            pose(3)=pose(3)*180/pi; % rad to deg
            
            %This block shouldn't run if the ego vehicle: (destinationReached or Collided)
            if obj.vehicle.status.collided || obj.vehicle.pathInfo.destinationReached
                ReferenceYaw= obj.RouteOrientation + atan2(obj.refLatSpeed,speed);  % Output1: vehicle's reference steering en             
                referencePose = obj.referencePose';         % Output2: Reference pose
                refLatSpeed = 0;                            % Output3: Reference Lateral Speed
                return;
            end
            
            
            if ~obj.vehicle.pathInfo.destinationReached
                % The Vehicle hasn't reached its destination yet
                obj.vehicle.updateActualSpeed(speed); % Vehicle - Set Functions

                if obj.vehicle.pathInfo.routeCompleted
                    % The Vehicle has completed its Route
                    nextRoute = obj.generateCurrentRoute(obj.vehicle,obj.vehicle.pathInfo.path,obj.vehicle.pathInfo.lastWaypoint);
                    currentTrajectory = obj.generateTrajectoryFromPath(obj.vehicle,obj.vehicle.pathInfo.path);
                    
                    obj.vehicle.setCurrentRoute(nextRoute);              % Vehicle - Set Functions
                    obj.vehicle.setCurrentTrajectory(currentTrajectory); % Vehicle - Set Functions
                    obj.vehicle.setRouteCompleted(false);                % Vehicle - Set Functions
                    
                end
                
                speedAccordingtoSimulation = speed*0.01*obj.simSpeed;%speed limit on curved road
                                
                obj.takeRoute(obj.vehicle,speedAccordingtoSimulation,obj.vehicle.pathInfo.currentTrajectory);
            
            end
            
            ReferenceYaw=obj.RouteOrientation +atan2(obj.refLatSpeed,speed);  % Output1: Reference pose
            referencePose = obj.referencePose';         % Output2: vehicle's actual pose
            refLatSpeed=obj.refLatSpeed;                % Output3: Reference Lateral Speed
        end
        

        function takeRoute(obj,car,speed,refRoute)
            RotationVector = refRoute(3,:);
            
            %% Generate lane switching trajectory if commanded
            if ~(obj.changeLane==0)
                if isempty(obj.trajPolynom)
                    obj.checkLaneSwitch(car);
                end
            end
            
            %% Generate forward waypoints
            if (RotationVector(1) == 0) %Straight motion
                obj.move_straight(car,speed,refRoute(2,:));
                
            else %Rotational motion
                P_final = refRoute(2,:);
                
                %Determine rotation direction: left or right
                if car.pathInfo.currentTrajectory(4,:) == -ones(1,3) % -1 means turn left
                    obj.rotate_left(car,speed,P_final);
                    
                elseif car.pathInfo.currentTrajectory(4,:) == ones(1,3) % 1 means turn right
                    obj.rotate_right(car,speed,P_final);
                end
            end
            
        end
        
        
        function checkLaneSwitch(obj,car)                                                         
                    %obj.laneSwitchTime % Delta T for lane changing = defined in the superclass
                    obj.trajPolynom = obj.generateMinJerkTrajectory(car,obj.laneSwitchTime); 
        end
        
        
        function trajPolynom=generateMinJerkTrajectory(obj,car,T)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];%logging lane-switch start time
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];%coordinates conversion
            
            %% Minimum Jerk Trajectory Generation
            x_f = T*car.dynamics.speed; % Target longitudinal - s coordinate
            
            if obj.changeLane ==1 % To the left
                y_f = car.pathInfo.d+obj.laneWidth; % Target lateral - d coordinate
                car.pathInfo.laneId = car.pathInfo.laneId + 0.5; % Means the vehicle is in between lanes - switching to left
            elseif obj.changeLane ==2 % To the right
                y_f = car.pathInfo.d-obj.laneWidth; % Target lateral - d coordinate
                car.pathInfo.laneId = car.pathInfo.laneId - 0.5; % Means the vehicle is in between lanes - switching to right
                % car.pathInfo.d is used to compansate for the small
                % values, TODO check later if this can be generalized for
                % more lanes than two
            end           

            obj.laneSwitchTargetPoint=[x_f 0 y_f]+obj.laneSwitchStartPoint;
            %%  Minimun jerk trajectory function for the calculation in y direction (Lateral)
            syms t; % time
            % matrix with polynom coefficients for  lateral position, speed and acceleration
            %         a0   a1    a2     a3      a4       a5
            d(t) = [  1     t   t^2    t^3     t^4     t^5;
                      0     1   2*t  3*t^2   4*t^3   5*t^4;
                      0     0     2    6*t  12*t^2  20*t^3];
            % Initial boundary conditions
            ti = 0; % time
            d_ti = [car.pathInfo.d; 0; 0]; % Initial position, speed, acceleration
            
            % Final boundary conditions
            tf = T;
            d_tf = [y_f; 0; 0]; % Final position, speed, acceleration        
            % Solve all linear equations with conditions at t = ti and t = tf
            A = linsolve([d(ti);d(tf)], [d_ti; d_tf]);
            A = double(A);
            a0=A(1);
            a1=A(2);
            a2=A(3);
            a3=A(4);
            a4=A(5);
            a5=A(6);

            trajPolynom = [a0 a1 a2 a3 a4 a5];
        end

        

        function generateStraightWaypoints(obj,car)
            %this function generates waypoints according to reference
            %trajectory. Waypoints are used as input signal for the Stanley
            %controller.
            route = car.pathInfo.currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];%Start- and endpoint of the current route
            position_Cart = car.dynamics.position([1,3]).*[1 -1];%Position of the vehicle in Cartesian coordinate
            radian = car.pathInfo.currentTrajectory(3,1);%radian of the curved road, is 0 for straight road
            [s,vehicle_d,orientation_C,routeLength] = obj.Cartesian2Frenet(route,position_Cart,radian);%Coordinate Conversion function
            
            car.updateVehicleFrenetPosition(s,vehicle_d,routeLength); % Update Vehicle Frenet Coordinates       
            %% If lane-changing trajectory exists
            if(~isempty(obj.trajPolynom))% if lane-changing trajectory exists
                obj.generateLaneChanging_WPs(car)
            else
                obj.refLatSpeed =0;
            end
            
            % ISSUE: Doesn't have meaning with LaneId-0.5 
            %d=obj.laneWidth*(car.pathInfo.laneId-0.5)+obj.latOffset;
            d = obj.latOffset;
            obj.latOffsetError = d-vehicle_d;%lateral offset error
            
            [targetPosition_C,roadOrientation] = obj.Frenet2Cartesian(route,s,obj.latOffsetError+d,radian);%Coordinate Conversion function
            obj.referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];%Required format for the Stanley controller
            % TRY
            obj.RouteOrientation = roadOrientation;
        end
        
        function generateLeftRotationWaypoints(obj,car)
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

            car.updateVehicleFrenetPosition(s,vehicle_d,routeLength); % Update Vehicle Frenet Coordinates
            %% If lane-changing trajectory exists
            if(~isempty(obj.trajPolynom))% if lane-changing trajectory exists
                obj.generateLaneChanging_WPs(car)
            else
                obj.refLatSpeed =0;
            end

            % ISSUE: Doesn't have meaning with LaneId-0.5 
            d=-obj.latOffset;%negative is only for left rotating vehicle -> Check if minus is right
            
            obj.latOffsetError = d-vehicle_d;%lateral offset error
            [targetPosition_C,roadOrientation] = obj.Frenet2Cartesian(route,s,obj.latOffsetError+d,radian);%Coordinate Conversion function
            obj.referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];%Required format for the Stanley controller
            % TRY
            obj.RouteOrientation = roadOrientation;
        end
        
        function generateRightRotationWaypoints(obj,car)
            %this function generates waypoints according to reference
            %trajectory. Waypoints are used as input signal for the Stanley
            %controller.
            route = car.pathInfo.currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];%Start- and endpoint of the current route
            position_Cart = car.dynamics.position([1,3]).*[1 -1];%Position of the vehicle in Cartesian coordinate
            radian = -abs(car.pathInfo.currentTrajectory(3,1));%radian of the curved road, is negative for left rotating curved road
            [s,vehicle_d,orientation_C,routeLength] = obj.Cartesian2Frenet(route,position_Cart,radian);%Coordinate Conversion function
            car.pathInfo.s = s;% Arc length
            car.pathInfo.routeEndDistance = routeLength-s;%distance to the current route's endpoint
            
            car.updateVehicleFrenetPosition(s,vehicle_d,routeLength); % Update Vehicle Frenet Coordinates
            %% If lane-changing trajectory exists
            if(~isempty(obj.trajPolynom))% if lane-changing trajectory exists
                obj.generateLaneChanging_WPs(car)
            else
                obj.refLatSpeed =0;
            end

            % ISSUE: Doesn't have meaning with LaneId-0.5 
            d=obj.latOffset;%for right rotating vehicle
            
            obj.latOffsetError = d-vehicle_d;%lateral offset error
            [targetPosition_C,roadOrientation] = obj.Frenet2Cartesian(route,s,obj.latOffsetError+d,radian);%Coordinate Conversion function
            obj.referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];%Required format for the Stanley controller
            % TRY
            obj.RouteOrientation = roadOrientation;
        end

        function generateLaneChanging_WPs(obj, car) 
            t=get_param('MOBATSim','SimulationTime')-obj.laneSwitchStartTime;
            
            % Unfortunately there is no better way than this, double2cell -> deal is slower
            a0=obj.trajPolynom(1);% a0
            a1=obj.trajPolynom(2);% a1
            a2=obj.trajPolynom(3);% a2
            a3=obj.trajPolynom(4);% a3
            a4=obj.trajPolynom(5);% a4
            a5=obj.trajPolynom(6);% a5
            
            % Target lateral - p point
            T = obj.laneSwitchTime;
            y_f = a0+a1*T+a2*T^2+a3*T^3+a4*T^4+a5*T^5;
            
            %% TODO: Conditions shouldn't depend solely on time
            if abs(y_f-car.pathInfo.d) > 0.05 %lane-changing is not finished
                obj.latOffset = a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5;                    % reference delta_d
                obj.refLatSpeed= a1  + 2*a2*t +  3*a3*t.^2  +  4*a4*t.^3  +  5*a5*t.^4; % reference delta_d_dot
                
            else%lane-changing done
                obj.refLatSpeed = 0;
                obj.latOffset = car.pathInfo.d;%reset reference delta_d
                car.status.laneSwitchFinish = 1;%lane-changing done flag
                if (y_f-car.pathInfo.d) > 0 %left lane-changing
                    car.pathInfo.laneId = car.pathInfo.laneId+0.5;
                elseif (y_f-car.pathInfo.d) < 0%right lane-changing
                    car.pathInfo.laneId = car.pathInfo.laneId-0.5;
                end
                %car.status.canLaneSwitch = 0;%reset flag
                obj.trajPolynom=[];%reset polynomial
            end
            
        end
        
        %% Override for experiment - Later incorparate into WaypointGenerator.m
              
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
                d = abs(r-norm(l)); % Previously: d = norm(l)-r;
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
    %% Standard Simulink Output functions
    methods(Static,Access = protected)

        
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
        
        function [out, out2, out3] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 1];
            out2 = [1 3];
            out3 = [1 1];
            
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
        
        function [out,out2, out3] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = 'double';
            out2 = 'double';
            out3 = 'double';

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end
        
        function [out, out2, out3] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
            
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
        
        function [out, out2, out3] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
        
        function sts = getSampleTimeImpl(obj)
            % Define sample time type and parameters
            sts = obj.createSampleTime("Type", "Inherited");
        end
    end
    
end
