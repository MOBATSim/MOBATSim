classdef PurePursuit_WPGenerator < WaypointGenerator
    % This blocks generates waypoints.
    %
    
    % Pre-computed constants
    properties(Access = private)
        latOffsetError = 0;  
        
        % Temp variable, later should be created in the function
        refLatSpeed =0;
        RouteOrientation = 0;
        
        Kpoints = 6;
        ref_d = 0;
        
        currentPathPoints =[];
        nextPathPoints=[];
        allPathPoints =[];
        calculatedRoutesArray = [];
        laneChangingPoints =[];
        
        laneChangeTime = 4;
        laneChangeStartPoint = [];
        laneChangeTargetPoint = [];
        laneChangeStartTime = [];
        
    end
    
    methods
        % Constructor
        function obj = PurePursuit_WPGenerator(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@WaypointGenerator(obj);  % Inherit the setupImpl function of the Superclass @WaypointGenerator
        end
        
        function nextWPs = stepImpl(obj,pose,speed,changeLane)
            
            obj.registerVehiclePoseAndSpeed(obj.vehicle,pose,speed); % Sets/Registers vehicle current Pose and speed
            
            %% This block shouldn't run if the ego vehicle: (destinationReached or Collided)
            if obj.vehicle.status.collided || obj.vehicle.pathInfo.destinationReached
                nextWPs = repmat([pose(1) pose(2)],obj.Kpoints,1); % Output: PathPoints for Pure Pursuit
                return;
            elseif obj.vehicle.pathInfo.routeCompleted % The Vehicle determines the initial trajectory and route
                obj.vehicle.setInitialRouteAndTrajectory();
            end
            
            %% Check and generate lane switching trajectory if commanded
            if ~(changeLane==0) && isempty(obj.trajPolynom)
                    obj.trajPolynom = obj.generateMinJerkTrajectory(obj.vehicle,obj.laneChangeTime,changeLane);
                    obj.laneChangingPoints = obj.generateLaneChanging_WPs(obj.vehicle);
            end

            obj.generatePathFollowingWaypoints(obj.vehicle)
            obj.vehicle.checkWaypointReached(obj.vehicle.pathInfo.currentTrajectory(2,:));
            
            if ~isempty(obj.laneChangingPoints)
                nextWPs =[obj.currentPathPoints(1,:); obj.laneChangingPoints; obj.currentPathPoints(2,:)];
                nextWPs = obj.checkNextWPsOutputSize(nextWPs,obj.Kpoints);
            else
                obj.currentPathPoints(2:end,2) = obj.currentPathPoints(2:end,2) + obj.ref_d;
                nextWPs = obj.currentPathPoints;  % Output: PathPoints for Pure Pursuit
            end
            
            
            
        end
        
        function nextWPs = checkNextWPsOutputSize(obj,nextWPs,K)
            % Prune reached WPs
            Vpos = [obj.vehicle.dynamics.position(1) -obj.vehicle.dynamics.position(3)];
            [~,idx] = min(vecnorm(Vpos-nextWPs,2,2));
            nextWPs = nextWPs(idx:end,:);
            
            if idx>size(obj.laneChangingPoints,1)
                obj.trajPolynom=[];%reset polynomial
                obj.laneChangingPoints =[];
                if (obj.ref_d-obj.vehicle.pathInfo.d) > 0 %left lane-changing
                    obj.vehicle.pathInfo.laneId = obj.vehicle.pathInfo.laneId+0.5;
                elseif (obj.ref_d-obj.vehicle.pathInfo.d) < 0%right lane-changing
                    obj.vehicle.pathInfo.laneId = obj.vehicle.pathInfo.laneId-0.5;
                end
            end
            
            
            
            % Adjust the nextWPs so that it fits the getOutputSizeMethod specifications
            if size(nextWPs,1) < K
                missingRowNr = K-size(nextWPs,1);
                nextWPs(end+1:end+missingRowNr,:) = repmat(nextWPs(end,:),missingRowNr,1);
                nextWPs(K-missingRowNr:K,2) = nextWPs(K-missingRowNr:K,2)+obj.ref_d; % Keep the lateral offset as Ego drives
            elseif size(nextWPs,1) > obj.Kpoints
                nextWPs = nextWPs(1:K,:);
            end
        end
        
        
        
        function trajPolynom=generateMinJerkTrajectory(obj,car,T,changeLane)
            obj.laneChangeStartTime = obj.getCurrentTime;
            car.dataLog.laneChangeStartTime = [car.dataLog.laneSwitchStartTime obj.laneChangeStartTime];%logging lane-switch start time
            obj.laneChangeStartPoint = car.dynamics.position.*[1 1 -1];%coordinates conversion
            
            %% Minimum Jerk Trajectory Generation
            x_f = T*car.dynamics.speed; % Target longitudinal - s coordinate
            
            if changeLane ==1 % To the left
                y_f = obj.laneWidth; % Target lateral - d coordinate
                target_d = obj.laneWidth;
                car.pathInfo.laneId = car.pathInfo.laneId + 0.5; % Means the vehicle is in between lanes - switching to left
            elseif changeLane ==2 % To the right
                y_f = 0; % Target lateral - d coordinate
                target_d = -car.pathInfo.d;
                car.pathInfo.laneId = car.pathInfo.laneId - 0.5; % Means the vehicle is in between lanes - switching to right
                % car.pathInfo.d is used to compansate for the small
                % values, TODO check later if this can be generalized for
                % more lanes than two
            end
            
            obj.laneChangeTargetPoint=[x_f 0 target_d]+obj.laneChangeStartPoint;
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
        
        function generatePathFollowingWaypoints(obj,car)
            %this function generates waypoints according to reference
            %trajectory. Waypoints are used as input signal for the Stanley
            %controller.
            route = car.pathInfo.currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];%Start- and endpoint of the current route
            position_Cart = car.dynamics.position([1,3]).*[1 -1];%Position of the vehicle in Cartesian coordinate
            radian = car.pathInfo.currentTrajectory(3,1);%radian of the curved road, is 0 for straight road
            
            [s,vehicle_d,orientation_C,routeLength] = obj.Cartesian2Frenet(route,position_Cart,radian);%Coordinate Conversion function
            
            
            car.updateVehicleFrenetPosition(s,vehicle_d,routeLength); % Update Vehicle Frenet Coordinates
            
        end
        
        function generateStraightWaypoints(obj,car)
            
        end
        
        function generateLeftRotationWaypoints(obj,car)

            
        end
        
        function generateRightRotationWaypoints(obj,car)
            
        end
        
        function newWP_all = generateLaneChanging_WPs(obj, car)
            t=obj.getCurrentTime-obj.laneChangeStartTime;
            
            % Unfortunately there is no better way than this, double2cell -> deal is slower
            a0=obj.trajPolynom(1);% a0
            a1=obj.trajPolynom(2);% a1
            a2=obj.trajPolynom(3);% a2
            a3=obj.trajPolynom(4);% a3
            a4=obj.trajPolynom(5);% a4
            a5=obj.trajPolynom(6);% a5
            
            % Target lateral - p point
            T = obj.laneChangeTime;
            y_f = a0+a1*T+a2*T^2+a3*T^3+a4*T^4+a5*T^5;
            
            tP = t:0.2:T;
            newWP_s=car.dynamics.position(1)+(car.dynamics.speed*tP);
            newWP_d=-car.dynamics.position(3)+(a0+a1*tP+a2*tP.^2+a3*tP.^3+a4*tP.^4+a5*tP.^5)-car.pathInfo.d;
            newWP_all = [newWP_s' newWP_d'];
            
            obj.ref_d = a0+a1*T+a2*T^2+a3*T^3+a4*T^4+a5*T^5;
            
        end
        
        function currentPathPoints = findTheNextKPoints(~,Vpos,nextPoints,K)
            K = K - 1;
            [~,idx] = min(vecnorm(Vpos-nextPoints,2,2));
            if idx+K > size(nextPoints,1)
                currentPathPoints = nextPoints(idx:end,:);
                lastPathPoints = repmat(nextPoints(end,:),K-(size(nextPoints,1)-idx),1);
                currentPathPoints =  [currentPathPoints; lastPathPoints];
            else
                currentPathPoints = nextPoints(idx:idx+K,:);
            end
            
        end
        
        
        %% Override for experiment - Later incorparate into WaypointGenerator.m
        
        function [position_Cart,orientation_Cart] = Frenet2Cartesian(~,route,s,d,radian)
            % Transform a position from Frenet coordinate to Cartesian coordinate
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
        
        function generate_straight_PathPoints(obj,route)
            Route_StartPoint = route(1,:);
            Route_endPoint = route(2,:);
            k = 50;
            deltaX = (Route_endPoint(1)-Route_StartPoint(1))/k;
            deltaY = (Route_endPoint(2)-Route_StartPoint(2))/k;
            
            if deltaX == 0
                % Vertical Road
                obj.nextPathPoints =[repmat(route(1,1),k+1,1) (route(1,2):deltaY:route(2,2))'];
            elseif deltaY == 0
                % Horizontal Road
                obj.nextPathPoints = [(route(1,1):deltaX:route(2,1))' repmat(route(1,2),k+1,1)];
            end
            
            obj.allPathPoints = [obj.allPathPoints; obj.nextPathPoints];
            obj.calculatedRoutesArray(end+1) = obj.vehicle.pathInfo.currentRoute;
            obj.currentPathPoints = obj.nextPathPoints(1:5,:);
            
        end
        
        function [s,d,yawAngle_in_Cartesian,routeLength] = Cartesian2Frenet(obj,route,vehiclePos_Cartesian,radian)
            %Transform a position in Cartesian coordinate into Frenet coordinate
            
            %Function Inputs:
            %route:                 2x2 array [x_s y_s;x_e y_e] the starting point and the ending point of the road
            %vehiclePos_Cartesian:  1x2 array [x y] in Cartesian coordinate
            %radian:                The angle of the whole curved road, positive for counterclockwise turn
            
            %Function Output:
            %yawAngle_in_Cartesian: The angle of the tangent vector on the reference roadline(d=0)
            %s:                     Traversed length along the reference roadline
            %d:                     Vertical offset distance to the reference roadline,positive d means away from center
            
            Route_StartPoint = route(1,:);
            Route_endPoint = route(2,:);
            
            
            if radian == 0%straight road
                obj.curvature = 0;
                
                route_Vector = Route_endPoint-Route_StartPoint;
                route_UnitVector = route_Vector/norm(route_Vector);
                yawAngle_in_Cartesian = atan2(route_UnitVector(2),route_UnitVector(1));% orientation angle of the vehicle in Cartesian Coordinate
                posVector = vehiclePos_Cartesian-Route_StartPoint;
                
                sideVector = [cos(yawAngle_in_Cartesian+pi/2) sin(yawAngle_in_Cartesian+pi/2)];% side vector is perpendicular to the route
                sideVector = round(sideVector,5);
                
                obj.currentPathPoints = obj.findTheNextKPoints(vehiclePos_Cartesian,obj.vehicle.pathInfo.BOGPath,obj.Kpoints);
                %obj.generate_straight_PathPoints(route);
                
                
                s = dot(posVector,route_UnitVector);% the projection of posVector on route_UnitVector
                
                d = dot(posVector,sideVector);% the projection of posVector on sideVector
                routeLength = norm(Route_endPoint-Route_StartPoint);% the length of the route_Vector
                
            else % Curved Road
                
                rotationCenter = obj.vehicle.pathInfo.currentTrajectory(3,[2 3]); % Get the rotation center
                rotationCenter(2) = -rotationCenter(2); % Transform the coordinate
                r = norm(Route_StartPoint-rotationCenter); % Get the radius of the rotation
                startPointVector = Route_StartPoint-rotationCenter;% vector OP_1 in Frenet.xml
                
                obj.currentPathPoints = obj.findTheNextKPoints(vehiclePos_Cartesian,obj.vehicle.pathInfo.BOGPath,obj.Kpoints);
                
                
                l = vehiclePos_Cartesian-rotationCenter;% the vector from rotation center to position
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
             
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon("WaypointGenerator.png");
        end
        
    end
    %% Standard Simulink Output functions
    methods(Static,Access = protected)
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
        
        function out = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [obj.Kpoints 2];
            
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
        
        function out = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = 'double';
            
            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end
        
        function out = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
        
        function out = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            
            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
        
        function sts = getSampleTimeImpl(obj)
            % Define sample time type and parameters
            sts = obj.createSampleTime("Type", "Inherited");
        end
    end
    
end
