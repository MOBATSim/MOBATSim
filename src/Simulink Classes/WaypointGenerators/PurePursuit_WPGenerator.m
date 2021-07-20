classdef PurePursuit_WPGenerator < WaypointGenerator
    % This blocks generates waypoints for the Pure Pursuit lateral controller.
    %
    
    % Pre-computed constants
    properties(Access = private)
        
        Kpoints = 6; % The number of next path points to be output to the Pure Pursuit controller
        ref_d = 0; % The reference lateral coordinate "d" for tracking the right or the left lane
        
        currentPathPoints =[];
        laneChangingPoints =[];
        
        laneChangeTime = 4;
        laneChangeStartTime = [];
        offset_d_flag = 0;
        
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
            
            % Sets/Registers vehicle current Pose and speed
            obj.registerVehiclePoseAndSpeed(obj.vehicle,pose,speed); 
            
            %% This block shouldn't run if the ego vehicle: (destinationReached or Collided)
            if obj.vehicle.status.collided || obj.vehicle.pathInfo.destinationReached
                nextWPs = repmat([pose(1) pose(2)],obj.Kpoints,1); % Output: PathPoints for Pure Pursuit
                return;
            elseif obj.vehicle.pathInfo.routeCompleted % The Vehicle determines the initial trajectory and route
                obj.vehicle.setInitialRouteAndTrajectory();
            end
            %%
            
            % Calculate helping variables for the reference path calculation
            route = obj.vehicle.pathInfo.currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];%Start- and endpoint of the current route
            Vpos_C = [pose(1) pose(2)];%Position of the vehicle in Cartesian coordinate
            radian = obj.vehicle.pathInfo.currentTrajectory(3,1);%radian of the curved road, is 0 for straight road
            
            % Generate reference path following waypoints in Cartesian
            obj.currentPathPoints = obj.generatePathFollowingWaypoints(Vpos_C,obj.vehicle.pathInfo.BOGPath,obj.Kpoints);
            
            % Convert from Cartesian to Frenet
            [s,d,routeLength] = obj.Cartesian2Frenet(route,Vpos_C,radian);
            obj.vehicle.updateVehicleFrenetPosition(s,d,routeLength); % Update Vehicle Frenet Coordinates
            
            % Check if Waypoint is Reached
            obj.vehicle.checkWaypointReached(obj.vehicle.pathInfo.currentTrajectory(2,:));          
            
            % Check and generate lane changing trajectory if commanded
            if ~(changeLane==0) && isempty(obj.laneChangingPoints)
                    Frenet_LaneChangingPoints = obj.generateMinJerkTrajectory(obj.vehicle,obj.laneChangeTime,changeLane);
                    obj.laneChangingPoints = obj.Frenet2Cartesian(s,d,Frenet_LaneChangingPoints,obj.vehicle.pathInfo.currentTrajectory);
            end
            
            % If there are lane changing path points            
            if ~isempty(obj.laneChangingPoints)
                    nextWPs =[obj.currentPathPoints(1,:); obj.laneChangingPoints; obj.currentPathPoints(2,:)];
                    nextWPs = obj.updateLaneChangingNextWPs(nextWPs);
                    nextWPs = obj.checkNextWPsOutputSize(nextWPs,obj.Kpoints); % Output: PathPoints for Pure Pursuit
            else
                % If there are no lane changing path points
                if obj.offset_d_flag % If the vehicle is on the left lane +d must be added
                    s_next = (s:4:obj.vehicle.pathInfo.routeEndDistance+s)';
                    d_next=repmat(obj.ref_d,length(s_next),1);
                    
                    nextWPs = obj.array_Frenet2Cartesian(route,s_next,d_next,radian);
                    nextWPs = obj.checkNextWPsOutputSize(nextWPs,obj.Kpoints);

                else % If the vehicle is on the right lane, it follows the generated path points
                    nextWPs = obj.currentPathPoints;  % Output: PathPoints for Pure Pursuit
                end
            end
            
            
            
        end
        
        function updatedLaneChangingPoints_Cartesian = Frenet2Cartesian(obj,s,d,laneChangingPoints,currentTrajectory)
            route = currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];
            radian = currentTrajectory(3,1);
            left = currentTrajectory(4,1);
            
            if radian == 0
                route_Vector = route(2,:)-route(1,:);
                route_UnitVector = route_Vector/norm(route_Vector);
                yawAngle_in_Cartesian = atan2(route_UnitVector(2),route_UnitVector(1));
                sideVector = [cos(yawAngle_in_Cartesian+pi/2) sin(yawAngle_in_Cartesian+pi/2)];
                sideVector = round(sideVector,5);
                
                % Lane Changing Points were already in Frenet - only "s" value should be added
                % "d" is already the reference
                updatedLaneChangingPoints_Frenet=[s+laneChangingPoints(:,1) laneChangingPoints(:,2)];
                
                updatedLaneChangingPoints_Cartesian = updatedLaneChangingPoints_Frenet(:,1)*route_UnitVector+updatedLaneChangingPoints_Frenet(:,2)*sideVector+route(1,:);
            else
                startPoint = route(1,:);
                endPoint = route(2,:);
                r = sqrt((norm(endPoint-startPoint))^2/(1-cos(radian))/2);%The radius of the road segment， according to the law of the cosines
                center = [currentTrajectory(3,2) -currentTrajectory(3,3)];
                startPointVector = startPoint-center;%OP1 in Frenet.xml
                startPointVectorAng = atan2(startPointVector(2),startPointVector(1));
                l = r+(left*d);%current distance from rotation center to position
                lAng = sign(radian)*s/r+startPointVectorAng;% the angle of vector l
                position_Cart = l*[cos(lAng) sin(lAng)]+center;% the position in Cartesion coordinate
                orientation_Cart = lAng+sign(radian)*pi/2;
                orientation_Cart = mod(orientation_Cart,2*pi);
                orientation_Cart = orientation_Cart.*(0<=orientation_Cart & orientation_Cart <= pi) + (orientation_Cart - 2*pi).*(pi<orientation_Cart & orientation_Cart<2*2*pi);   % angle in (-pi,pi]
                
                updatedLaneChangingPoints_Frenet=[s+laneChangingPoints(:,1) laneChangingPoints(:,2)];
                %frePoints = [(updatedLaneChangingPoints_Frenet(:,1)/r) updatedLaneChangingPoints_Frenet(:,2)];
                all_s = updatedLaneChangingPoints_Frenet(:,1);
                all_d = updatedLaneChangingPoints_Frenet(:,2);
                updatedLaneChangingPoints_Cartesian = obj.array_Frenet2Cartesian(route,all_s,all_d,radian);
            end
            
            

        end
        
        function nextWPs = checkNextWPsOutputSize(obj,nextWPs,K)
            
            % Adjust the nextWPs so that it fits the getOutputSizeMethod specifications
            if size(nextWPs,1) < K
                missingRowNr = K-size(nextWPs,1);
                nextWPs(end+1:end+missingRowNr,:) = repmat(nextWPs(end,:),missingRowNr,1);
            elseif size(nextWPs,1) > obj.Kpoints
                nextWPs = nextWPs(1:K,:);
            end
        end
        
        function nextWPs = updateLaneChangingNextWPs(obj,nextWPs)
            % Prune reached WPs
            Vpos = [obj.vehicle.dynamics.position(1) -obj.vehicle.dynamics.position(3)];
            [~,idx] = min(vecnorm(Vpos-nextWPs,2,2));
            nextWPs = nextWPs(idx:end,:);
            
            % If the lane-changing is almost done, reset the trajPolynom so track the reference trajectory with +d
            if idx>size(obj.laneChangingPoints,1) 
                obj.trajPolynom=[];%reset polynomial
                obj.laneChangingPoints =[];
                if (obj.ref_d-obj.vehicle.pathInfo.d) > 0 %left lane-changing
                    obj.vehicle.pathInfo.laneId = obj.vehicle.pathInfo.laneId+0.5;
                    obj.offset_d_flag = 1;
                elseif (obj.ref_d-obj.vehicle.pathInfo.d) < 0%right lane-changing
                    obj.vehicle.pathInfo.laneId = obj.vehicle.pathInfo.laneId-0.5;
                    obj.offset_d_flag = 0;
                end
            end
        end
        
                        
        function newWP_all=generateMinJerkTrajectory(obj,car,tf,changeLane)
            obj.laneChangeStartTime = obj.getCurrentTime;
            car.dataLog.laneChangeStartTime = [car.dataLog.laneSwitchStartTime obj.laneChangeStartTime];%logging lane-switch start time
            
            %% Minimum Jerk Trajectory Generation            
            if changeLane ==1 % To the left
                y_f = obj.laneWidth; % Target lateral - d coordinate
                car.pathInfo.laneId = car.pathInfo.laneId + 0.5; % Means the vehicle is in between lanes - switching to left
            elseif changeLane ==2 % To the right
                y_f = 0; % Target lateral - d coordinate
                car.pathInfo.laneId = car.pathInfo.laneId - 0.5; % Means the vehicle is in between lanes - switching to right
            end
                        
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
            
            obj.trajPolynom = [a0 a1 a2 a3 a4 a5];
            
            tP = 0:0.2:tf;
            newWP_s=car.dynamics.speed*tP; % Target longitudinal - s coordinate
            newWP_d=a0+a1*tP+a2*tP.^2+a3*tP.^3+a4*tP.^4+a5*tP.^5;
            newWP_all =  [newWP_s' newWP_d'];
            obj.ref_d = a0+a1*tf+a2*tf^2+a3*tf^3+a4*tf^4+a5*tf^5;
        end
        
        function currentPathPoints = generatePathFollowingWaypoints(~,Vpos,nextPoints,K)
            % This function determines the next closest path points along the reference trajectory.
            
            K = K - 1; % idx is the first point so the next K-1 points should be determined
            
            % Find the index of the closest point to calculate the forward points
            [~,idx] = min(vecnorm(Vpos-nextPoints,2,2));
            
            if idx+K > size(nextPoints,1) % If the vehicle is coming to the end of the reference trajectory
                currentPathPoints = nextPoints(idx:end,:);
                lastPathPoints = repmat(nextPoints(end,:),K-(size(nextPoints,1)-idx),1); % Destination point is duplicated
                currentPathPoints =  [currentPathPoints; lastPathPoints]; % to fit the output size of [obj.Kpoints 2]
            else
                currentPathPoints = nextPoints(idx:idx+K,:); % Next points from the BOG-Path are used for the reference trajectory
            end
        end
  
        %% Override for experiment - Later incorparate into WaypointGenerator.m
          
        function [s,d,routeLength] = Cartesian2Frenet(obj,route,vehiclePos_Cartesian,radian)
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
                route_Vector = Route_endPoint-Route_StartPoint;
                route_UnitVector = route_Vector/norm(route_Vector);
                yawAngle_in_Cartesian = atan2(route_UnitVector(2),route_UnitVector(1));% orientation angle of the vehicle in Cartesian Coordinate
                posVector = vehiclePos_Cartesian-Route_StartPoint;
                
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
                                
                
                l = vehiclePos_Cartesian-rotationCenter;% the vector from rotation center to position
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
            end
        end
        
        function position_Cart = array_Frenet2Cartesian(~,route,s,d,radian)
            startPoint = route(1,:);
            endPoint = route(2,:);
            
            if radian ==0
                route_Vector = endPoint-startPoint;
                route_UnitVector = route_Vector/norm(route_Vector);
                yawAngle_in_Cartesian = atan2(route_UnitVector(2),route_UnitVector(1));% orientation angle of the vehicle in Cartesian Coordinate               
                sideVector = [cos(yawAngle_in_Cartesian+pi/2) sin(yawAngle_in_Cartesian+pi/2)];%vector of the tangent line of reference line
                sideVector = round(sideVector,5);
                position_Cart = s*route_UnitVector+d*sideVector+startPoint;% position= start point + length of journey
            else

                r = sqrt((norm(endPoint-startPoint))^2/(1-cos(radian))/2);%The radius of the road segment， according to the law of the cosines
                targetVector = (endPoint-startPoint)/norm(endPoint-startPoint); %Unit vector of route vector (p in Frenet.xml)
                beta = atan2(targetVector(2),targetVector(1)); %the angle of target vector and x axis in cartesian coordinate (theta 1 in Frenet.xml)
                plumbLength = cos(radian/2)*r; % the distance from circle center to targetVector (OG in Frenet.xml)
                plumbVector = [cos(beta+sign(radian)*pi/2) sin(beta+sign(radian)*pi/2)]*plumbLength;
                center = startPoint + targetVector*norm(endPoint-startPoint)/2 + plumbVector;%rotation center of the road in Cartesian coordinate
                startPointVector = startPoint-center;%OP1 in Frenet.xml
                startPointVectorAng = atan2(startPointVector(2),startPointVector(1));
                l = r-(sign(radian)*d);%current distance from rotation center to position
                lAng = sign(radian)*s/r+startPointVectorAng;% the angle of vector l
                position_Cart = l.*[cos(lAng) sin(lAng)]+center;% the position in Cartesion coordinate
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
