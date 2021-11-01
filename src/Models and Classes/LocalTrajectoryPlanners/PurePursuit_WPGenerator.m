classdef PurePursuit_WPGenerator < LocalTrajectoryPlanner
    % This blocks generates waypoints for the Pure Pursuit lateral controller.
    %
    
    % Pre-computed constants
    properties(Access = private)
        
        Kpoints = 6; % The number of next path points to be output to the Pure Pursuit controller
        ref_d = 0; % The reference lateral coordinate "d" for tracking the right or the left lane
        
        currentPathPoints =[];
        laneChangingPoints =[];
        
        laneChangeTime = 4;
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
            setupImpl@LocalTrajectoryPlanner(obj);  % Inherit the setupImpl function of the Superclass @WaypointGenerator
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
            currentTrajectory = obj.vehicle.pathInfo.currentTrajectory;
            Vpos_C = [pose(1) pose(2)];%Position of the vehicle in Cartesian coordinate
            
            % Determine Vehicle's Position in Frenet Coordinates
            [s,d] = obj.Cartesian2Frenet(currentTrajectory,Vpos_C); % Determine current <s,d>
            
            % Update Vehicle Frenet Coordinates <s,d>
            obj.vehicle.updateVehicleFrenetPosition(s,d);
            
            % Generate reference path following waypoints in Cartesian
            obj.currentPathPoints = obj.generatePathFollowingWaypoints(Vpos_C,obj.vehicle.pathInfo.BOGPath,obj.Kpoints);
            
            % Check if the Waypoint is Reached
            obj.vehicle.checkWaypointReached(currentTrajectory(2,:));      
            
            % Generate lane changing trajectory if commanded and not generated yet
            if ~(changeLane==0) && isempty(obj.laneChangingPoints)
                    Frenet_LaneChangingPoints = obj.generateMinJerkTrajectory(obj.vehicle,obj.laneChangeTime,changeLane);
                    obj.laneChangingPoints = obj.Frenet2Cartesian(s,Frenet_LaneChangingPoints,currentTrajectory);
            end
            
            % If there are already lane changing path points            
            if ~isempty(obj.laneChangingPoints)
                    nextWPs =[obj.currentPathPoints(1,:); obj.laneChangingPoints];
                    nextWPs = obj.updateLaneChangingNextWPs(nextWPs);
                    nextWPs = obj.checkNextWPsOutputSize(nextWPs,obj.Kpoints); % Output: PathPoints for Pure Pursuit
            else
                % If there are no lane changing path points
                if (obj.vehicle.pathInfo.laneId == 1) % If the vehicle is on the left lane +d must be added
                    s_next = (s:4:obj.vehicle.pathInfo.routeEndDistance+s)';
                    d_next=repmat(obj.ref_d,length(s_next),1);
                    % s_next already starts from "s" therefore "0" is sent as the current "s" to the function below
                    nextWPs = obj.Frenet2Cartesian(0,[s_next,d_next],currentTrajectory);
                    nextWPs = obj.checkNextWPsOutputSize(nextWPs,obj.Kpoints);

                else % If the vehicle is on the right lane, it follows the generated path points
                    nextWPs = obj.currentPathPoints;  % Output: PathPoints for Pure Pursuit
                end
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
            
            % If the lane-changing is almost done, reset the laneChangingPoints and only track the reference trajectory with +d
            if idx>size(obj.laneChangingPoints,1) 
                obj.laneChangingPoints =[];
                if abs(obj.ref_d-obj.vehicle.pathInfo.d) < 0.05 % lane-changing completed                    
                    if obj.ref_d > 0
                        obj.vehicle.pathInfo.laneId = obj.vehicle.pathInfo.laneId+0.5; %left lane-changing completed
                    else
                        obj.vehicle.pathInfo.laneId = obj.vehicle.pathInfo.laneId-0.5; %right lane-changing completed
                    end
                    
                    
                end
            end
        end
                                
        function newWP_all=generateMinJerkTrajectory(obj,car,t_f,changeLane)
            % Minimum Jerk Trajectory Generation            
            if changeLane ==1 % To the left
                y_f = obj.laneWidth; % Target lateral - d coordinate
                car.pathInfo.laneId = car.pathInfo.laneId + 0.5; % Means the vehicle is in between lanes - switching to left
            elseif changeLane ==2 % To the right
                y_f = 0; % Target lateral - d coordinate
                car.pathInfo.laneId = car.pathInfo.laneId - 0.5; % Means the vehicle is in between lanes - switching to right
            end
                        
            % Minimun jerk trajectory function for the calculation in "d" direction (Frenet Lateral)
            % Determining the polynomial coefficients for  lateral position, speed and acceleration

            % Initial boundary conditions
            t_i = 0; % initial time always set to zero (relatively makes no difference)
            d_i =   [  1     t_i   t_i^2    t_i^3     t_i^4     t_i^5];
            d_dot_i = [0     1   2*t_i  3*t_i^2   4*t_i^3   5*t_i^4];
            d_ddot_i = [0     0     2    6*t_i  12*t_i^2  20*t_i^3];
            
            d_ti = [car.pathInfo.d; 0; 0]; % Initial lateral position, speed, acceleration
            
            % Final boundary conditions
            d_f =   [  1     t_f   t_f^2    t_f^3     t_f^4     t_f^5];
            d_dot_f = [0     1   2*t_f  3*t_f^2   4*t_f^3   5*t_f^4];
            d_ddot_f = [0     0     2    6*t_f  12*t_f^2  20*t_f^3];
            
            d_tf = [y_f; 0; 0]; % Final lateral position, speed, acceleration
            
            A = [d_i;d_dot_i;d_ddot_i;d_f;d_dot_f;d_ddot_f];
            B = [d_ti;d_tf];
            % Solve for the coefficients using 6 linear equations with 6 boundary conditions at t = ti and t = tf
            a = linsolve(A,B);
                        
            tP = 0:0.2:t_f; % To calculate the lane changing path points for the trajectory with 0.2 s intervals
            newWP_s=car.dynamics.speed*tP; % Target longitudinal - "s" coordinates of the path points
            newWP_d=a(1)+a(2)*tP+a(3)*tP.^2+a(4)*tP.^3+a(5)*tP.^4+a(6)*tP.^5; % "d" coordinates corresponding to the trajectory
            newWP_all =  [newWP_s' newWP_d']; % create a set of path points
            obj.ref_d = a(1)+a(2)*t_f+a(3)*t_f^2+a(4)*t_f^3+a(5)*t_f^4+a(6)*t_f^5; % reference "d" value by the end of the lane changing maneuver
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
  
        %% Override for experiment - Later incorparate into LocalTrajectoryPlanner.m
        
        function updatedPathPoints_Cartesian = Frenet2Cartesian(~,s,laneChangingPoints,currentTrajectory)
            route = currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];
            radian = currentTrajectory(3,1);
            cclockwise = currentTrajectory(4,1);
            
            if radian == 0
                route_Vector = route(2,:)-route(1,:);
                route_UnitVector = route_Vector/norm(route_Vector);
                normalVector = [-route_UnitVector(2),route_UnitVector(1)];% Fast rotation by 90 degrees to find the normal vector  
                
                
                % Lane Changing Points were already in Frenet - only "s" value should be added
                % "d" is already the reference
                updatedPathPoints_Frenet=[s+laneChangingPoints(:,1) laneChangingPoints(:,2)];
                
                updatedPathPoints_Cartesian = updatedPathPoints_Frenet(:,1)*route_UnitVector+updatedPathPoints_Frenet(:,2)*normalVector+route(1,:);
            else

                updatedPathPoints_Frenet=[s+laneChangingPoints(:,1) laneChangingPoints(:,2)];
                all_s = updatedPathPoints_Frenet(:,1);
                all_d = updatedPathPoints_Frenet(:,2);
                
                startPoint = route(1,:);
                rotationCenter = currentTrajectory(3,[2 3]).*[1 -1]; % Get the rotation center
                
                startPointVector = startPoint-rotationCenter;% Vector pointing from the rotation point to the start
                r = norm(startPointVector); % Get the radius of the rotation
                
                startPointVectorAng = atan2(startPointVector(2),startPointVector(1));
                
                l = r+(all_d*cclockwise);%current distance from rotation center to position
                lAng = all_s/r+startPointVectorAng;% the angle of vector l
                updatedPathPoints_Cartesian = l.*[cos(lAng) sin(lAng)]+rotationCenter;% the positions in Cartesian
            end
            
            

        end
                    
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon("MOBATSIM-Icon-Set_9- PurePursuit WP Generator.png");
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
