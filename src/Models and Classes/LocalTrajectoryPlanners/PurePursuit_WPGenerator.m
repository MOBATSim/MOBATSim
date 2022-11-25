classdef PurePursuit_WPGenerator < LocalTrajectoryPlanner
    % This blocks generates waypoints for the Pure Pursuit lateral controller.
    %
    
    % Pre-computed constants
    properties(Access = private)  
        Kpoints = 6; % The number of next path points to be output to the Pure Pursuit controller
        
        currentPathPoints =[];  % Arrays of waypoints to follow
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
                obj.laneChangingPoints  = obj.generateMinJerkTrajectory(obj.vehicle, obj.laneChangeTime, changeLane, s, currentTrajectory);
                obj.laneChangingPoints = obj.laneChangingPoints(:, 1:2); % Orientation not necessary for Pure Pursuit
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
                    [nextWPs, ~] = obj.Frenet2Cartesian(s_next,d_next,currentTrajectory);
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
                obj.finishLaneChangingManeuver(0.05);
            end
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
