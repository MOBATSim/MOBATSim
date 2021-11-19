classdef Stanley_RefPoseGenerator < LocalTrajectoryPlanner
    % This blocks generates the reference pose required by the Stanley Lateral Controller.
    %
    
    % Pre-computed constants
    properties(Access = private)
        referencePose = [0; 0; 0];
        
        wheelBase = 3;
        d_dot_ref % Reference lateral speed during lane changing maneuver
        roadOrientation % Orientation of the current trajectory
    end
    
    methods
        % Constructor
        function obj = Stanley_RefPoseGenerator(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@LocalTrajectoryPlanner(obj);  % Inherit the setupImpl function of the Superclass @WaypointGenerator
        end
        
        function [poseOut, referencePose] = stepImpl(obj,pose,speed,changeLane)
            
            obj.registerVehiclePoseAndSpeed(obj.vehicle,pose,speed); % Sets/Registers vehicle current Pose and speed
            
            centerFrontAxle_C = obj.getVehicleFrontAxleCenterPoint(pose, obj.wheelBase);
            
            pose(3) = rad2deg(pose(3)); % rad to deg
            
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
            
            % Cartesian to Frenet Coordinate Transformation
            [s,d] = obj.Cartesian2Frenet(currentTrajectory,Vpos_C); % Determine current <s,d>
            
            % Update Vehicle Frenet Coordinates <s,d>
            obj.vehicle.updateVehicleFrenetPosition(s,d)
            
            if changeLane 
                [obj.laneChangingPoints, obj.roadOrientation, obj.d_dot_ref]  = obj.generateLaneChangingPoints(s, changeLane, currentTrajectory);
                % TODO: Update necessary when current trajectory changes during maneuver
            end
            
            if ~isempty(obj.laneChangingPoints) % Executing maneuver
                [refPos, idxReference] = obj.getClosestPointOnTrajectory(centerFrontAxle_C, obj.laneChangingPoints);
                refOrientation = atan2(obj.d_dot_ref(idxReference), speed) + obj.roadOrientation(idxReference); 
                if idxReference >= size(obj.laneChangingPoints, 1)
                    obj.laneChangingPoints = [];
                    obj.finishLaneChangingManeuver(0.05);
                end
            else % Add <delta s>
                %s = s+0.01;
                
                [s, ~] = obj.Cartesian2Frenet(currentTrajectory, centerFrontAxle_C); % Projection of font axle positon on current Frenet reference trajectory
                % Generate Reference Pose for Stanley / obj.ref_d is the reference lateral displacement value
                [refPos, refOrientation] = obj.Frenet2Cartesian(s, obj.ref_d, currentTrajectory);%Coordinate Conversion function
            end
            
            %Required format for the Stanley controller
            obj.referencePose = [refPos(1); refPos(2); rad2deg(refOrientation)]; % Orientation in degree for Stanley
            
            % Check if the Waypoint is Reached
            obj.vehicle.checkWaypointReached(currentTrajectory(2,:));
     
            %Output 1: Reference Pose on the followed trajectory
            referencePose = obj.referencePose';
            %Output 2: Current Pose of the vehicle
            poseOut=pose';
            
        end
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon("MOBATSIM-Icon-Set_10- Stanley.png");
        end   
    end
    
    methods(Static)
        function centerFrontAxle = getVehicleFrontAxleCenterPoint(poseRearAxle, wheelBase)
            % Get the center of the vehicle's front axle by transforming the vehicle's location
            
            x_RearAxle = poseRearAxle(1);
            y_RearAxle = poseRearAxle(2);
            yaw = poseRearAxle(3);
            
            x_centerFrontAxle = x_RearAxle + wheelBase*cos(yaw);
            y_centerFrontAxle = y_RearAxle + wheelBase*sin(yaw);
            
            centerFrontAxle = [x_centerFrontAxle, y_centerFrontAxle];
        end
        
        function [closestPoint, idxInTrajectory] = getClosestPointOnTrajectory(point, trajectory)
            % Calculate which point on a trajectory is closest to a given point
            
            [~, idxInTrajectory] = min(sum((trajectory - point).^2, 2));
            closestPoint = trajectory(idxInTrajectory, :);
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
