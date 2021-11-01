<<<<<<< HEAD
=======
<<<<<<< HEAD
classdef Stanley_RefPoseGenerator < LocalTrajectoryPlanner
    % This blocks generates the reference pose required by the Stanley Lateral Controller.
    %
    
    % Pre-computed constants
    properties(Access = private)
        referencePose = [0; 0; 0];
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
            radian = currentTrajectory(3,1);
            
            % Cartesian to Frenet Coordinate Transformation
            [s,d] = obj.Cartesian2Frenet(currentTrajectory,Vpos_C); % Determine current <s,d>
            
            % Update Vehicle Frenet Coordinates <s,d>
            obj.vehicle.updateVehicleFrenetPosition(s,d)
            
            if changeLane % Add <delta s, delta d>
                s = s+0.01; 
                %TODO: Not implemented yet + delta "d"
                % Generate Reference Pose for Stanley
                [refPos,refOrientation] = obj.Frenet2Cartesian(currentTrajectory,s,d,radian);%Coordinate Conversion function
                
            else % Add <delta s>
                s = s+0.01;
                % Generate Reference Pose for Stanley / obj.ref_d is the reference lateral displacement value
                [refPos,refOrientation] = obj.Frenet2Cartesian(currentTrajectory,s,obj.ref_d,radian);%Coordinate Conversion function
            end
   
            %Required format for the Stanley controller
            obj.referencePose = [refPos(1); refPos(2); refOrientation];
            
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
=======
>>>>>>> MOBATSim-development
classdef Stanley_RefPoseGenerator < LocalTrajectoryPlanner
    % This blocks generates the reference pose required by the Stanley Lateral Controller.
    %
    
    % Pre-computed constants
    properties(Access = private)
        referencePose = [0; 0; 0];
        latOffset = 0;%variable to save reference delta_d in Frenet coordinate
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
            radian = currentTrajectory(3,1);
            
            % Cartesian to Frenet Coordinate Transformation
            [s,d] = obj.Cartesian2Frenet(currentTrajectory,Vpos_C); % Determine current <s,d>
            
            % Update Vehicle Frenet Coordinates <s,d>
            obj.vehicle.updateVehicleFrenetPosition(s,d)
            
            if changeLane % Add <delta s, delta d>
                s = s+0.01; 
                %TODO: Not implemented yet + delta "d"
                % Generate Reference Pose for Stanley
                [refPos,refOrientation] = obj.Frenet2Cartesian(currentTrajectory,s,d,radian);%Coordinate Conversion function
                
            else % Add <delta s>
                s = s+0.01;
                % Generate Reference Pose for Stanley
                [refPos,refOrientation] = obj.Frenet2Cartesian(currentTrajectory,s,d,radian);%Coordinate Conversion function
            end
   
            %Required format for the Stanley controller
            obj.referencePose = [refPos(1); refPos(2); refOrientation];
            
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
<<<<<<< HEAD
=======
>>>>>>> 80aa577... ModelChecking now a own repo
>>>>>>> MOBATSim-development
