classdef VehicleDrivingMode_Ego < matlab.System & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
    % Behavioral Planner - Vehicle determines its driving mode.
    %
    
    % Public, tunable properties
    properties
        Vehicle_id
    end
    
    % Pre-computed constants
    properties(Access = private)
        vehicle
        LaneChangeTime = 4;
    end
    
    methods
        % Constructor
        function obj = VehicleDrivingMode_Ego(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.vehicle = evalin('base', "Vehicles(" + obj.Vehicle_id + ")");
        end
        
        function [SpeedReference, DistanceReference,LeadSpeed, DrivingMode, Dist2Stop, laneChange] = stepImpl(obj, vehicleDetected) %#ok<INUSD>
            %This block shouldn't run if the ego vehicle has reached its destination
            if obj.vehicle.pathInfo.destinationReached
                SpeedReference=0;
                DistanceReference = -1;
                LeadSpeed = -1;
                DrivingMode = 1;
                Dist2Stop = -1;
                laneChange = 0;
                return;
            end
            
            % Evaluation of lane-changing maneuver
            if vehicleDetected
                laneChange = switch_decision(obj);
            else
                laneChange = 0;
            end
            
            
            if laneChange
                DrivingMode = 1;
                SpeedReference=obj.vehicle.dynamics.maxSpeed;
                DistanceReference =-1;
                LeadSpeed = -1;
                Dist2Stop = -1;
                return;
            else
                % get sensor information from vehicle
                distanceToLeadingVehicle = obj.vehicle.sensors.distanceToLeadingVehicle;
                leadingVehicleSpeed = obj.vehicle.sensors.leadingVehicleSpeed;
                %Output 1: Reference Speed
                SpeedReference = obj.vehicle.dynamics.maxSpeed;
                
                %Output 2: Reference distance to the vehicle in front
                DistanceReference = distanceToLeadingVehicle;
                % Limit leader distance
                if DistanceReference > 100
                    DistanceReference = 100;
                end
                
                %Output 3: The speed of the leading vehicle
                LeadSpeed = leadingVehicleSpeed;
                %Output 4: Driving mode
                if vehicleDetected % check if vehicle detected
                    if distanceToLeadingVehicle > obj.vehicle.sensors.frontSensorRange ...
                            || distanceToLeadingVehicle < 0 ... % Front vehicle out of sensor range
                            || distanceToLeadingVehicle > 4*(obj.vehicle.dynamics.speed-LeadSpeed) % There is enough distance
                        % Mode 1 = Drive at reference speed
                        DrivingMode = 1;
                        
                    elseif distanceToLeadingVehicle > obj.vehicle.sensors.AEBdistance
                        % Mode 2 = Follow leading vehicle at platoon mode
                        DrivingMode = 2;
                        
                    elseif (leadingVehicleSpeed - obj.vehicle.dynamics.speed)>0 % Too close but leading vehicle is faster
                        % Mode 2 = Follow leading vehicle at platoon mode
                        DrivingMode = 2;
                        
                    else % Leading vehicle too close !!!
                        % Mode 3 = Stop
                        DrivingMode = 3;
                        
                    end
                else
                    %M ode 1 = Drive at reference speed
                    DrivingMode = 1;
                end
                
            end
            
            
            
            
            
            
            if(obj.vehicle.pathInfo.stopAt ~= 0)
                %Output 5: Distance to stop before a crossroad
                Dist2Stop = norm(obj.vehicle.dynamics.position - obj.vehicle.map.get_coordinates_from_waypoint(obj.vehicle.pathInfo.stopAt));
                if ~DrivingMode == 3
                    % choose approach the crossroad when not stopping
                    DrivingMode = 4;
                end
            else
                %Output 5: Distance to stop before a crossroad
                Dist2Stop = 0;
            end
            obj.vehicle.setDrivingMode(DrivingMode);
            
            
        end
        %% helper function
        function laneChange = switch_decision(obj)
            
            if checkNecessaryConditionsforLaneChanging(obj)
                if (obj.vehicle.pathInfo.laneId==0)... % if vehicle is already on the right lane
                        &&(obj.vehicle.dynamics.maxSpeed >obj.vehicle.sensors.leadingVehicleSpeed*2)... % and the vehicle is more than 2 times faster than the lead vehicle
                        &&(obj.vehicle.sensors.distanceToLeadingVehicle <(obj.vehicle.dynamics.speed-obj.vehicle.sensors.leadingVehicleSpeed)*4)% and the vehicle can overtake in 4 seconds
                    obj.vehicle.status.canLaneSwitch = 1;%left lane-changing command
                    laneChange = 1;
                elseif (obj.vehicle.pathInfo.laneId==1)&&(abs(obj.vehicle.sensors.rearVehicleSafetyMargin)>2)&&(obj.vehicle.status.ttc>3.5)%conditions for right lane-changing
                    obj.vehicle.status.canLaneSwitch = 2;%right lane-changing command
                    laneChange = 2;
                else
                    laneChange = 0;
                end
            else
                laneChange = 0;
            end
            
        end
        
        function allowed = checkNecessaryConditionsforLaneChanging(obj)
            % Check if currentRoute variable Exists and NON-zero and NOT Single lane
            if isempty(obj.vehicle.pathInfo.currentRoute) || obj.vehicle.pathInfo.currentRoute ==0 || obj.vehicle.map.get_lane_number_from_route(obj.vehicle.pathInfo.currentRoute)==1
                allowed = 0;
            else
                % Check if there is enough distance in the route
                if (obj.vehicle.pathInfo.routeEndDistance> obj.vehicle.dynamics.speed*obj.LaneChangeTime)
                    allowed = 1;
                else
                    % Check if there is a next route
                    if length(obj.vehicle.pathInfo.path)>=3
                        % Check if there is enough distance if the next route is also double lane and straight
                        % (plannedNextRoute for curved roads is dangerous because its curvature is unknown at the time)
                        plannedNextRoute = obj.vehicle.map.getRouteIDfromPath(obj.vehicle.pathInfo.path([2 3]));
                        if ~isempty(plannedNextRoute) && ~(obj.vehicle.map.get_lane_number_from_route(plannedNextRoute)==1) && (obj.vehicle.pathInfo.currentTrajectory(3,1) == 0)
                            allowed = 1;
                        else
                            allowed = 0;
                        end
                    else
                        allowed = 0;
                    end
                    
                end
                
            end
        end
        
        %% Standard Simulink Output functions
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            
            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);
            
            % Set private and protected properties
            %s.myproperty = obj.myproperty;
        end
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon("situationAwareness.png");
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            
            % Set private and protected properties
            % obj.myproperty = s.myproperty;
            
            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
    end
    methods(Static, Access = protected)
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename('class'));
        end
        
        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(mfilename('class'));
        end
        
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
        
        function ds = getDiscreteStateImpl(~)
            % Return structure of properties with DiscreteState attribute
            ds = struct([]);
        end
        
        function flag = isInputSizeLockedImpl(~,~)
            % Return true if input size is not allowed to change while
            % system is running
            flag = true;
        end
        
        function [out,out2,out3,out4,out5,out6] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = 'double';
            out2 = 'double';
            out3 = 'double';
            out4 = 'double';
            out5 = 'double';
            out6 = 'double';
            
            
            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end
        
        function [out,out2,out3,out4,out5,out6] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
        
        function [out,out2,out3,out4,out5,out6] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
            out6 = true;
            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
        
        function [out,out2,out3,out4,out5,out6] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 1];
            out2 = [1 1];
            out3 = [1 1];
            out4 = [1 1];
            out5 = [1 1];
            out6 = [1 1];
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
    end
end
