classdef VehicleDrivingMode < matlab.System & matlab.system.mixin.Propagates ...
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
        vehicles % for testing collision avoidance
        oldNextWaypoint % check if next waypoint is free, for testing collision avoidance
    end
    
    methods
        % Constructor
        function obj = VehicleDrivingMode(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.vehicle = evalin('base',strcat('Vehicle',int2str(obj.Vehicle_id)));
            obj.vehicles = evalin('base','Vehicles'); % for testing collision avoidance
            obj.oldNextWaypoint = 0;
        end
               
        function [SpeedReference, DistanceReference,LeadSpeed, DrivingMode, Dist2Stop] = stepImpl(obj,LeaderSpeed,LeaderDistance,emergencyCase)
            %This block shouldn't run if the vehicle has reached its destination
            if obj.vehicle.pathInfo.destinationReached
                SpeedReference=0;
                DistanceReference =-1;
                LeadSpeed = -1;
                DrivingMode = 1;
                Dist2Stop = -1;
                return;
            end
                        
            
            %Output 4: Driving mode
            if(emergencyCase == 0)
                % Mode 1: drive at reference speed
                DrivingMode = 1;
            elseif(emergencyCase == 1)
                % Mode 2: follow leading vehicle at platoon mode
                DrivingMode = 2;
            elseif(emergencyCase == 2)
                if (obj.vehicle.dynamics.speed - LeaderSpeed)>0
                    % Mode 3: stop
                    DrivingMode = 3;
                else
                    DrivingMode = 2;
                end
                
            else
                DrivingMode = 1;
            end
            
            %Output 1: Reference Speed
            SpeedReference = obj.vehicle.dynamics.maxSpeed;
            %Output 2: Reference distance to the vehicle in front
            DistanceReference = LeaderDistance;
            %Output 3: The speed of the leading vehicle
            LeadSpeed =LeaderSpeed;
            
            
            if(obj.vehicle.pathInfo.stopAt ~= 0)
                %Output 5: Distance to stop before a crossroad
                Dist2Stop = norm(obj.vehicle.dynamics.position - obj.vehicle.map.get_coordinates_from_waypoint(obj.vehicle.pathInfo.stopAt));
                DrivingMode = 4;
                if(emergencyCase == 2)
                    if (obj.vehicle.dynamics.speed - LeaderSpeed)>0
                        DrivingMode = 3;
                    end
                end
            else
                %Output 5: Distance to stop before a crossroad
                Dist2Stop = 0;
            end
            
            %%%%% Safety planner test %%%%%
            enableSafetyPlanner = false; % condition to enable safety planner
            if enableSafetyPlanner
                % find an other car heading to the same waypoint
                [lastWaypoints, nextWaypoints] = obj.getActiveWaypoints(obj.vehicles); % Get needed waypoints for check
                
                % check if next waypoint should be checked
                checkNextWaypoint = (obj.oldNextWaypoint == lastWaypoints(obj.vehicle.id)) ... % when heading to another waypoint
                                    || (obj.vehicle.dynamics.speed == 0); % or speed is zero
                    
                if checkNextWaypoint
                    % check if next waypoint is free
                    competingCarId = obj.checkNextWaypointClear(obj.vehicle.id, lastWaypoints, nextWaypoints);
                    if competingCarId
                        % a competing car to waypoint is found
                        competingCarSpeed = obj.vehicles(competingCarId).dynamics.speed;
                        if (competingCarSpeed ~= 0) ...
                           || (competingCarSpeed == 0) && (competingCarId>=obj.vehicle.id)
                            % stop when the other is already moving to next waypoint or
                            % is waiting but has higher Id
                            
                            % Level 2 = Emergency Brake
                            DrivingMode = 3;
                        end
                    end
                end
                
                
            end
        end
        
        function [lastWaypoints, nextWaypoints] = getActiveWaypoints(~, vehicles)
            % get the waypoints all vehicle come from and go to
            % ATTENTION: a vehicle should not pass a waypoint twice
            
            lastWaypoints = [];
            nextWaypoints = [];
            for i = 1:length(vehicles)
                % get actual waypoint from every vehicle
                pathInfo = vehicles(i).pathInfo;
                % get last waypoint
                lastWaypoints = [lastWaypoints; pathInfo.lastWaypoint];
                % find the point after the last waypoint when not
                % destination reached
                if ~pathInfo.destinationReached
                    nextWaypoint = pathInfo.path(find(pathInfo.path == pathInfo.lastWaypoint)+1);
                    nextWaypoints = [nextWaypoints; nextWaypoint];
                else
                    % when destination is reached, last waypoint is the
                    % next waypoint
                    nextWaypoints = [nextWaypoints; pathInfo.lastWaypoint];
                end
                
            end
        end 
        
        function competingCarId = checkNextWaypointClear(~, carId, lastWaypoints, nextWaypoints)
            % Check if an other, competing car is heading to next waypoint from an other waypoint

            for i = 1:length(nextWaypoints)
                if (nextWaypoints(carId) == nextWaypoints(i))&&(i~=carId)
                    % Check only other cars with the same next waypoint
                    if lastWaypoints(carId) ~= lastWaypoints(i)
                        % Find other cars coming from an other waypoint
                        % (on the same lane (last waypoint) sensor detects
                        % other vehicle)
                        competingCarId = i; % the other car also heading to this waypoint
                        return
                    end
                end
            end
            % no other car found heading to the next waypoint
            competingCarId = false;
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
            icon = matlab.system.display.Icon("BehaviouralPlanner.png");
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
        
        function [out,out2,out3,out4,out5] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = 'double';
            out2 = 'double';
            out3 = 'double';
            out4 = 'double';
            out5 = 'double';
            
            
            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end
        
        function [out,out2,out3,out4,out5] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
        
        function [out,out2,out3,out4,out5] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
            
            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
        
        function [out,out2,out3,out4,out5] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 1];
            out2 = [1 1];
            out3 = [1 1];
            out4 = [1 1];
            out5 = [1 1];
            
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
    end
end
