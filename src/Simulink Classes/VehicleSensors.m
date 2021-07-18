classdef VehicleSensors < matlab.System & handle & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon & matlab.system.mixin.SampleTime
    % Vehicle Sensor Block: Ideal radar implementation in MOBATSim.
    %
    
    % Public, tunable properties
    properties
        Vehicle_id
        Tsample = 0.01;
        OffsetTime =0;
    end
    
    % Pre-computed constants
    properties(Access = private)
        vehicle  %Ego Vehicle
        Vehicles= evalin('base','Vehicles');    %Other Vehicles (Used to generate the distance value)
    end
    
    methods
        % Constructor
        function obj = VehicleSensors(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.vehicle = obj.Vehicles(obj.Vehicle_id);
        end
        
        function [V2VcommIDs, vehicleDetected] = stepImpl(obj)
            
            % Output1 : V2VcommIDs      -> V2V Data Links (0,1)                       
            V2VcommIDs = obj.vehicle.V2VdataLink;
            
            vehicleDetected = false;
            % Default values for no detection
            if obj.vehicle.pathInfo.destinationReached ...              % No detection when destination reached
                    || obj.vehicle.status.stop == true ...              % or vehicle stopped
                    || isempty(obj.vehicle.pathInfo.currentTrajectory)  % or has no planned trajectory
                
                leadingVehicle = [];
                rearVehicle = [];
                distanceToLeading = Inf;
                distanceToRear = Inf;                
            else
                % Detection function
                % Output2: distanceToLeading    -> distance to vehicle in front
                [leadingVehicleID, distanceToLeading, rearVehicleID, distanceToRear] = obj.detectVehicles(obj.vehicle,obj.Vehicles);
                % get front vehicle if exists
                if leadingVehicleID > 0
                    leadingVehicle = obj.Vehicles(leadingVehicleID);
                    % Output2: vehicle detected
                    vehicleDetected = true;
                else
                    leadingVehicle = [];
                end
                % get rear vehicle if exists
                if rearVehicleID > 0
                    rearVehicle = obj.Vehicles(rearVehicleID);
                    % Output2: vehicle detected
                    vehicleDetected = true;
                else
                    rearVehicle = [];
                end
            end
                                            
            % update vehicle sensor data
            obj.vehicle.setVehicleSensorDetection(leadingVehicle,distanceToLeading,rearVehicle, distanceToRear);
        end        
        
        %% Helper functions
        function [leadingVehicleID, distanceToLeading, rearVehicleID, distanceToRear] = detectVehicles(obj ,car, Vehicles)
            % Default values for the outputs assigned arbitrary values for empty (not detected)
            leadingVehicleID = -1;
            distanceToLeading = Inf;
            
            rearVehicleID = -1;
            distanceToRear = Inf;
            
            % Sensor information about front and behinds vehicle nearby (on the same route)
            [frontDetection,rearDetection] = obj.findVehiclesOnTheSameRoute(car,Vehicles);
            
            % Sensor information about a vehicle ahead (on the next route if not already found on the same route)
            if isempty(frontDetection)
                frontDetection = obj.findVehiclesOnTheNextRoute(car,Vehicles);
            end
            
            if ~isempty(frontDetection)
                % Front sensor perceives a vehicle in front if it is in range
                if min(frontDetection(:,2)) <= car.sensors.frontSensorRange
                    % Get the closest vehicle in front within the sensor range
                    rowId = frontDetection(:,2)== min(frontDetection(:,2));
                    % Register the detected vehicle
                    leadingVehicleID = frontDetection(rowId,1);
                    distanceToLeading = min(frontDetection(:,2));
                end
            end
            
            if ~isempty(rearDetection)
                % Rear sensor perceives a vehicle behind if it is in range
                if min(rearDetection(:,2)) <= car.sensors.frontSensorRange
                    % Get the closest vehicle in rear within the sensor range
                    rowId = rearDetection(:,2)== min(rearDetection(:,2));
                    % Register the detected vehicles
                    rearVehicleID = rearDetection(rowId,1);
                    distanceToRear = min(rearDetection(:,2));
                end
            else
                %% TODO: Use Map getBackwardNeighbourRoutes to find vehicles behind
            end
            
        end
        
        function [frontDetection, rearDetection] = findVehiclesOnTheSameRoute(~,car,Vehicles)
            frontDetection = [];
            rearDetection = [];
            
            i = 1:length(Vehicles);
            i(car.id)=[]; % Remove the car with the same id
            VehiclesOnSameRoute = Vehicles(i(car.pathInfo.currentRoute == [cat(1,Vehicles(i).pathInfo).currentRoute]));
            
            if ~isempty(VehiclesOnSameRoute) % If there are other vehicles on the same route
                for vehicle_=VehiclesOnSameRoute
                    % This exception happens only when following vehicles
                    % reach waypoints and not update their trajectory at the right place
                    if nnz(~(car.pathInfo.currentTrajectory == car.map.getRouteDefinitionfromRouteID(car.pathInfo.currentRoute)))
                        disp('asd')
                        %Inconsistency: The Route is updated but not the Trajectory -> TODO: Fix this issue in Vehicle Kinematics later on and remove this workaround
                    elseif nnz(~(car.pathInfo.currentTrajectory == vehicle_.pathInfo.currentTrajectory))
                        disp('asd')
                        %Inconsistency: The Route is updated but not the Trajectory -> TODO: Fix this issue in Vehicle Kinematics later on and remove this workaround
                    elseif ~(car.pathInfo.laneId == vehicle_.pathInfo.laneId) %TODO: Check if this logic about different lanes holds for all situations
                        % Sense side vehicle for a safe lane-changing (TODO: Either carry to Side vehicle or update Rear Vehicle)
                        relativeDistance = (car.pathInfo.s - vehicle_.pathInfo.s)-((vehicle_.physics.size(3)/2)+(car.physics.size(3)/2));
                        rearDetection = [rearDetection; [vehicle_.id relativeDistance]];
                    else
                        
                        relativeDistance = abs(car.pathInfo.s - vehicle_.pathInfo.s)-((vehicle_.physics.size(3)/2)+(car.physics.size(3)/2));
                        
                        if vehicle_.pathInfo.destinationReached
                            % If the "egoVehicle" is behind the "vehicle_"
                            % and "vehicle_" has reached it's destination
                            relativeDistance=norm(vehicle_.dynamics.position-car.dynamics.position)-((vehicle_.physics.size(3)/2)+(car.physics.size(3)/2));
                            frontDetection = [frontDetection; [vehicle_.id relativeDistance]];
                            
                        elseif (car.pathInfo.s < vehicle_.pathInfo.s)
                            % If the "egoVehicle" is behind the "vehicle_"
                            frontDetection = [frontDetection; [vehicle_.id relativeDistance]];
                            
                        elseif (car.pathInfo.s > vehicle_.pathInfo.s)
                            % If the "egoVehicle" is ahead of the "vehicle_"
                            rearDetection = [rearDetection; [vehicle_.id relativeDistance]];
                        end
                    end
                end
            end
            
            
        end
        
        function frontDetection = findVehiclesOnTheNextRoute(~,car,Vehicles)
            frontDetection = [];
            i = 1:length(Vehicles);
            idx = car.map.getForwardNeighbourRoutes(car.pathInfo.currentRoute) == cat(2,cat(2,Vehicles(i).pathInfo).currentRoute);
            [~, idx] = find(idx,length(i), 'first');
            
            for j=1:length(idx)
                %Check if there is a vehicle on the next
                %neighbouring routes
                if nnz(~(car.pathInfo.currentTrajectory == car.map.getRouteDefinitionfromRouteID(car.pathInfo.currentRoute)))
                    %Inconsistency: The Route is updated but not the Trajectory -> TODO: Fix this issue in Vehicle Kinematics later on and remove this workaround
                else
                    relativeDistance = (norm(car.dynamics.position-car.pathInfo.currentTrajectory(2,:)) + Vehicles(idx(j)).pathInfo.s)-((Vehicles(idx(j)).physics.size(3)/2)+(car.physics.size(3)/2));
                    frontDetection = [frontDetection; [Vehicles(idx(j)).id relativeDistance]];
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
        
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            
            % Set private and protected properties
            % obj.myproperty = s.myproperty;
            
            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
               
        function flag = isInputSizeLockedImpl(~,~)
            % Return true if input size is not allowed to change while
            % system is running
            flag = true;
        end
             
        function sts = getSampleTimeImpl(obj)
            % Define sample time type and parameters
            %sts = obj.createSampleTime("Type", "Inherited");
            
            % Example: specify discrete sample time
            sts = createSampleTime(obj,'Type','Discrete',...
                'SampleTime',obj.Tsample, ...
                'OffsetTime',obj.OffsetTime);
        end
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon("sensor.png");
        end
    end
    
    methods(Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename('class'));
        end
        
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
        
        function [out,out2] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [1 length(obj.Vehicles)];
            out2 = [1 1];            
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
        
        function [out,out2] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = 'double';
            out2 = 'boolean';
            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end
        
        function [out,out2] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
        
        function [out,out2] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
        
        function ds = getDiscreteStateImpl(~)
            % Return structure of properties with DiscreteState attribute
            ds = struct([]);
        end
        
        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(mfilename('class'));
        end
    end
end
