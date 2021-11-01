classdef VehicleSensors < matlab.System & handle & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon & matlab.system.mixin.SampleTime
    % Vehicle Sensor Block: Ideal radar implementation in MOBATSim.
    %
    
    % Public, tunable properties
    properties
        Vehicle_id
        Tsample
        OffsetTime
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
            
            % Output1 : V2VcommIDs -> V2V Data Links <0 or 1>                       
            V2VcommIDs = obj.vehicle.V2VdataLink;

            % Output2 : vehicleDetected -> <false or true>
            vehicleDetected = false;             
            leadingVehicle = [];
            rearVehicle = [];
            
            % Default values for NO DETECTION ->
            if obj.vehicle.pathInfo.destinationReached ...              % IF the vehicle has reached destination
                    || obj.vehicle.status.stop == true ...              % OR the vehicle has stopped
                    || isempty(obj.vehicle.pathInfo.currentTrajectory)  % OR the vehicle has no planned trajectory
                
                distanceToLeading = Inf;
                distanceToRear = Inf;                
            else
                % Detection Function
                [leadingVehicleID, distanceToLeading, rearVehicleID, distanceToRear] = obj.detectVehicles(obj.vehicle,obj.Vehicles);
                
                % Get leading vehicle if exists
                if leadingVehicleID > 0
                    leadingVehicle = obj.Vehicles(leadingVehicleID);
                    % Output2: vehicle detected
                    vehicleDetected = true;
                end
                
                % get rear vehicle if exists
                if rearVehicleID > 0
                    rearVehicle = obj.Vehicles(rearVehicleID);
                    % Output2: vehicle detected
                    vehicleDetected = true;
                end
            end
                                            
            % Register the detected vehicles to Vehicle Sensor Detection Data
            obj.vehicle.setVehicleSensorDetection(leadingVehicle,distanceToLeading,rearVehicle, distanceToRear);
        end        
        
        %% Helper functions
        function [leadingVehicleID, distanceToLeading, rearVehicleID, distanceToRear] = detectVehicles(obj ,car, Vehicles)
            % Default values for the outputs assigned arbitrary values for empty (not detected)
            leadingVehicleID = -1;
            distanceToLeading = Inf;
            
            rearVehicleID = -1;
            distanceToRear = Inf;
            
            % Sensor information about front and rear vehicles nearby (on the same route)
            [frontDetection,rearDetection] = obj.findVehiclesOnTheSameRoute(car,Vehicles);
            
            % Sensor information about a vehicle ahead (on the next route if not already found on the same route)
            if isempty(frontDetection)
                frontDetection = obj.findVehiclesOnTheNextRoute(car,Vehicles);
            end
            
            % Sensor information about a vehicle behind (on one of the previous routes if not already found on the same route)
            if isempty(rearDetection)
                rearDetection = obj.findVehiclesOnThePreviousRoutes(car,Vehicles);
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
                    if ~(car.pathInfo.laneId == vehicle_.pathInfo.laneId)
                        % Sense side vehicle for a safe lane-changing (TODO: Register as Side vehicle for more complicated algorithms)
                        relativeDistance = (car.pathInfo.s - vehicle_.pathInfo.s)-((vehicle_.physics.size(3)/2)+(car.physics.size(3)/2));
                        rearDetection(end+1,:) = [vehicle_.id relativeDistance]; %#ok<AGROW>
                    else
                        
                        relativeDistance = abs(car.pathInfo.s - vehicle_.pathInfo.s)-((vehicle_.physics.size(3)/2)+(car.physics.size(3)/2));
                        
                        if vehicle_.pathInfo.destinationReached
                            % If the "egoVehicle" is behind the "vehicle_"
                            % and "vehicle_" has reached it's destination
                            relativeDistance=norm(vehicle_.dynamics.position-car.dynamics.position)-((vehicle_.physics.size(3)/2)+(car.physics.size(3)/2));
                            frontDetection(end+1,:) = [vehicle_.id relativeDistance]; %#ok<AGROW>
                            
                        elseif (car.pathInfo.s < vehicle_.pathInfo.s)
                            % If the "egoVehicle" is behind the "vehicle_"
                            frontDetection(end+1,:) = [vehicle_.id relativeDistance]; %#ok<AGROW>
                            
                        elseif (car.pathInfo.s > vehicle_.pathInfo.s)
                            % If the "egoVehicle" is ahead of the "vehicle_"
                            rearDetection(end+1,:) = [vehicle_.id relativeDistance]; %#ok<AGROW>
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
                relativeDistance = (norm(car.dynamics.position-car.pathInfo.currentTrajectory(2,:)) + Vehicles(idx(j)).pathInfo.s)-((Vehicles(idx(j)).physics.size(3)/2)+(car.physics.size(3)/2));
                frontDetection(end+1,:) = [Vehicles(idx(j)).id relativeDistance]; %#ok<AGROW>                          
            end
                    
        end
        
        function rearDetection = findVehiclesOnThePreviousRoutes(~,car,Vehicles)
            rearDetection = [];
            i = 1:length(Vehicles);
            idx = car.map.getBackwardNeighbourRoutes(car.pathInfo.currentRoute) == cat(2,cat(2,Vehicles(i).pathInfo).currentRoute);
            [~, idx] = find(idx,length(i), 'first');
            
            for j=1:length(idx)
                %Check if there is a vehicle on one of the previous routes
                relativeDistance = (car.pathInfo.s + Vehicles(idx(j)).pathInfo.routeEndDistance)-((Vehicles(idx(j)).physics.size(3)/2)+(car.physics.size(3)/2));
                rearDetection(end+1,:) = [Vehicles(idx(j)).id relativeDistance];  %#ok<AGROW>
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
            icon = matlab.system.display.Icon("MOBATSIM-Icon-Set_4- Sensor.png");
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
