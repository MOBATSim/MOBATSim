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
        vehicle %Ego Vehicle
        Vehicles %Other Vehicles (Used to generate the distance value)
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
            obj.vehicle = evalin('base',strcat('Vehicle',int2str(obj.Vehicle_id)));
            obj.Vehicles = evalin('base','Vehicles');
        end
        
        
        function [V2VcommIDs, ObjectinFront] = stepImpl(obj)
            %This block shouldn't run if the vehicle has reached its
            %destination
            if obj.vehicle.pathInfo.destinationReached
                % 2 Outputs: Vehicle in front id, Distance to the vehicle in front
                V2VcommIDs = -1;
                ObjectinFront = -1;
                
            else
                % Detect Vehicles around if the vehicle is not on halt
                if obj.vehicle.status.stop ==0 && ~isempty(obj.vehicle.pathInfo.currentTrajectory)
                    [V2VcommIDs, ObjectinFront] = obj.detectVehicles(obj.vehicle,obj.Vehicles);
                    %%
                    %new function from Qihang
                    check_leadingVehicle(obj);
                    check_behindVehicle(obj);
                    %check_doubleLane(obj);
                    %%
                else
                    % No detection value if it is on halt
                    V2VcommIDs = -1;
                    ObjectinFront = -1;
                end
                
            end
            
            
            % Output1 : V2VcommIDs      -> Vehicle in front id
            % Output2 : ObjectinFront   -> Distance to the vehicle in front
            obj.vehicle.setVehicleFrontSensor(V2VcommIDs,ObjectinFront)
        end
        
        
        
        
        
        
        
        
        
        
        function [V2VcommIDs, ObjectinFront] = detectVehicles(~,car, Vehicles)
            %% Sensor information about a vehicle nearby (on the same route)
            id_distance = [];
            
            i = 1:length(Vehicles);
            i(car.id)=[]; % Remove the car with the same id
            
            VehiclesOnSameRoute=Vehicles(i(car.pathInfo.currentRoute == [cat(1,Vehicles([i]).pathInfo).currentRoute]));
            
            if ~isempty(VehiclesOnSameRoute)
                %If there is a vehicle on the same route,
                c_distancetoDestination = norm(car.dynamics.position-car.pathInfo.currentTrajectory(2,:));
                
                for vehicle_=VehiclesOnSameRoute
                    v_distancetoDestination = norm(vehicle_.dynamics.position-vehicle_.pathInfo.currentTrajectory(2,:));
                    
                    if (c_distancetoDestination > v_distancetoDestination)
                        % If the "car" is behind the "vehicle"
                        distance_ = norm(vehicle_.dynamics.position-car.dynamics.position)-((vehicle_.physics.size(3)/2)+(car.physics.size(3)/2));
                        id_ = vehicle_.id;
                        id_distance = [id_distance; [id_ distance_]];
                    end
                    
                end
            end
            %% Sensor information about a vehicle ahead (on the next route)
            if isempty(id_distance)
                i = 1:length(Vehicles);
                idx = car.map.getForwardNeighbourRoutes(car.pathInfo.currentRoute) == cat(2,cat(2,Vehicles(i).pathInfo).currentRoute);
                [~, idx] = find(idx,length(i), 'first');
                
                for j=1:length(idx)
                    %Check if there is a vehicle on the next
                    %neighbouring routes
                    distance_ = (norm(car.dynamics.position-car.pathInfo.currentTrajectory(2,:)) + norm(Vehicles(idx(j)).dynamics.position-car.pathInfo.currentTrajectory(2,:)))-((Vehicles(idx(j)).physics.size(3)/2)+(car.physics.size(3)/2));
                    id_ = Vehicles(idx(j)).id;
                    id_distance = [id_distance; [id_ distance_]];
                end
            end
            
            if ~isempty(id_distance)
                % Front sensor perceives a vehicle in front if it is in range
                if min(id_distance(:,2)) <= car.sensors.frontSensorRange
                    
                    rowId = id_distance(:,2)== min(id_distance(:,2));
                    
                    
                    V2VcommIDs = id_distance(rowId,1);
                    ObjectinFront = min(id_distance(:,2));
                    return;
                else
                    ObjectinFront = 1000; %we can also assign an arbitrary value
                    V2VcommIDs = 0;
                end
                
            else
                ObjectinFront = 1000; %we can also assign an arbitrary value
                V2VcommIDs = 0;
                return;
            end
        end
        function check_leadingVehicle(obj)
            %this function checks if there's a leading vehicle ahead, if
            %multiple leading vehicles exist, set nearest vehicle to be the
            %leading vehicle
            leadingVehicle = [];
            ego_route = obj.vehicle.pathInfo.currentRoute;% Search vehicle on this route
            str1 = 'obj.vehicle.map.Vehicles';
            %%
            %traverse leading Vehicle on current route
            for i = 1:length(eval(str1))
                checkVehicle = eval(strcat(str1,'(',int2str(i),')'));%To be checked vehicle
                if isequal(checkVehicle.pathInfo.currentRoute,ego_route)&&(checkVehicle.pathInfo.s>obj.vehicle.pathInfo.s)%If this vehicle is on the same route and ahead of the ego vehicle
                    if isempty(leadingVehicle)%If no leading vehicle exists, set this vehicle to be the leading vehicle
                        leadingVehicle = checkVehicle;
                    elseif checkVehicle.pathInfo.s<leadingVehicle.pathInfo.s %if there's already a leading vehicle, find the closest one 
                        leadingVehicle = checkVehicle;
                    end
                end
                
            end
            if ~isempty(leadingVehicle)
                relativeSpeed = obj.vehicle.dynamics.speed-leadingVehicle.dynamics.speed;
                relativeDistance = leadingVehicle.pathInfo.s-obj.vehicle.pathInfo.s;
                obj.vehicle.sensors.ttc = relativeDistance/relativeSpeed;
                obj.vehicle.sensors.leadingVehicle = leadingVehicle;
            else
                %continue search next route
                idx = find(obj.vehicle.pathInfo.path==obj.vehicle.pathInfo.lastWaypoint);
                if idx+2<=length(obj.vehicle.pathInfo.path) % Next Route
                    nextRoute = obj.vehicle.map.getRouteIDfromPath([obj.vehicle.pathInfo.path(idx+1) obj.vehicle.pathInfo.path(idx+2)]);
                else % Destination Reached // CurrentRoute stays the same
                    nextRoute = obj.vehicle.pathInfo.currentRoute;
                end
                for i = 1:length(eval(str1))
                    if nextRoute == obj.vehicle.pathInfo.currentRoute
                        break;
                    end
                    checkVehicle = eval(strcat(str1,'(',int2str(i),')'));
                    if isequal(checkVehicle.pathInfo.currentRoute,nextRoute)
                        if isempty(leadingVehicle)
                            leadingVehicle = checkVehicle;
                        elseif checkVehicle.pathInfo.s<leadingVehicle.pathInfo.s
                            leadingVehicle = checkVehicle;
                        end
                    end
                end
                if ~isempty(leadingVehicle)
                    relativeSpeed = obj.vehicle.dynamics.speed-leadingVehicle.dynamics.speed;
                    relativeDistance = leadingVehicle.pathInfo.s+obj.vehicle.pathInfo.routeEndDistance;
                    obj.vehicle.sensors.ttc = relativeDistance/relativeSpeed;
                    obj.vehicle.sensors.leadingVehicle = leadingVehicle;
                else
                    obj.vehicle.sensors.ttc = 1000;
                    obj.vehicle.sensors.leadingVehicle = [];
                end
            end
            
        end
        
        function check_behindVehicle(obj)
            %this function checks if there's a vehicle behind, if
            %multiple leading vehicles exist, set nearest vehicle to be the
            %leading vehicle
            if obj.vehicle.pathInfo.startingPoint == obj.vehicle.pathInfo.lastWaypoint
                obj.vehicle.pathInfo.staticPath = obj.vehicle.pathInfo.path;
            end
            behindVehicle = [];
            ego_route = obj.vehicle.pathInfo.currentRoute;
            str1 = 'obj.vehicle.map.Vehicles';
            %%
            %search leading Vehicle
            for i = 1:length(eval(str1))
                checkVehicle = eval(strcat(str1,'(',int2str(i),')'));
                if isequal(checkVehicle.pathInfo.currentRoute,ego_route)&&(checkVehicle.pathInfo.s<obj.vehicle.pathInfo.s)
                    if isempty(behindVehicle)
                        behindVehicle = checkVehicle;
                    elseif checkVehicle.pathInfo.s>behindVehicle.pathInfo.s
                        behindVehicle = checkVehicle;
                    end
                end
            end
            %%
            %store behind vehicle information
            if ~isempty(behindVehicle)
                relativeSpeed = obj.vehicle.dynamics.speed-behindVehicle.dynamics.speed;
                relativeDistance = obj.vehicle.pathInfo.s-behindVehicle.pathInfo.s;
                safetyTimeMargin = relativeDistance/relativeSpeed;
                obj.vehicle.sensors.behindVehicleSafetyMargin = safetyTimeMargin;
                obj.vehicle.sensors.behindVehicle = behindVehicle;
            else
                %continue to check last route
                idx = find(obj.vehicle.pathInfo.staticPath==obj.vehicle.pathInfo.lastWaypoint);
                if idx>1 % last Route
                    lastRoute = obj.vehicle.map.getRouteIDfromPath([obj.vehicle.pathInfo.staticPath(idx-1) obj.vehicle.pathInfo.staticPath(idx)]);
                else % Destination Reached // CurrentRoute stays the same
                    lastRoute = obj.vehicle.pathInfo.currentRoute;
                end
                for i = 1:length(eval(str1))
                    if lastRoute == obj.vehicle.pathInfo.currentRoute
                        break;
                    end
                    checkVehicle = eval(strcat(str1,'(',int2str(i),')'));
                    if isequal(checkVehicle.pathInfo.currentRoute,lastRoute)
                        if isempty(behindVehicle)
                            behindVehicle = checkVehicle;
                        elseif checkVehicle.pathInfo.s>behindVehicle.pathInfo.s
                            behindVehicle = checkVehicle;
                        end
                    end
                end
                if ~isempty(behindVehicle)
                    relativeSpeed = obj.vehicle.dynamics.speed-behindVehicle.dynamics.speed;
                    relativeDistance = obj.vehicle.pathInfo.s+behindVehicle.pathInfo.routeEndDistance;
                    safetyTimeMargin = relativeDistance/relativeSpeed;
                    obj.vehicle.sensors.behindVehicleSafetyMargin = safetyTimeMargin;
                    obj.vehicle.sensors.behindVehicle = behindVehicle;
                else
                    obj.vehicle.sensors.behindVehicleSafetyMargin = 1000;
                    obj.vehicle.sensors.behindVehicle = [];
                end
            end
        end
        function check_doubleLane(obj)
            ego_route = obj.vehicle.pathInfo.currentRoute;
            if ismember(ego_route,obj.vehicle.map.doubleLane)
                obj.vehicle.sensors.onDoubleLane = 1;
            else
                obj.vehicle.sensors.onDoubleLane = 0;
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
        
        function [out,out2] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 1];
            out2 = [1 1];
            
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
