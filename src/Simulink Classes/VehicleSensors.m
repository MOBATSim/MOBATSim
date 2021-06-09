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
        vehicle     %Ego Vehicle
        Vehicles    %Other Vehicles (Used to generate the distance value)
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
            obj.Vehicles = evalin('base','Vehicles');
            obj.vehicle = obj.Vehicles(obj.Vehicle_id);
        end
        
        
        function [V2VcommIDs, ObjectinFront] = stepImpl(obj)

            if obj.vehicle.pathInfo.destinationReached %Skip function if the vehicle has reached its destination
                % 2 Outputs: Vehicle in front id, Distance to the vehicle in front
                V2VcommIDs = -1;
                ObjectinFront = -1;
                % 2 variables needed to register % TODO check if it should be also output of the function
                V2VcommID_back = -1;
                ObjectBehind = -1;
                % Break out
                
            else
                % Detect Vehicles around if the ego vehicle is not on halt
                if obj.vehicle.status.stop ==0 && ~isempty(obj.vehicle.pathInfo.currentTrajectory)
                    % Detection function
                    [V2VcommIDs, ObjectinFront, V2VcommID_back, ObjectBehind] = obj.detectVehicles(obj.vehicle,obj.Vehicles);
                                  
                else
                    % No detection value if the ego vehicle is on halt
                    V2VcommIDs = -1;
                    ObjectinFront = -1;
                    V2VcommID_back = -1;
                    ObjectBehind = -1;
                end
                
            end
            
            
            % Output1 : V2VcommIDs      -> Vehicle in front id
            % Output2 : ObjectinFront   -> Distance to the vehicle in front
            obj.vehicle.setVehicleSensorDetection(V2VcommIDs,ObjectinFront,V2VcommID_back, ObjectBehind)
        end
        
        
        
        function [V2VcommIDs, ObjectinFront, V2VcommID_back, ObjectBehind] = detectVehicles(~ ,car, Vehicles)
                %% Detect vehicles ahead
                %[ttc, leadingVehicle] = check_leadingVehicle(obj); %new function from Qihang
                %% Detect vehicles behind
                %[behindVehicleSafetyMargin, behindVehicle] = check_behindVehicle(obj); %new function from Qihang
                
                %% Default values for the outputs %we can also assign an arbitrary value % TODO remove redundant variables
                V2VcommIDs = -1;
                ObjectinFront = 1000;
                
                V2VcommID_back = -1;
                distance_back = 1000; % Redundant variable for ObjectBehind to avoid sending empty assignment at the end of the function

            %% Sensor information about front and behinds vehicle nearby (on the same route)
            id_distance = [];
            id_distance_back = [];
            
            i = 1:length(Vehicles);
            i(car.id)=[]; % Remove the car with the same id
            
            VehiclesOnSameRoute=Vehicles(i(car.pathInfo.currentRoute == [cat(1,Vehicles(i).pathInfo).currentRoute]));
            
            if ~isempty(VehiclesOnSameRoute)
                %If there is a vehicle on the same route,
                c_distancetoDestination = norm(car.dynamics.position-car.pathInfo.currentTrajectory(2,:));
                
                for vehicle_=VehiclesOnSameRoute
                    v_distancetoDestination = norm(vehicle_.dynamics.position-vehicle_.pathInfo.currentTrajectory(2,:));
                    
                    if nnz(~(car.pathInfo.currentTrajectory == car.map.getRouteDefinitionfromRouteID(car.pathInfo.currentRoute)))
                        break; %Inconsistency: The Route is updated but not the Trajectory -> TODO: Fix this issue in Vehicle Kinematics later on and remove this workaround
                    end
                    
                    if (c_distancetoDestination > v_distancetoDestination)
                        % If the "egoVehicle" is behind the "vehicle"
                        relativedistance = norm(vehicle_.dynamics.position-car.dynamics.position)-((vehicle_.physics.size(3)/2)+(car.physics.size(3)/2));
                        %obj.vehicle.pathInfo.s-behindVehicle.pathInfo.s % TODO: At some point "s" parameter should be used
                        %to increase accuracy rather than direct distances to the goal point especially for curved roads
                        id_ = vehicle_.id; % ID of the vehicle ahead
                        id_distance = [id_distance; [id_ relativedistance]];
                        
                    elseif (c_distancetoDestination <= v_distancetoDestination)
                        % If the "egoVehicle" is ahead of the "vehicle"            
                        distance_back = norm(vehicle_.dynamics.position-car.dynamics.position)-((vehicle_.physics.size(3)/2)+(car.physics.size(3)/2));
                        
                        V2VcommID_back = vehicle_.id; % ID of the vehicle behind                        
                        id_distance_back = [id_distance_back; [V2VcommID_back distance_back]];
                    end
                    
                end
            end
            
            %% Sensor information about a vehicle ahead (on the next route if not already found on the same route)
            if isempty(id_distance)
                i = 1:length(Vehicles);
                idx = car.map.getForwardNeighbourRoutes(car.pathInfo.currentRoute) == cat(2,cat(2,Vehicles(i).pathInfo).currentRoute);
                [~, idx] = find(idx,length(i), 'first');
                
                for j=1:length(idx)
                    %Check if there is a vehicle on the next
                    %neighbouring routes
                    relativedistance = (norm(car.dynamics.position-car.pathInfo.currentTrajectory(2,:)) + norm(Vehicles(idx(j)).dynamics.position-car.pathInfo.currentTrajectory(2,:)))-((Vehicles(idx(j)).physics.size(3)/2)+(car.physics.size(3)/2));
                    
                    id_ = Vehicles(idx(j)).id;
                    id_distance = [id_distance; [id_ relativedistance]];
                    
                end
            end

            if ~isempty(id_distance)
                % Front sensor perceives a vehicle in front if it is in range
                if min(id_distance(:,2)) <= car.sensors.frontSensorRange
                    
                    rowId = id_distance(:,2)== min(id_distance(:,2));
                    
                    % Register the detected vehicles
                    V2VcommIDs = id_distance(rowId,1);
                    ObjectinFront = min(id_distance(:,2));
                    ObjectBehind = id_distance_back;                    
                    return;
                else
                    % There is a vehicle in front actually but not in the range of the front sensor
                    ObjectBehind = distance_back;
                    return;
                end
                
            else
                % There is no vehicle in front, nor any perceived
                ObjectBehind = distance_back;
                return;
            end
            
        end
        
       
      %% Will be removed after checking the main function  
        function [ttc,leadingVehicle] = check_leadingVehicle(obj) 
            %this function checks if there's a leading vehicle ahead, if
            %multiple leading vehicles exist, set nearest vehicle to be the
            %leading vehicle
            leadingVehicle = [];
            ego_route = obj.vehicle.pathInfo.currentRoute;% Search vehicle on this route
            

            %%
            %traverse leading Vehicle on current route
            for vehicle_ = obj.Vehicles
                if vehicle_.id == obj.vehicle.id
                    break;
                end
                if isequal(vehicle_.pathInfo.currentRoute,ego_route)&&(vehicle_.pathInfo.s>obj.vehicle.pathInfo.s)%If this vehicle is on the same route and ahead of the ego vehicle
                    if isempty(leadingVehicle)%If no leading vehicle exists, set this vehicle to be the leading vehicle
                        leadingVehicle = vehicle_;
                    elseif vehicle_.pathInfo.s<leadingVehicle.pathInfo.s %if there's already a leading vehicle, find the closest one
                        leadingVehicle = vehicle_;
                    end
                end
            end
            
            
            if ~isempty(leadingVehicle)
                relativeSpeed = obj.vehicle.dynamics.speed-leadingVehicle.dynamics.speed;
                relativeDistance = leadingVehicle.pathInfo.s-obj.vehicle.pathInfo.s;
                ttc = relativeDistance/relativeSpeed;
            else
                %continue search next route
                idx = find(obj.vehicle.pathInfo.path==obj.vehicle.pathInfo.lastWaypoint);
                if idx+2<=length(obj.vehicle.pathInfo.path) % Next Route
                    nextRoute = obj.vehicle.map.getRouteIDfromPath([obj.vehicle.pathInfo.path(idx+1) obj.vehicle.pathInfo.path(idx+2)]);
                else % Destination Reached // CurrentRoute stays the same
                    nextRoute = obj.vehicle.pathInfo.currentRoute;
                end
                for vehicle_ = obj.Vehicles
                    if nextRoute == obj.vehicle.pathInfo.currentRoute || vehicle_.id == obj.vehicle.id
                        break;
                    end

                    if isequal(vehicle_.pathInfo.currentRoute,nextRoute)
                        if isempty(leadingVehicle)
                            leadingVehicle = vehicle_;
                        elseif vehicle_.pathInfo.s<leadingVehicle.pathInfo.s
                            leadingVehicle = vehicle_;
                        end
                    end
                end
                if ~isempty(leadingVehicle)
                    relativeSpeed = obj.vehicle.dynamics.speed-leadingVehicle.dynamics.speed;
                    relativeDistance = leadingVehicle.pathInfo.s+obj.vehicle.pathInfo.routeEndDistance;
                    ttc = relativeDistance/relativeSpeed;
                else
                    ttc = 1000;
                    leadingVehicle = [];
                end
            end
            
        end
        
        function [behindVehicleSafetyMargin, behindVehicle] = check_behindVehicle(obj)
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
                behindVehicleSafetyMargin = safetyTimeMargin;
                behindVehicle = behindVehicle;
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
                    relativeSpeed = behindVehicle.dynamics.speed - obj.vehicle.dynamics.speed;
                    relativeDistance = obj.vehicle.pathInfo.s+behindVehicle.pathInfo.routeEndDistance;
                    safetyTimeMargin = relativeDistance/relativeSpeed;
                    behindVehicleSafetyMargin = safetyTimeMargin;
                    behindVehicle = behindVehicle;
                else
                    behindVehicleSafetyMargin = 1000;
                    behindVehicle = [];
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
