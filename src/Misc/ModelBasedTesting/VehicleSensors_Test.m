classdef VehicleSensors_Test < matlab.System & handle & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon & matlab.system.mixin.SampleTime
    % Vehicle Sensor Block: Ideal radar implementation in MOBATSim.
    %
    % NOTE: When renaming the class name Untitled, the file name
    % and constructor name must be updated to use the class name.
    %
    % This template includes most, but not all, possible properties, attributes,
    % and methods that you can implement for a System object in Simulink.

    % Public, tunable properties
    properties
        Vehicle_id
        Tsample = 0.01;
        OffsetTime =0;
    end

    % Public, non-tunable properties
    properties(Nontunable)

    end

    properties(DiscreteState)

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
        
        
        function [V2VcommIDs,VehiclePosition,StopPosition,stopAt,ObjectinFront,StatusEmergencyCase,destinationReached,Speed] = stepImpl(obj)
            %This block shouldn't run if the vehicle has reached its
            %destination
            
            %get some dynamic values in the model
            destinationReached = obj.vehicle.pathInfo.destinationReached;
            StatusEmergencyCase = obj.vehicle.status.emergencyCase;
            stopAt = obj.vehicle.pathInfo.stopAt;
            Speed = obj.vehicle.dynamics.speed; 
            
            %Get the position of the vehicle and the position to stop
            VehiclePosition = obj.vehicle.dynamics.position;         
            if stopAt ~= 0
                StopPosition = obj.vehicle.map.get_coordinates_from_waypoint(stopAt);
            else
                StopPosition = obj.vehicle.dynamics.position;
            end
            
            
            if obj.vehicle.pathInfo.destinationReached
                % 2 Outputs: Vehicle in front id, Distance to the vehicle in front
                V2VcommIDs = -1;
                ObjectinFront = -1;
            
            else
                % Update the sensor data of the vehicle if it is not on halt
                if obj.vehicle.status.stop ==0 && ~isempty(obj.vehicle.pathInfo.currentTrajectory)
                    obj.updateSensorData(obj.vehicle,obj.Vehicles);
                end
                
                % 2 Outputs: Vehicle in front id, Distance to the vehicle in front
                V2VcommIDs = obj.vehicle.sensors.vehicleInFrontId;
                ObjectinFront = obj.vehicle.sensors.frontDistance;
            end
            
        end
        
        
        
        
        
        
        
        
        
        
        function updateSensorData(~,car, Vehicles)
            
            id_distance = [];
             
            i = 1:length(Vehicles);
            i(car.id)=[]; % Remove the car with the same id
                        
            VehiclesOnSameRoute=Vehicles(car.pathInfo.currentRoute == [cat(1,Vehicles([i]).pathInfo).currentRoute]);
            
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
            
            %check if it is ahead
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
                    car.sensors.frontDistance = min(id_distance(:,2));
                    rowId = id_distance(:,2)==car.sensors.frontDistance;
                    car.sensors.vehicleInFrontId = id_distance(rowId,1);
                    return;
                else
                    car.sensors.frontDistance = 1000; %we can also assign an arbitrary value
                    car.sensors.vehicleInFrontId = 0;
                end
                
            else
                car.sensors.frontDistance = 1000; %we can also assign an arbitrary value
                car.sensors.vehicleInFrontId = 0;
                return;
            end
        end

%% Standard Simulink Output functions
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
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

        function ds = getDiscreteStateImpl(obj)
            % Return structure of properties with DiscreteState attribute
            ds = struct([]);
        end

        function flag = isInputSizeLockedImpl(obj,index)
            % Return true if input size is not allowed to change while
            % system is running
            flag = true;
        end

        function [out,out2,out3,out4,out5,out6,out7,out8] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [1 1];
            out2 = [1 3];
            out3 = [1 3];
            out4 = [1 1];
            out5 = [1 1];
            out6 = [1 1];
            out7 = [1 1];
            out8 = [1 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out,out2,out3,out4,out5,out6,out7,out8] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = 'double';
            out2 = 'double';
            out3 = 'double';
            out4 = 'double';
            out5 = 'double';
            out6 = 'double';
            out7 = 'boolean';
            out8 = 'double';

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out,out2,out3,out4,out5,out6,out7,out8] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;
            out7 = false;
            out8 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out,out2,out3,out4,out5,out6,out7,out8] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
            out6 = true;
            out7 = true;
            out8 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
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

        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(mfilename('class'));
        end
    end
end
