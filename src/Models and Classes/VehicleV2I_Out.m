classdef VehicleV2I_Out < matlab.System & handle & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
    % Sends requests to the AIM (implemented inside Infrastructure) to receive go/stop commands for crossing intersections.
    %
    
    % Contributors: Fabian Hart, Mustafa Saraoglu, Johannes Pintscher
    
    % Public, tunable properties
    properties
        Vehicle_id
    end
    
    % Pre-computed constants
    properties(Access = private)
        vehicle
        futureData
        atIntersection % [IntersectionID IntersectionNode]
        % Intersection Zone:
        % x -> Not in Intersection
        % d -> Decision Node
        % i -> Incoming Node
        % p -> Passing (In Conflict Zone)
        % o -> Outgoing Node
    end
    
    methods
        % Constructor
        function obj = VehicleV2I_Out(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.vehicle = evalin('base', "Vehicles(" + obj.Vehicle_id + ")");
            obj.atIntersection = [0 "x"];
        end
        
        
        function V2Idata = stepImpl(obj,waypointReached)
            %This block shouldn't run if the vehicle has reached its
            %destination
            if obj.vehicle.pathInfo.destinationReached
                V2Idata = [0 0];
            else
                % When waypoints are reached, check if arriving at an intersection 
                % (so that communication with AIM can be established)
                %Output the V2I data
                if waypointReached
                    V2Idata = obj.determineIntersectionZone(obj.vehicle,obj.vehicle.pathInfo.lastWaypoint);
                else
                    V2Idata = [0 0];
                end
                
            end
        end
        
        function V2Idata = determineIntersectionZone(obj,car,current_point)
            % Vehicle signals the AIM the intersection zone it is in
            map = car.map;
            
            %% The vehicle is recently arriving -> Intersection Starting Nodes
            if nnz(cat(1,map.AIM_Units.startingNodes)==current_point)
                intersectionID = find(any(cat(1,map.AIM_Units.startingNodes)==current_point,2));
                car.status.atIntersection = [intersectionID "d"];
                
                if obj.vehicle.V2IdataLink==1
                    V2Idata = [intersectionID 1];
                end
                
                %% The vehicle has reached the braking point -> Intersection Braking Nodes
            elseif nnz(cat(1,map.AIM_Units.brakingNodes)==current_point) % car reaches Braking Point
                intersectionID = find(any(cat(1,map.AIM_Units.brakingNodes)==current_point,2));
                car.status.atIntersection = [intersectionID "i"];
                
                if obj.vehicle.V2IdataLink==1
                    V2Idata = [intersectionID 2];
                end
                
                %% The vehicle has reached the stopping point before the Conflict Zone -> Intersection Stopping Nodes
            elseif nnz(cat(1,map.AIM_Units.stoppingNodes)==current_point) % car reaches the Stopping Point
                intersectionID = find(any(cat(1,map.AIM_Units.stoppingNodes)==current_point,2));
                car.status.atIntersection = [intersectionID "p"];
                V2Idata = [intersectionID 3];
                
                %% The vehicle leaves the Conflict Zone thus the Intersection -> Intersection Leaving Nodes
            elseif nnz(cat(1,map.AIM_Units.leavingNodes)==current_point)>0 % car leaves leaves the intersection
                intersectionID = find(any(cat(1,map.AIM_Units.leavingNodes)==current_point,2));
                if obj.vehicle.V2IdataLink==1
                    V2Idata = [intersectionID 4];
                end
                car.status.atIntersection = [0 "x"];
            else
                V2Idata = [0 0];
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
            icon = matlab.system.display.Icon("MOBATSIM-Icon-Set_6- V2l.png");
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
        %% Simulink customization functions
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
            flag = false;
        end
        
        function out = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 2];
            
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
        
    end
end
