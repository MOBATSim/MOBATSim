classdef VehicleV2I_Out < matlab.System & handle & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
    % Sends requests to the Infrastructure for crossroads and right of way.
    %
    % NOTE: When renaming the class name Untitled, the file name
    % and constructor name must be updated to use the class name.
    %
    % This template includes most, but not all, possible properties, attributes,
    % and methods that you can implement for a System object in Simulink.
    
    % Public, tunable properties
    properties
        Vehicle_id
    end
    
    % Public, non-tunable properties
    properties(Nontunable)
        
    end
    
    properties(DiscreteState)
        
    end
    
    % Pre-computed constants
    properties(Access = private)
        vehicle
        accelerationPhase;
        futureData
        inCrossroad % [crossroadId crossroadZone]
        % crossroadZone:
        % 1 -> arrivingZone
        % 2 -> stoppingZone
        % 3 -> intersectionZone
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
            
            obj.accelerationPhase =  zeros(1,5);
            obj.inCrossroad = [0 0];
        end
          
        
        function V2Idata = stepImpl(obj,waypointReached)
            %This block shouldn't run if the vehicle has reached its
            %destination
            if obj.vehicle.pathInfo.destinationReached
                V2Idata = [0 0];
            else
                % When waypoints are reached, check for crossroad actions
                %Output the V2I data
                if waypointReached
                    V2Idata = obj.checkCrossroadActions(obj.vehicle,obj.vehicle.pathInfo.lastWaypoint);
                else
                    V2Idata = [0 0];
                end
                
            end
        end      
        
        function V2Idata = checkCrossroadActions(obj,car,current_point)
            % check for crossroad actions
            map = car.map;
            
            if nnz(cat(1,map.crossroadUnits.startingNodes)==current_point)
                crossroadId = find(any(cat(1,map.crossroadUnits.startingNodes)==current_point,2));
                car.decisionUnit.inCrossroad = [crossroadId 1];
                                
                if obj.vehicle.V2IdataLink==1
                    V2Idata = [crossroadId 1];
                end
                
            elseif nnz(cat(1,map.crossroadUnits.brakingNodes)==current_point) % car reaches Braking Point 
                crossroadId = find(any(cat(1,map.crossroadUnits.brakingNodes)==current_point,2));
                car.decisionUnit.inCrossroad = [crossroadId 2];
                
                if obj.vehicle.V2IdataLink==1
                    V2Idata = [crossroadId 2];
                end
                
            elseif nnz(cat(1,map.crossroadUnits.stoppingNodes)==current_point) % car reaches Stopping Point
                crossroadId = find(any(cat(1,map.crossroadUnits.stoppingNodes)==current_point,2));
                car.decisionUnit.inCrossroad = [crossroadId 3];
                V2Idata = [crossroadId 3];
                
            elseif nnz(cat(1,map.crossroadUnits.leavingNodes)==current_point)>0 % car leaves crossroad
                crossroadId = find(any(cat(1,map.crossroadUnits.leavingNodes)==current_point,2));
                if obj.vehicle.V2IdataLink==1
                    V2Idata = [crossroadId 4];   
                end
                car.decisionUnit.inCrossroad = [0 0];
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
            icon = matlab.system.display.Icon("V2I.png");
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
