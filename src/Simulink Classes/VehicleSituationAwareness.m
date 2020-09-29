classdef VehicleSituationAwareness < matlab.System & handle & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
    % The Situation awareness component processes the perceived data in order to assess the emergency case of the vehicle and its relative situation according to the other vehicles around.
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
    end
    
    methods
        % Constructor
        function obj = VehicleSituationAwareness(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.vehicle = evalin('base',strcat('Vehicle',int2str(obj.Vehicle_id)));
        end
        
        
        
        
        
        
        
        
        
        
        
        function [leaderDistance,emergencyCase] = stepImpl(obj, FrontSensorData)
            %This block shouldn't run if the vehicle has reached its
            %destination
            if obj.vehicle.pathInfo.destinationReached
                emergencyCase = -1;
                leaderDistance = -1;
            else
                % Output 1: distance to the leading vehicle
                leaderDistance = FrontSensorData;
                
                % Output 2: Emergency case signal
                try
                    emergencyCase=obj.determineEmergencyCase(obj.vehicle,FrontSensorData);
                catch
                    emergencyCase=obj.determineEmergencyCase(obj.vehicle,FrontSensorData);
                end
            end
        end
        
        
        
        
        
        
        
        
        function emergencyCase = determineEmergencyCase(~, car, frontDistance)
            if car.status.emergencyCase == 3
                %If collision happens Emergency Case stays as 3
                car.status.emergencyCase = 3;
                emergencyCase=3;
                
            elseif frontDistance > car.sensors.frontSensorRange
                
                % Level 0 = Safe
                car.status.emergencyCase = 0;
                emergencyCase =0;
                
            elseif frontDistance >80
                
                % Level 1 = Car far in front
                car.status.emergencyCase = 1;
                emergencyCase=1;
                
            elseif frontDistance > car.sensors.AEBdistance
                
                % Level 2 = Car close infront
                car.status.emergencyCase = 1;
                emergencyCase=1;
                
            elseif frontDistance >0
                
                % Level 2 = emergency brake
                car.status.emergencyCase = 2;
                emergencyCase=2;
                
            elseif frontDistance <0
                % Level BUG
                car.status.emergencyCase = 2;
                emergencyCase=2;
                
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
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename('class'));
        end
        
        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(mfilename('class'));
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
        
        
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
    end
end
