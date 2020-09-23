classdef VehicleCommunication_v2v < matlab.System & handle & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
    % This module gets the futureData(other vehicles' predictions) which has a V2V connection with this vehicle.
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
        Vehicles
    end

    methods
        % Constructor
        function obj = VehicleCommunication_v2v(varargin)
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
        
        
        
        
        
        
        
        
        
        
        
        
        
        function [OtherVehiclesFutureData,LeaderSpeed] = stepImpl(obj,CommunicationID)
            %This block shouldn't run if the vehicle has reached its
            %destination
            if obj.vehicle.pathInfo.destinationReached
                % Output1: Get lead vehicle speed if there is any
                LeaderSpeed = -1;               
                % Output2: Collect Future Data
                OtherVehiclesFutureData = -1;
                
            else
                % Output1: Get lead vehicle speed if there is any
                LeaderSpeed = obj.getLeaderSpeedifExists(obj.Vehicles,CommunicationID);
                
                % Output2: Collect Future Data
                OtherVehiclesFutureData = obj.CollectFutureData(obj.vehicle, obj.Vehicles);
            end
        end
        
        
        
        
        
        
        
        
        
        
        
        
        function futureData = CollectFutureData(~,car, Vehicles)
            i = 1:length(Vehicles);
            i = i(car.V2VdataLink==1); % Remove the vehicles that don't have V2V connection to the car
            i(car.id)=[]; % Remove the car with the same id

            futureData =cat(1,cat(1,[Vehicles(i).decisionUnit]).futureData);

        end
        
        function LeaderSpeed = getLeaderSpeedifExists(~, Vehicles,CommunicationID)
            
            if CommunicationID~=0
                LeaderSpeed = Vehicles(CommunicationID).dynamics.speed;
            else
                LeaderSpeed = 0;
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
            icon = matlab.system.display.Icon("V2V.png");
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
            out = [8000 6];
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
            out = false;
            out2 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end

        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
    end
end
