classdef VehicleDrivingMode_Qihang < matlab.System & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
    % Behavioral Planner - Vehicle determines its driving mode.
    %
    % NOTE: When renaming the class name Untitled2, the file name
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
        function obj = VehicleDrivingMode(varargin)
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
        
        
        
        
        
        
        
        function [SpeedReference, DistanceReference,LeadSpeed, DrivingMode, Dist2Stop,switch_flag] = stepImpl(obj,LeaderSpeed,LeaderDistance,emergencyCase)
            %This block shouldn't run if the vehicle has reached its
            %destination
            if obj.vehicle.pathInfo.destinationReached
                SpeedReference=0;
                DistanceReference =-1;
                LeadSpeed = -1;
                DrivingMode = 1;
                Dist2Stop = -1;
                switch_flag=0;
            else
               switch_flag = switch_decision(obj,LeaderSpeed,LeaderDistance);
                
                %Output 4: Driving mode
                if(emergencyCase == 0)
                    DrivingMode = 1;
                elseif(emergencyCase == 1)
                    DrivingMode = 2;
                elseif(emergencyCase == 2)
                    if (obj.vehicle.dynamics.speed - LeaderSpeed)>0
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
                
            end
        end
        
        %% helper function
        function switch_flag=switch_decision(obj,LeaderSpeed,LeaderDistance)
            switch_flag=0;
            if(~isempty(obj.vehicle.pathInfo.currentTrajectory))
            if(obj.vehicle.pathInfo.currentTrajectory(3,1)==0)
                %straight lane
                if(obj.vehicle.dynamics.speed - LeaderSpeed)>0
                    switch_flag = 1;
                end
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
            flag = true;
        end
        
        function [out,out2,out3,out4,out5,out6] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = 'double';
            out2 = 'double';
            out3 = 'double';
            out4 = 'double';
            out5 = 'double';
            out6 = 'double';
            
            
            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end
        
        function [out,out2,out3,out4,out5,out6] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;
            
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
        
        function [out,out2,out3,out4,out5,out6] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
            out6 = true;
            
            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
        
        function [out,out2,out3,out4,out5,out6] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 1];
            out2 = [1 1];
            out3 = [1 1];
            out4 = [1 1];
            out5 = [1 1];
            out6 = [1 1];
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
    end
end
