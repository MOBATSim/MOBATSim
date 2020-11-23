classdef VehicleNetwork_Out < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.CustomIcon
    % The current values of position, speed and path plans of the vehicle are packed by the V2V communication output component and shared with the rest of the network of connected vehicles. 
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.
    
    % Public, tunable properties
    properties
        Vehicle_id
    end
    
    properties(DiscreteState)
        
    end
    
    % Pre-computed constants
    properties(Access = private)
        vehicle
    end
    
    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.vehicle = evalin('base',strcat('Vehicle',int2str(obj.Vehicle_id)));
            
        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon("V2V.png");
        end
        
        
        
        
        
        
        
        
        
        function stepImpl(obj,FuturePlan)
            obj.vehicle.decisionUnit.futureData = FuturePlan;
            
        end
    end
    
    
    
    
    
    
    
    methods(Static, Access = protected)
        %% Standard Simulink Output functions
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
        
        
        function flag = isInputSizeLockedImpl(~,~)
            % Return true if input size is not allowed to change while
            % system is running
            flag = false;
        end
        
    end
end
