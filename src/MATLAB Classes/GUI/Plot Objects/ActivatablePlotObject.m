classdef (Abstract) ActivatablePlotObject < handle
    %ACTIVATABLEPLOTOBJECT A basic class for all plotted objects that could
    %be activated to allow showing them
    %   Detailed explanation goes here
    properties
        Active (1,1) logical = false   % allows the object to be shown
    end
        
    methods      
        %% setters
        function set.Active(obj, active)
            % trigger visibility when property value is changed
            
            if obj.Active ~= active
                obj.setVisibility(active);
                obj.Active = active;
            end
        end
        
    end
           
    methods (Access = protected)
        function initialize(obj, active)
            % init plot object
            
            % set visibilty through active
            obj.Active = active;
            obj.setVisibility(active);
        end
   
    end
    
    methods (Abstract)
        % Every plot object needs an update function for changing the
        % appereance
        update(obj) 
    end
    
    methods (Access = protected, Abstract) 
        % Every plot object needs an function to change visiblity
        setVisibility(obj, visible)
    end
end
