classdef (Abstract) ActivatablePlotObject < handle
    %ACTIVATABLEPLOTOBJECT A basic class for all plotted objects that could
    %be activated to allow showing them
    %   Detailed explanation goes here
    properties
        Active          % allows the object to be shown
    end
        
    methods (Access = protected) 
        % Every plot object needs an function to change visiblity
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
