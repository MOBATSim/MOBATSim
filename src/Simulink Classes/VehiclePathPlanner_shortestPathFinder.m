classdef VehiclePathPlanner_shortestPathFinder < VehiclePathPlanner
    % Path Planner Plans paths.
    %
    % NOTE: When renaming the class name Untitled, the file name
    % and constructor name must be updated to use the class name.
    %
    % This template includes most, but not all, possible properties, attributes,
    % and methods that you can implement for a System object in Simulink.
    
    % Public, tunable properties
    properties
        
    end
    
    % Public, non-tunable properties
    properties(Nontunable)
        
    end
    
    properties(DiscreteState)
        
    end
    
    % Pre-computed constants
    properties(Access = private)

    end
    
    methods
        % Constructor
        function obj = VehiclePathPlanner_shortestPathFinder(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        
        function setupImpl(obj)
            setupImpl@VehiclePathPlanner(obj); % Inherit the setupImpl function of the Superclass @VehiclePathPlanner
        end
        
        function FuturePlan = findPath(obj,OtherVehiclesFutureData)
            Path = obj.vehicle.map.get_shortest_path(obj.vehicle.pathInfo.lastWaypoint, ...
            obj.vehicle.pathInfo.destinationPoint);
           
            obj.vehicle.pathInfo.path = Path;
            FuturePlan = [(ones(1,size(Path,2)).*obj.vehicle.id)' Path' zeros(size(Path,2),3)];
            
        end
    end
    
end
