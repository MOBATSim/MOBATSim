classdef VehiclePathPlanner_Astar < VehiclePathPlanner
    % VehiclePathPlanner_A* Inherits the VehiclePathPlanner. Generates a path to reach the destination waypoint.
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.
    
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
        function obj = VehiclePathPlanner_Astar(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    methods (Access = protected)
        
        function setupImpl(obj)
            setupImpl@VehiclePathPlanner(obj); % Inherit the setupImpl function of the Superclass @VehiclePathPlanner
        end
        
        %function [FuturePlan, waypointReached] = stepImpl(obj,OtherVehiclesFutureData)
        % This part is defined in the SuperClass so don't edit it unless you want to override. 
        %end
        
        function FuturePlan = findPath(obj,OtherVehiclesFutureData)
            FuturePlan = obj.findNextRoute(obj.vehicle, obj.vehicle.pathInfo.lastWaypoint, ...
            obj.vehicle.pathInfo.destinationPoint,get_param(obj.modelName,'SimulationTime'),OtherVehiclesFutureData);
        end
    end
    
    
end