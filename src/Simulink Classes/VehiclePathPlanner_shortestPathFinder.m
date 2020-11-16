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
        
        function FuturePlan = findPath(obj,~)
            Path = obj.vehicle.map.get_shortest_path(obj.vehicle.pathInfo.lastWaypoint, ...
            obj.vehicle.pathInfo.destinationPoint);
           
            obj.vehicle.pathInfo.path = Path;
            % FuturePlan Format of Shortest PathPlanner: nx6 -> [id RouteID 0 0 0 -1] 
            %We don't have the timing information so we set zeros to the last three columns

            RouteID = [];
            % We need the RouteIDs so we derive it from the Path 
            % Waypoints(Nodes) are always one more than Routes(Edges) so "i" until length(Path)-1
            for i = 1:(length(Path)-1) 
                RouteID = [RouteID; obj.vehicle.map.getRouteIDfromPath([obj.vehicle.pathInfo.path(i)  obj.vehicle.pathInfo.path(i+1)])];
            end
            %The last three columns are to fit the data according to other PathPlanner FutureDatas
            FuturePlan = [(ones(1,size(Path,2)-1).*obj.vehicle.id)' RouteID zeros(size(Path,2)-1,3) (ones(1,size(Path,2)-1)*-3)'];
            
        end
        
        function futureData = deleteCollidedVehicleFutureData(~,futureData)
            % TODO: fix the architecture for this abstract method later
        end
    end
    
end
