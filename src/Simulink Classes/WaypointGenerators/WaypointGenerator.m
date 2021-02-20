classdef WaypointGenerator < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime & matlab.system.mixin.CustomIcon
    %WAYPOINTGENERATOR Superclass for waypoint generators
    %   Detailed explanation goes here
    
    properties
        Vehicle_id
    end
    
    % Pre-computed constants
    properties(Access = private)
        vehicle
        map = evalin('base','Map');
        simSpeed = evalin('base','simSpeed');
        modelName = evalin('base','modelName');
    end
    
    methods
        function obj = WaypointGenerator(varargin)
            %WAYPOINTGENERATOR Construct an instance of this class
            %   Detailed explanation goes here
            setProperties(obj,nargin,varargin{:});
        end
    end

    methods(Access = protected)
        function currentRoute = generateCurrentRoute(~,car, path, lastWaypoint)
            
            idx = find(path==lastWaypoint);
            if idx+1<=length(path) % Next Route
                currentRoute = car.map.getRouteIDfromPath([path(idx) path(idx+1)]);
            else % Destination Reached // CurrentRoute stays the same
                currentRoute = car.pathInfo.currentRoute;
            end
        end
        
         function currentTrajectory = generateTrajectoryFromPath(~,car,path)
            % Format of route for vehicle dynamics (translation)
            if (isempty(find((car.map.connections.translation(:,1) == path(1) )&(car.map.connections.translation(:,2)== path(2)), 1 )) == false)
                index = find((car.map.connections.translation(:,1) == path(1) )&(car.map.connections.translation(:,2)== path(2)) );
                currentTrajectory = [car.map.waypoints(car.map.connections.translation(index,1),:);
                    car.map.waypoints(car.map.connections.translation(index,2),:);
                    zeros(1,3);
                    zeros(1,3)];
            end
            % Format of route for vehicle dynamics (curves)
            if (isempty(find((car.map.connections.circle(:,1) == path(1) )&(car.map.connections.circle(:,2)== path(2)), 1 )) == false)
                index = find((car.map.connections.circle(:,1) == path(1) )&(car.map.connections.circle(:,2)== path(2)) );
                currentTrajectory = [car.map.waypoints(car.map.connections.circle(index,1),:);
                    car.map.waypoints(car.map.connections.circle(index,2),:);
                    abs(car.map.connections.circle(index,3)),car.map.connections.circle(index,4),car.map.connections.circle(index,6);
                    -sign(car.map.connections.circle(index,3))*ones(1,3)];
            end
        end
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon("WaypointGenerator.png");
        end
    end
end

