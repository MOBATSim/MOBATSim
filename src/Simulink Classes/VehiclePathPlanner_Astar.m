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
        function [FuturePlan, waypointReached] = stepImpl(obj,OtherVehiclesFutureData)
            %This block shouldn't run if the vehicle has reached its
            %destination
            %% choose path
            if obj.vehicle.pathInfo.destinationReached
                FuturePlan = obj.vehicle.decisionUnit.futureData;
                waypointReached=1;
            else
                % Firstly check if the vehicle has reached its destination so it stops.
                obj.vehicle.checkifDestinationReached();
                
                % Then check if the vehicle has completed its route but still needs to reach its destination
                if obj.vehicle.pathInfo.routeCompleted == 1 && obj.vehicle.pathInfo.destinationReached == 0
                    
                    % Time Stamps are logged when waypoints are reached
                    obj.vehicle.dataLog.timeStamps = [obj.vehicle.dataLog.timeStamps;[obj.vehicle.pathInfo.lastWaypoint get_param(obj.modelName,'SimulationTime')]];
                    
                    % Build the future plan by deriving the next routes and building the path
                    %Output 1: Future plan of the vehicle
                    FuturePlan = obj.findNextRoute(obj.vehicle, obj.vehicle.pathInfo.lastWaypoint, obj.vehicle.pathInfo.destinationPoint,get_param(obj.modelName,'SimulationTime'),OtherVehiclesFutureData);
                    obj.vehicle.setStopStatus(false);
                else
                    % If the vehicle is still on its route then the future data stays the same
                    %Output 1: Future plan of the vehicle
                    FuturePlan = obj.vehicle.decisionUnit.futureData;
                end
                
                % Check if crossroad
                obj.crossroadCheck(obj.vehicle);
                
                
                %Output 2: Waypoint Reached enabler
                if obj.vehicle.pathInfo.routeCompleted
                    waypointReached =1;
                else
                    waypointReached =0;
                end
            end
            %% Grid path generation
            if mod(get_param(obj.modelName,'SimulationTime'),0.2) == 0
                %plotting can decrease performance, so dont update to often
                obj.vehicle.pathInfo.BOGPath = generate_BOGPath(obj.Map,obj.vehicle.pathInfo.path,obj.vehicle.id);
            end
        end
    end
    

end