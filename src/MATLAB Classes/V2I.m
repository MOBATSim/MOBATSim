classdef V2I < handle
    %V2V Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dataLinkV2I
        id
        map
    end
    
    methods
        function obj = V2I(id,dataLinkV2I,map)
            obj.id  = id;
            obj.dataLinkV2I = dataLinkV2I;
            obj.map = map;
        end %Constructor
        
        function carReachesCrossroadV2I(obj,car,starting_point,global_timesteps,crossroadId)
            if obj.dataLinkV2I==1                
                obj.map.crossroadUnits(crossroadId).carReachesCrossroad(car,starting_point,global_timesteps);
            else
                fallbackVehicle = car.V2V.checkV2Ifallback(car); % TODO: check, method are not existing
                if ~isempty(fallbackVehicle)
                    
                    fallbackVehicle.V2I.carReachesCrossroadV2I(car,starting_point,global_timesteps,crossroadId)
                end
            end
        end
        
        function carReachesBreakingPointV2I(obj,car,breakingPoint,global_timesteps,crossroadId)
            if obj.dataLinkV2I==1
                obj.map.crossroadUnits(crossroadId).carReachesBreakingPoint(car,breakingPoint,global_timesteps);
            else
                fallbackVehicle = car.V2V.checkV2Ifallback(car); % TODO: check, method are not existing
                if ~isempty(fallbackVehicle)
                    % if car is not registered yet in the IM yet, it has to
                    % be done before
                    if ~any(obj.map.crossroadUnits(crossroadId).arrivingQueue(:,1)==car.id)
                        
                        %get startingNode to register vehicle correctly
                        arrivingDirection = find(breakingPoint == obj.map.crossroadUnits(crossroadId).breakingNodes);
                        startingNode = obj.map.crossroadUnits(crossroadId).startingNodes(arrivingDirection);
                        
                        fallbackVehicle.V2I.carReachesCrossroadV2I(car,startingNode,global_timesteps,crossroadId)
                    end
                    
                    fallbackVehicle.V2I.carReachesBreakingPointV2I(car,breakingPoint,global_timesteps,crossroadId)
                end
                
            end
        end
        
        function carLeavesCrossroadV2I(obj,car,global_timesteps,crossroadId)
            if obj.dataLinkV2I==1
                obj.map.crossroadUnits(crossroadId).carLeavesCrossroad(car,global_timesteps);
            else
                fallbackVehicle = car.V2V.checkV2Ifallback(car); % TODO: check, method are not existing
                if ~isempty(fallbackVehicle)
                    
                    
                    fallbackVehicle.V2I.carLeavesCrossroadV2I(car,global_timesteps,crossroadId)
                end
                
            end
        end
        
        function stopFlag = requestRightOfWayV2I(obj,car,crossroadId,global_timesteps)
            
            if obj.dataLinkV2I==1
                
                stopFlag = obj.map.crossroadUnits(crossroadId).requestRightOfWay(car.id);
            else
                fallbackVehicle = car.V2V.checkV2Ifallback(car); % TODO: check, method are not existing
                if ~isempty(fallbackVehicle)
                    
                    % if car is not registered yet in the IM yet, it has to
                    % be done before
                    if ~any(obj.map.crossroadUnits(crossroadId).arrivingQueue(:,1)==car.id)
                        
                        %get startingNode and breakingNode to register vehicle correctly
                        breakingPoint = car.pathInfo.path(1);
                        arrivingDirection = find(breakingPoint == obj.map.crossroadUnits(crossroadId).breakingNodes);
                        startingNode = obj.map.crossroadUnits(crossroadId).startingNodes(arrivingDirection);
                        
                        fallbackVehicle.V2I.carReachesCrossroadV2I(car,startingNode,global_timesteps,crossroadId)
                        fallbackVehicle.V2I.carReachesBreakingPointV2I(car,breakingPoint,global_timesteps,crossroadId)
                    end
                    
                    
                    
                    stopFlag = fallbackVehicle.V2I.requestRightOfWayV2I(car,crossroadId);
                else
                    % if theres no fallback vehicle near the crossroad
                    stopFlag = 1;
                end
                
            end
        end    
        
    end
end