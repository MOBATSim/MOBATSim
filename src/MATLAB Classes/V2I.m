classdef V2I < handle
    %V2V Summary of this class goes here
    %   Detailed explanation goes here
    
    properties         
        dataLinkV2I
        id
    end
    
    methods
      function obj = V2I(id,dataLinkV2I)
          obj.id  = id;
          obj.dataLinkV2I = dataLinkV2I;
        end %Constructor
        
        function carReachesCrossroadV2I(obj,car,starting_point,global_timesteps,crossroadId)
           if obj.dataLinkV2I==1
               Map = evalin('base','Map');
               Map.crossroadUnits(crossroadId).carReachesCrossroad(car,starting_point,global_timesteps);
           else
               fallbackVehicle = car.V2V.checkV2Ifallback(car);
               if ~isempty(fallbackVehicle)
                   
                fallbackVehicle.V2I.carReachesCrossroadV2I(car,starting_point,global_timesteps,crossroadId)
               end
            end
        end
        
        function carReachesBreakingPointV2I(obj,car,breakingPoint,global_timesteps,crossroadId)
            Map = evalin('base','Map');
            if obj.dataLinkV2I==1                
                Map.crossroadUnits(crossroadId).carReachesBreakingPoint(car,breakingPoint,global_timesteps);
            else
                fallbackVehicle = car.V2V.checkV2Ifallback(car);
                if ~isempty(fallbackVehicle)
                    % if car is not registered yet in the IM yet, it has to
                    % be done before
                    if any(Map.crossroadUnits(crossroadId).arrivingQueue(:,1)==car.id) == 0
                        
                        %get startingNode to register vehicle correctly
                        arrivingDirection = find(breakingPoint == Map.crossroadUnits(crossroadId).breakingNodes);
                        startingNode = Map.crossroadUnits(crossroadId).startingNodes(arrivingDirection);
                        
                        fallbackVehicle.V2I.carReachesCrossroadV2I(car,startingNode,global_timesteps,crossroadId)
                    end
                    
                    fallbackVehicle.V2I.carReachesBreakingPointV2I(car,breakingPoint,global_timesteps,crossroadId)
                end
                
            end
        end
        
        function carLeavesCrossroadV2I(obj,car,global_timesteps,crossroadId)
             if obj.dataLinkV2I==1
                 Map = evalin('base','Map');
                 Map.crossroadUnits(crossroadId).carLeavesCrossroad(car,global_timesteps);
             else
                 fallbackVehicle = car.V2V.checkV2Ifallback(car);
                 if ~isempty(fallbackVehicle)
                     
                     
                     fallbackVehicle.V2I.carLeavesCrossroadV2I(car,global_timesteps,crossroadId)
                 end
                 
             end
        end
        
        function stopFlag = requestRightOfWayV2I(obj,car,crossroadId,global_timesteps)
            Map = evalin('base','Map');
            stopFlag = 1;
            if obj.dataLinkV2I==1
                
                stopFlag = Map.crossroadUnits(crossroadId).requestRightOfWay(car.id);
            else
                fallbackVehicle = car.V2V.checkV2Ifallback(car);
                if ~isempty(fallbackVehicle)
                    
                    % if car is not registered yet in the IM yet, it has to
                    % be done before
                    if any(Map.crossroadUnits(crossroadId).arrivingQueue(:,1)==car.id) == 0
                        
                        %get startingNode and breakingNode to register vehicle correctly
                        breakingPoint = car.pathInfo.path(1);
                        arrivingDirection = find(breakingPoint == Map.crossroadUnits(crossroadId).breakingNodes);
                        startingNode = Map.crossroadUnits(crossroadId).startingNodes(arrivingDirection);
                        
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
        
        function  timeToReach = getEstimations(obj,vehicle,stoppingNode,ETAcarInFront,paramsPlatooning,global_timesteps)
            timeToReach = vehicle.calculateEstimatedTimeOfArrival(stoppingNode,ETAcarInFront,paramsPlatooning,global_timesteps);
        end
        
        
    end
end