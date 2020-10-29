classdef Pedestrian < handle
    %PEDESTRIAN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        position
        orientation
        speed
        map
        path
        count
    end
    
    methods
        function obj = Pedestrian(start,dest,speed,map)
            %PEDESTRIAN Construct an instance of this class
            %   Detailed explanation goes here
            obj.position = start(1:2);            
            obj.speed = speed;
            obj.orientation = start(3)
            obj.map=map
            vMap = validatorOccupancyMap;
            vMap.Map = map;
            planner = plannerHybridAStar(vMap, 'MinTurningRadius', 2.5);

            entrance = [start(1)*2, (start(2)+200)*2, start(3)];
            packagePickupLocation = [dest(1)*2, (dest(2)+200)*2, dest(3)];
            route = plan(planner, entrance, packagePickupLocation);
            route = route.States;
            obj.path=[round(route(:,1)/2), round(route(:,2)/2-200),route(:,3)]
            obj.count=1
            % 3/2*pi
        end
        
        function newPosition = walk(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            if obj.count==size(obj.path,1)
                newPosition=obj.position;
            else
                obj.count=obj.count+1
                newPosition = obj.path(obj.count,1:2)
                obj.position=newPosition;
                obj.orientation=obj.path(obj.count,3)
            end
        end
    end
end

