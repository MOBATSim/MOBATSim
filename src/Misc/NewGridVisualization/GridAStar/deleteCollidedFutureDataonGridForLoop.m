function futureData = deleteCollidedFutureDataonGridForLoop(obj,futureData)
            %deletes future data of vehicles that will not move because of collision
            %FD [carID, coordinates of GL x, y, speed, time, deviation]
            otherCars = 1:10;
            otherCars = otherCars(otherCars ~= obj.vehicle.id);
            vehicles = obj.Map.Vehicles;
            for car = otherCars
                if vehicles(car).status.collided
                    %remove every entry with the collided car from FD
                    futureData = futureData(futureData(:,1)~=car,:);
                    %% block the start node of the crash
%                     coords = obj.Map.waypoints(vehicles(car).pathInfo.lastWaypoint,:);
%                     coords = obj.mapBOG.world2grid([coords(1)-obj.xOffset,-coords(3)-obj.yOffset]);                    
%                     futureData = [futureData;[car , coords , 0, 0, -1]];
                    %% block the future node of the crash
                    coords = obj.Map.waypoints(vehicles(car).pathInfo.path(2),:);
                    coords = obj.mapBOG.world2grid([coords(1)-obj.xOffset,-coords(3)-obj.yOffset]);
                    futureData = [futureData;[car , coords , 0, 0, -1]];
                end                
            end
        end