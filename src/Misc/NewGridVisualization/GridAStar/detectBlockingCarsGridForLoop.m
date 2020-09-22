function futureData = detectBlockingCarsGridForLoop(obj,futureData)
            %if a car without future data blocks a node, we set the node blocked
            otherCars = 1:10;
            otherCars = otherCars(otherCars ~= obj.vehicle.id);
            vehicles = obj.Map.Vehicles;
            for car = otherCars
                if vehicles(car).pathInfo.destinationReached                    
                    coords = obj.Map.waypoints(vehicles(car).pathInfo.lastWaypoint,:);
                    coords = obj.mapBOG.world2grid([coords(1)-obj.xOffset,-coords(3)-obj.yOffset]);
                    futureData = [futureData;[car , coords , 0, 0, -1]];
                end
            end
        end