function [newFutureData,vehiclePathPlanner] = gridAStar(vehiclePathPlanner, globalTime,futureData)
            %this function performs a A* search with the grid location
            %objects from obj.gridLocationMap
            %futureData = [carID, coordinates of GL x, y, speed, time, probability]
            %globalTime is the current time of the simulation
            %newFutureData is the newly created FD from this vehicle
            
            %% prepare the search
            if isempty(futureData)
                futureData = [0 0 0 0 0 0];  
            else
                futureData = deleteCollidedFutureDataonGridForLoop(vehiclePathPlanner,futureData);
            end
            futureData = detectBlockingCarsGridForLoop(futureData);
            %update temp goal if necessary
            if vehiclePathPlanner.tempGoalNode == vehiclePathPlanner.vehicle.pathInfo.lastWaypoint
                %if we are at our temp goal
                
                %we need to update oour goal
                vehiclePathPlanner.tempGoalNode = vehiclePathPlanner.vehicle.pathInfo.destinationPoint;
                %we have to repush our goal to the open list to search if
                %the path is still blocked
            end
            carID = vehiclePathPlanner.vehicle.id;
            %first we will set up our current position as a field
            openList = containers.Map();
            closedList = containers.Map();
            %add your location to the open list
            curPos = vehiclePathPlanner.vehicle.pathInfo.lastWaypoint;
            curPos = vehiclePathPlanner.Map.waypoints(curPos,:);
            curGridPos = vehiclePathPlanner.Map.bogMap.world2grid([curPos(1)-vehiclePathPlanner.Map.xOffset,-curPos(3)-vehiclePathPlanner.Map.yOffset]);
            curKey = append( num2str(curGridPos(1)),",",num2str(curGridPos(2)) );
            startKey = curKey;
            curGL = vehiclePathPlanner.Map.gridLocationMap(curKey);
            curGL.gValue = globalTime;
            curGL.speedVector(carID) = vehiclePathPlanner.vehicle.dynamics.speed;
            openList(curKey) = curGL;
            
            %get key of goal node
            curPos = vehiclePathPlanner.Map.waypoints(vehiclePathPlanner.tempGoalNode,:);
            goalPos = vehiclePathPlanner.Map.bogMap.world2grid([curPos(1)-vehiclePathPlanner.Map.xOffset,-curPos(3)-vehiclePathPlanner.Map.yOffset]);%TODO move tempgoal init to a new init function
            goalKey = append( num2str(goalPos(1)),",",num2str(goalPos(2)) );
            goalCoordinates = str2num(goalKey);
            
            %check acceleration
            checkForAccelerationPhase(vehiclePathPlanner);
            
            %% start searching
            while ~isempty(openList.keys)
                %as long as open list is not empty
                %search for min f in open list
                
                olKeys = openList.keys;
                curKey = olKeys{1};
                curGL = openList(curKey); %preassign first entry
                olKeys = olKeys(1,2:end);
                for k = olKeys
                    newGL = openList(k{1,1});
                    if newGL.fValue < curGL.fValue
                        curGL = newGL;
                        curKey = k{1,1};
                    end
                end
                %now remove the gl from the open list
                openList.remove(curKey);
                % calculate time over curGL
                [nextSpeed,travelTime] = checkACCOnGrid(vehiclePathPlanner,curGL,curGL.speedVector(carID));
                curGL = correctTimeWithFD(carID,nextSpeed,vehiclePathPlanner.simSpeed,travelTime,curGL,futureData);
                %now we can push the current grid location to the closed list
                closedList(curKey) = curGL;  
                %% for every successor
                [successors,lastNodeNR] = getSuccessors(curGL,curGL.edgeStart);
                %now check every successor
                for succKey = successors    
                    %test for block or closed list
                    if closedList.isKey(succKey) || isSuccBlocked(succKey,futureData,curGL.gValue + travelTime)
                        %if the successor is already in the closed list or
                        %blocked we can ignore it
                        continue;
                    end 
                    succGL = vehiclePathPlanner.Map.gridLocationMap(succKey);
                    %% regular search step
                    %if we have yet to rech the goal, we continue the search
                    succGL = getGLCosts(carID,curGL.speedVector(carID),curGL.timeVector(carID),vehiclePathPlanner.simSpeed,vehiclePathPlanner.vehicle.dynamics.maxSpeed,curGL.gValue,curGL.totalDistance,curGL.distance,succGL,goalCoordinates);
                    %% if we found the goal
                    if strcmp(succKey , goalKey)
                        %we found the goal node
                        %set parent
                        succGL.parent = curKey;
                        %mark as blocked
                        succGL.deviation = succGL.deviation * -1;
                        %push it to closed list
                        closedList(succKey) = succGL;
                        %build path
                        [newFutureData,newPath] = gridBuildPath(carID, closedList, goalKey, startKey);
                        vehiclePathPlanner.vehicle.pathInfo.path = newPath;
                        
                        return;
                    end
                    %% test the rest                    
                    if openList.isKey(succKey) && openList(succKey).fValue <= succGL.fValue
                        %if we already have a shorter way to go on succGL,
                        %we dont need to push
                        continue;
                    else
                        %set parent
                        succGL.parent = curKey;
                        %set start of edge
                        succGL.edgeStart = lastNodeNR;
                        %add to open list
                        openList(succKey) = succGL;
                    end                    
                end                              
            end
            %% no path was found
            %if we reach this code, no path was found
            %try to update to a closer node
            if newGoalNodeGrid(vehiclePathPlanner,futureData)
                [newFutureData,vehiclePathPlanner] = gridAStar(vehiclePathPlanner,globalTime,futureData);
                return;
            else
                %we cant reach any node from now
                disp(["No possible path was found from vehicle " num2str(carID)])
                vehiclePathPlanner.stopVehicle();
                newFutureData = [];
                return;
            end
        
        end