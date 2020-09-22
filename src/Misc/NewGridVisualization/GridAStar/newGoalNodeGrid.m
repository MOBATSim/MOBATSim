function possible = newGoalNodeGrid(obj,futureData)
            %try to update obj.tempGoalNode to a node that is not blocked
            %FD [carID, coordinates of GL x, y, speed, time, deviation]
            path = obj.vehicle.pathInfo.path;
            for i = 3 : length(path)
                %test every node if it is blocked (3 is the next node)
                %we go on our previous path until we reach a blocked one
                coords = obj.Map.waypoints(path(i),:);
                coords = obj.mapBOG.world2grid([coords(1)-obj.xOffset,-coords(3)-obj.yOffset]);
                if ~isempty(futureData(   futureData(:,2) == coords(1) & futureData(:,3) == coords(2) &  futureData(:,6)<0  ))
                    %if we have encountered a block, we need to see if it
                    %is reachable
                    i = i-1;
                    if i > 2
                        obj.tempGoalNode = path(i);
                        possible = true;                      
                    else
                        possible = false;
                    end
                    return;
                end
            end
            possible = false;
        end