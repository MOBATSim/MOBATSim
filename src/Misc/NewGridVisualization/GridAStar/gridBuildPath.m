function [newFD,newPath] = gridBuildPath(carID, closedList, goalKey, startKey)
            curGL = closedList(goalKey);
            curKey = goalKey;
            newPath = [];
            newFD = [];
            while ~strcmp(curKey,startKey)
                if curGL.parent == 0
                    %if we searched and there was no parent assigned, we
                    %made a mistake somewhere
                    disp(append("Error in pathbuilding with vehicle ", num2str(carID)) )
                    newFD = [];
                    newPath = [];
                    return;
                end
                
                %assign cur GL to path
                if curGL.nodeNR ~= 0
                    newPath = [newPath, curGL.nodeNR];
                end
                %build future data
                %FD [carID, coordinates of GL x, y, speed, time, probability]
                newFD = [newFD; [carID, curGL.coordinates, curGL.speedVector(carID), curGL.gValue, curGL.deviation]];
                %got to the parent
                curKey = curGL.parent;
                curGL = closedList(curKey);                
            end
            %we have built FD and path now
            %we need to flip path to be in order from start to goal
            newPath = fliplr([newPath, closedList(startKey).nodeNR]);
        end

