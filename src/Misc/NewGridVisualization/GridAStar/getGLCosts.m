function succGL = getGLCosts(carID,nextSpeed,travelTime,simSpeed,maxSpeed,curG,curTotalDistance,curDistance,succGL,goalCoordinates)
            %this function calculates the cost to get through curGL to succGL            
            %% now calculate the g and f values
            succGL.gValue = curG + travelTime;%we need this much time to go to succGL
            succGL.totalDistance = curTotalDistance + curDistance;%we travelled so much so far
            succGL.speedVector(carID) = nextSpeed;%this will be our speed on this GL
            h = (1/ simSpeed) * (1/maxSpeed) * norm(goalCoordinates - succGL.coordinates);
            succGL.fValue = succGL.gValue + h;
            %% calculate deviation
            %deviation d(speed,totalDistance)
%             x = (maxSpeed*10 + curTotalDistance)/1000;
%             succGL.deviation = travelTime * power(x,2);
            succGL.deviation = 0.01*succGL.gValue;
        end