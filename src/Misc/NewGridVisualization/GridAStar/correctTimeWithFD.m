function curGL = correctTimeWithFD(carID,nextSpeed,simSpeed,travelTime,curGL,futureData)
            %to know how long we actually took, we need to keep future data
            %in mind
            %FD [carID, coordinates of GL x, y, speed, time, deviation]
            %% check for other cars on current grid location
            %we want to know how long it takes to reach succGL
            currentFutureData = futureData(futureData(:,2) == curGL.coordinates(1) & futureData(:,3) == curGL.coordinates(2),:);
            if ~isempty(currentFutureData)
                curGL.timeVector(carID) = travelTime + curGL.gValue;
                %we now need to calculate probability
                %for that we use pdf
                for j = 1 : size(currentFutureData,1)
                    id = currentFutureData(j,1);
                    x = [curGL.timeVector(carID)-curGL.deviation,curGL.timeVector(carID)+curGL.deviation];
                    mu = currentFutureData(j,5);
                    sigma = abs(currentFutureData(j,6));                    
                    %assign the pbb value to the car id entry in timeVcetor
                    %using normal distribution
                    pbb = normcdf(x,mu,sigma);
                    curGL.probabilityVector(id) = pbb(2)-pbb(1);
                    curGL.speedVector(id) = currentFutureData(j,4);
                end
                %% now use the probability to alter travel time
                otherCars = currentFutureData(:,1)';
                for car = otherCars
                    if curGL.probabilityVector(car) > 0.3 && curGL.speedVector(car) < nextSpeed
                        %% disturbing car on same route
                        nextSpeed = curGL.speedVector(car);
                        travelTime = (1/ simSpeed) *curGL.distance * (1/ nextSpeed);% 
                    end
                end
                curGL.timeVector(carID) = travelTime;
                curGL.speedVector(carID) = nextSpeed;
            end
        end