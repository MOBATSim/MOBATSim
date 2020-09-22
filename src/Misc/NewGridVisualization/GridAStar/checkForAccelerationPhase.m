function checkForAccelerationPhase(vehiclePathPlanner)
            %used to calculate acceleration for future data estimation
            car = vehiclePathPlanner.vehicle;
            maxSpeed = car.dynamics.maxSpeed ;
            currentSpeed = vehiclePathPlanner.vehicle.dynamics.speed ;
            if abs(maxSpeed - currentSpeed) > 1
                if vehiclePathPlanner.accelerationPhase(1) == 0
                    % Neural Network is used to get average acceleration value
                    averageAcceleration = vehiclePathPlanner.simSpeed^2 * NN_acceleration([currentSpeed; maxSpeed-currentSpeed]);
                    accelerationDistance = vehiclePathPlanner.getAccelerationDistance(averageAcceleration, currentSpeed, maxSpeed);
                    vehiclePathPlanner.accelerationPhase = [1,currentSpeed,maxSpeed, accelerationDistance, averageAcceleration];
                end
            end
        end