function [nextSpeed,timeToReach] = checkACCOnGrid(vehiclePathPlanner,currentGL,currentSpeed)
            %nextSpeed = speed on end of edge, timeToReach = exit time of edge            
            
            distance = currentGL.distance;
            
            %% check for acceleration phase
            if vehiclePathPlanner.accelerationPhase(1) == 1
                accelerationDistance = vehiclePathPlanner.accelerationPhase(4);
                averageAcceleration = vehiclePathPlanner.accelerationPhase(5);
                %global distance
                currentTotalDistance = currentGL.totalDistance;
                if ((currentTotalDistance + distance - accelerationDistance) < 0)
                    % whole route in acceleration phase
                    timeToReach = (1/ vehiclePathPlanner.simSpeed) * (-currentSpeed/averageAcceleration + sqrt((currentSpeed/averageAcceleration)^2+2*distance/averageAcceleration));
                    nextSpeed = currentSpeed + averageAcceleration*timeToReach * vehiclePathPlanner.simSpeed;
                else
                    % route is divided in acceleration phase (t1)
                    % and constant speed phase (t2)
                    t1 = (1/ vehiclePathPlanner.simSpeed) * (-currentSpeed/averageAcceleration + sqrt((currentSpeed/averageAcceleration)^2+2*(accelerationDistance - currentTotalDistance)/averageAcceleration));
                    t2 =  (1/ vehiclePathPlanner.simSpeed) * (currentTotalDistance+ distance - accelerationDistance)/ currentGL.speedLimit;
                    timeToReach = t1+t2;
                    nextSpeed = vehiclePathPlanner.accelerationPhase(3);
                    vehiclePathPlanner.accelerationPhase = zeros(1,5); %set acceleration phase to zero
                    
                end
                
            else
                timeToReach =  (1/ vehiclePathPlanner.simSpeed) * distance* (1/ currentSpeed); %timesteps to reach neighbour
                nextSpeed = currentSpeed;
            end
        end