classdef Prediction < handle
    %PREDICITION Makes all the predictions used by Vehicle Analysing Window
    %   Detailed explanation goes here 
    
    properties
        
        egoVehicle      % vehicle the camera focus on
        vehicles        % all vehicles that should be shown in the analysis       
        
        timeSteps       = 0.1 % time between two discrete steps
        nrStepsPredict  = 10  % number of steps to predict in the future                                    
    end 
    
    methods
        function obj = Prediction(vehicles, egoVehicleId)
            %PREDICTION Construct an instance of this class
            %   Detailed explanation goes here
            
            obj.vehicles = vehicles;
            obj.egoVehicle = vehicles(egoVehicleId);
                                      
                                   
        end
                                          
        %% Transformation functions
        
        function [positionsToEgo, yawToEgo] = transformToEgoCoord(obj, positions, yaw)
            % rotate x,y-data from global 2D map coordinate system to ego
            % vehicle coordinate system
            arguments
                obj
                positions   (:,2) double          % x,y positions
                yaw         (:,1) double = 0      % angle for every position                  
            end
            
            % Get ego vehicle coordinates and rotation
            positionEgo = obj.egoVehicle.dynamics.position;
            positionEgo = [positionEgo(:,1), -positionEgo(:,3)]; % ego vehicle position in (x,y) transformed from 3D to 2D map
            yawEgo = obj.egoVehicle.dynamics.orientation(4); % rotation to global coordination system
            
            % Transform and rotate
            R = [cos(yawEgo) -sin(yawEgo); sin(yawEgo) cos(yawEgo)]; % rotation matrix
            positionsToEgo = (positions - positionEgo)*R; % calculate from global to local ego system coordinates
            
            % Angle
            yawToEgo = yaw - yawEgo; % angle in local coordinate system
        end
        
        %% Calculation functions
        
        function emergencyBrakeDistance = calculateEmergencyBrakeDistance(obj)
            % Calculate the distance a vehicle needs to stop with min deceleration
            
            % get current dynamic parameters
            curSpeed = obj.egoVehicle.dynamics.speed;
            minAcceleration = obj.egoVehicle.dynamics.minDeceleration;
            % calculate distance
            emergencyBrakeDistance = 0.5*-curSpeed^2/minAcceleration;
        end
        
        %% Prediction functions
        
        function [predictedDistance, predictedSpeed] = predictMovement(~, tStep, accelerations, curDistance, curSpeed)
            % predict distance and speed with given future accelerations
            %
            %   tStep               % time between two acceleration values in s
            %   accelerations       % row vector of actual and future acceleration values
            %   curDistance         % current distance covered
            %   curSpeed            % current speed
            %
            % the math of this formulars can be found in DA Pintscher
            
            k = length(accelerations)-1; % number of steps to predict, k = 0 means only one step
            
            % future speed calculation
            predictedSpeed = curSpeed + tStep*sum(accelerations);
            
            % future distance calculation
            n=0:k;
            predictedDistance = curDistance + tStep*(k+1)*curSpeed + ...
                                tStep^2/2*sum(accelerations(n+1).*(2*k+1-2*n)); % n+1 because of matlab indexing
        end
        
        function predictedPositions = predictMovementEgoVehicle(obj, accelerations, nrStepsPredict, timeStep )
            % Calculate one prediction for every acceleration profile with
            % number of steps to predict with time steps
            
            predictedPositions = zeros(1,length(accelerations)); 
            for i = 1:length(accelerations)
                % predict distance covered by ego vehicle in nr of predict steps with
                % time step with constant acceleration at current speed
                predictedDistance = obj.predictMovement(timeStep, (1:nrStepsPredict)*accelerations(i), 0, obj.egoVehicle.dynamics.speed);
                
                % restrict distance, vehicle should not move backwards
                if predictedDistance < 0, predictedDistance = 0; end
                
                predictedPositions(i) = predictedDistance;
            end
        end
        
        %% Leading vehicle functions
        
        function [deltaDistance, deltaSpeed] = getDeltaMovementToLeading(obj)
            % Get delta movement information from leading to ego vehicle
            % (leading - ego)
            
            % Leading vehicle
            leadingVehicle = obj.egoVehicle.sensors.leadingVehicle;
            
            % Relative distance
            if obj.egoVehicle.sensors.frontSensorRange < obj.egoVehicle.sensors.frontDistance
                deltaDistance = inf;
            else
                deltaDistance = obj.egoVehicle.sensors.frontDistance;
            end
            
            % Relative speed
            if leadingVehicle ~= 0
                deltaSpeed = leadingVehicle.dynamics.speed - obj.egoVehicle.dynamics.speed;
            else
                deltaSpeed = inf;
            end
        end
             
    end
end

