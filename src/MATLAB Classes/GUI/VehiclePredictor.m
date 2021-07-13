classdef VehiclePredictor < handle
    %VEHICLEPREDICTOR Makes all the predictions used by Vehicle Analysing Window
    %   Detailed explanation goes here 
    
    properties
        % Simulation
        currentSimTime          (1,1) double = 0    % current simulation time
        % Environment
        vehicles                (:,:) Vehicle       % all vehicles for simulation
        egoVehicle                    Vehicle       % vehicle the camera focus on       
        roadNetwork             (:,2)               % 2D boundaries of all roads in map coordinates
        roadNetworkEgo          (:,2)               % road network in ego perspective
        % Leading vehicle
        deltaDistance           (1,1) double = Inf  % Distance to leading vehicle (leading - ego)
        deltaSpeed              (1,1) double = Inf  % Speed difference to leading vehicle (leading - ego)
        safeVelocitySets        (2,100) double      % Contains distance-velocity pairs, when actual delta velocity is smaller at given distance, its a safe velocity
        safeTerminalVelocitySets(2,100) double      % Safe terminal velocity is like safe velocity but this region contains all states that are safe in a terminal state
        % Ego vehicle
        emBrakeDistance         (1,1) double = 0    % distance to stop ego vehicle in an emergency
        % Prediction
        timeSteps               (1,1) double = 0.1  % time between two discrete steps
        nrStepsPredict          (1,1) double = 10   % number of steps to predict in the future
        predictedPositions      (1,:) double        % positions of vehicle in predicted time with different motion profiles
        predictedDeltaDistances (1,:) double        % predicted distances between leading and ego for a number of steps in the future
        predictedDeltaSpeeds    (1,:) double        % predicted speed between leading and ego for a number of steps in the future
        % Near vehicles
        nearVehicles            (1,:) logical       % list containing information if vehicle is near by ego vehicle
        positionsToEgo          (:,2) double        % positions of vehicles to ego vehicle
        yawToEgo                (:,1) double        % angle of vehicles to ego vehicle
        % Prediction error table
        predictionErrorTable                        % table containing test data for checking prediction quality
        tableTimeStamp          (:,1) = []          % logged timestamps
        tableCurvedRoad         (:,1) = []          % is vehicle on curved road at this moment
        tablePosition           (:,1) = []          % logged positions
        tablePredictedPosition  (:,1) = NaN         % predicted position for this time
        tablePositionError      (:,1) = []          % position error for this time stamp
        simTs                   (1,1) = 0
    end
    
    properties
        % Environment
        drivingScenarioRoads    (:,:) drivingScenario = scenario_map_v1()   % driving scenario, but only containing roads 
    end
    
    methods
        function obj = VehiclePredictor(vehicles, egoVehicleId)
            %VEHICLEPREDICTOR Construct an instance of this class
            %   Detailed explanation goes here
            
            % set properties
            obj.vehicles = vehicles;
            obj.egoVehicle = vehicles(egoVehicleId);
            minDecelerations =  [-9.15 -6.15]; % Could be more flexible later
            safetyDistances = [1 2];
            range = 150; % 150m is the interesting maximal interesting range for interaction to leadin at the moment
            [obj.safeVelocitySets(1,:),obj.safeVelocitySets(2,:)]  = obj.getMaximalSafeSets(0-minDecelerations(1), ... 
                                                                                            safetyDistances(1), ...
                                                                                            [0 range]);
            [obj.safeTerminalVelocitySets(1,:),obj.safeTerminalVelocitySets(2,:)]  = obj.getMaximalSafeSets(0-minDecelerations(2), ... 
                                                                                                            safetyDistances(2), ...
                                                                                                            [0 range]);
                                                                                                        
            % setup
            % generate a plottable 2D road network (map coordinates ) out of a road scenario
            obj.roadNetwork = obj.prepareRoadNetwork(roadBoundaries(obj.drivingScenarioRoads));
            obj.roadNetworkEgo = obj.transformToEgoCoord(obj.roadNetwork);
        end
        
        %% Setup functions
        
        function roadNetwork = prepareRoadNetwork(~, roadNetwork)
            % Prepare road by merging all the road boundary parts and reordering
            
            for i=1:length(roadNetwork)
                roadNetwork{i} = [ roadNetwork{i}; NaN NaN NaN ]; 
                % append an Nan after every cell array
                % so that parts of the roadnetwork get
                % not connected with strange lines to each other when concatinated
            end
            roadNetwork = vertcat(roadNetwork{:}); % add all cellarrays together
            roadNetwork = [-roadNetwork(:,2) roadNetwork(:,1)]; % transform from bep coordinates (3D) to map 2D
            
        end
                                          
        %% Transformation functions
        
        function [positionsToEgo, yawToEgo] = transformToEgoCoord(obj, positions, yaw, egoVehicle)
            % rotate x,y-data from global 2D map coordinate system to ego
            % vehicle coordinate system
            arguments
                obj %#ok<INUSA>
                positions   (:,2) double                        % x,y positions
                yaw         (:,1) double    = 0                 % angle for every position
                egoVehicle  (1,1) Vehicle   = obj.egoVehicle    % ego vehicle were the pose should be transformed to
            end
            
            % Get ego vehicle coordinates and rotation
            positionEgo = egoVehicle.dynamics.position;
            positionEgo = [positionEgo(:,1), -positionEgo(:,3)]; % ego vehicle position in (x,y) transformed from 3D to 2D map
            yawEgo = egoVehicle.dynamics.orientation(4); % rotation to global coordination system
            
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
        
        function [distances, velocities] = getMaximalSafeSets(~, deltaAcceleration, safetyDistance, limits)
            % Calculate 100 linear spaced velocities between the limits
            % when vehicle should stop at 0 + safetyDistance and could
            % apply deltaAcceleration to the leading vehicle (leading -
            % ego)
            
            distances = linspace(limits(1), limits(2));
            velocities = sqrt(2*(distances-safetyDistance)*deltaAcceleration);
        end
        
        %% Prediction functions
        
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
        
        function [deltaDistances, deltaSpeeds] = predictEgoEmergencyStop(obj, nrStepsPredict, timeSteps)
            % Predict the delta distance and delta speed to leading vehicle for ego 
            % when doing an emergency stop for number of steps
            arguments
                obj
                nrStepsPredict = obj.nrStepsPredict
                timeSteps = obj.timeSteps
            end
            
            % first value is actual value
            deltaDistances = obj.deltaDistance;
            deltaSpeeds = obj.deltaSpeed;
            % stop when no leading vehicle in front
            if (deltaDistances == Inf) || (deltaSpeeds == Inf), return; end
            
            % needed as input for movement prediction
            predictedDistance = obj.deltaDistance;
            predictedSpeed = obj.deltaSpeed;
            
            deltaAcceleration = 0 - obj.egoVehicle.dynamics.minDeceleration; % deceleraton from leading is assumed as 0 at the moment
            
            for i=1:nrStepsPredict
                % Predict the distance and speed between ego and leading
                % vehicle
                [predictedDistance, predictedSpeed] = obj.predictMovement(timeSteps, deltaAcceleration, predictedDistance, predictedSpeed);
                % safe values
                deltaDistances(end+1) = predictedDistance; %#ok<AGROW>
                deltaSpeeds(end+1) = predictedSpeed; %#ok<AGROW>
            end
        end
        
        %% Ego vehicle functions
        
        function [nearVehicles, positionsToEgo, yawToEgo]  = getVehiclesNearByEgo(obj, radius, vehicles, egoVehicle)
            % Get all vehicles near by ego vehicle, maximum distance
            % specified with radius, and also the relative positions and
            % angle
            arguments
                obj
                radius      (1,1) double                        % radius to search for other vehicles
                vehicles    (:,:) Vehicle   = obj.vehicles      % all vehicles to search through
                egoVehicle  (1,1) Vehicle   = obj.egoVehicle    % ego vehicle
            end
            
             % get all positions and orientations
            positions = cat(1,cat(2,vehicles(:).dynamics).position);
            orientations = cat(1,cat(2,vehicles(:).dynamics).orientation);
            
            % transform from 3D to 2D map
            positions2D = [positions(:,1), -positions(:,3)]; % all vehicle position in (x,y)
            
            % transform positions from global to ego coordinate system
            [positionsToEgo, yawToEgo] = obj.transformToEgoCoord(positions2D, orientations(:,4));
            
            % get all vehicles in radius to ego vehicle
            distancesToOthers = dist(positionsToEgo,positionsToEgo(egoVehicle.id,:)'); % calculate distances from ego vehicle to other vehicles
            nearVehicles = distancesToOthers<radius; % select only vehicles closer than 150 m for plotting
            nearVehicles(egoVehicle.id) = 0; % remove ego vehicle from list
            
        end
        
        %% Leading vehicle functions
        
        function [deltaDistance, deltaSpeed] = getRelativeMovementToLeading(obj)
            % Get delta movement information from leading to ego vehicle
            % (leading - ego)
            
            % Leading vehicle
            leadingVehicle = obj.egoVehicle.sensors.leadingVehicle;
            
            % Relative distance
            if obj.egoVehicle.sensors.frontSensorRange < obj.egoVehicle.sensors.distanceToLeadingVehicle
                deltaDistance = inf;
            else
                deltaDistance = obj.egoVehicle.sensors.distanceToLeadingVehicle;
            end
            
            % Relative speed
            if leadingVehicle ~= 0
                deltaSpeed = leadingVehicle.dynamics.speed - obj.egoVehicle.dynamics.speed;
            else
                deltaSpeed = inf;
            end
        end
        
        %% Test functions
        
        function testPredictionError(obj)
            % log predicted data, real data and compare
            
            if abs(obj.egoVehicle.dynamics.acceleration) < 0.1 ... % only predict when acc = 0 or near 0
               && obj.egoVehicle.dynamics.speed > 0                % and moving

                % New time entry
                obj.tableTimeStamp(end+1) = obj.currentSimTime;
          
                % Actual position
                obj.tablePosition(end+1) = obj.egoVehicle.dynamics.position(1); % first only on x-Axis
                
                % Predicted value
                entryForPrediction = length(obj.tableTimeStamp) + obj.nrStepsPredict*obj.timeSteps/obj.simTs; % actual time entry plus future entries
                obj.tablePredictedPosition(entryForPrediction) = obj.predictMovement(obj.timeSteps, (1:obj.nrStepsPredict)*0, obj.tablePosition(end), obj.egoVehicle.dynamics.speed);
            end
            
            % Remove all future predicted entries when time jump happend
            if length(obj.tableTimeStamp) == 1 ... % check if first entry is bigger then simTs
               && (obj.tableTimeStamp(end) - 0) > 2*obj.simTs % time jump (two times for preventing rounding errors)
           
                % Set future predicted entries except the new one to NaN
                obj.tablePredictedPosition(length(obj.tableTimeStamp):end-1) = NaN;
            elseif length(obj.tableTimeStamp) >= 1 ... % check if time change is bigger then simTs
                    && (obj.tableTimeStamp(end)- obj.tableTimeStamp(end-1)) > 2*obj.simTs % time jump (two times for preventing rounding errors)
                
                % Set future predicted entries except the new one to NaN
                obj.tablePredictedPosition(length(obj.tableTimeStamp):end-1) = NaN;
            end
            
            if obj.currentSimTime == 15
                varNames = {'Time','Position','Predicted Position','Position Error'};
                % resize
                obj.tablePredictedPosition = obj.tablePredictedPosition(1:length(obj.tableTimeStamp),1);
                   
                obj.predictionErrorTable = table(obj.tableTimeStamp, ...
                                                 obj.tablePosition, ...
                                                 obj.tablePredictedPosition, ...
                                                 (obj.tablePosition - obj.tablePredictedPosition), ...
                                                 'VariableNames', varNames);               
            end 
        end
        
        
        %% Update functions
        
        function update(obj, currentSimTime)
            % Update properties
            arguments
                obj             (1,1)
                currentSimTime  (1,1) double = obj.currentSimTime % take acutal time, if no new time entered
            end           
            
            %% Get information
            
            % Calculate sim time step at first call within simulation
            if obj.currentSimTime == 0 && currentSimTime > 0
                obj.simTs = currentSimTime - obj.currentSimTime;
            end
            
            % Simulation time
            obj.currentSimTime = currentSimTime;
            
            % Leading vehicle
            [obj.deltaDistance, obj.deltaSpeed] = obj.getRelativeMovementToLeading();
            
            % Ego vehicle
            obj.emBrakeDistance = obj.calculateEmergencyBrakeDistance();
            % Predict positions with different motion profiles (constant accelerations)
            obj.predictedPositions = obj.predictMovementEgoVehicle([2 0 -1 -3], obj.nrStepsPredict, obj.timeSteps);
            
            %% Set information           
            
            % Road network
            obj.roadNetworkEgo = obj.transformToEgoCoord(obj.roadNetwork);
            
            % Prediction
            % Emergency stop
            [obj.predictedDeltaDistances, obj.predictedDeltaSpeeds] = obj.predictEgoEmergencyStop();
            % Test prediction error
            obj.testPredictionError();
            
            % Vehicles
            % get all vehicles and their poses, that are close as 150m to ego vehicle
            [obj.nearVehicles, obj.positionsToEgo, obj.yawToEgo] = obj.getVehiclesNearByEgo(150);
        end
    end
    
    methods(Static)
        %% Prediction functions
        
        function [predictedDistance, predictedSpeed] = predictMovement(tStep, accelerations, curDistance, curSpeed)
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
    end
end

