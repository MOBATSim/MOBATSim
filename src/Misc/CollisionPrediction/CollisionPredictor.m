classdef CollisionPredictor < matlab.System & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
    
    % Public, tunable properties
    properties
        
    end
    
    % Public, non-tunable properties
    properties(Nontunable)
        
    end
    
    properties(DiscreteState)
        
    end
    
    % Pre-computed constants
    properties(Access = private)
        Vehicles
        Map
        modelName = evalin('base','modelName');
    end
    
    methods(Access = protected)
        %% set the initial value of CollisionPredictor
        function setupImpl(obj)
            obj.Vehicles = evalin('base','Vehicles');
            obj.Map = evalin('base','Map');      
            Warning = cell2table(cell(0,10), 'VariableNames', {'V1', 'V2','V3',  'V4', 'V5', 'V6','V7','V8','V9', 'V10'});
            Status = cell2table(cell(0,10), 'VariableNames', {'V1', 'V2','V3',  'V4', 'V5', 'V6','V7','V8','V9', 'V10'});
            t = 0;
            save t t
            save Warning Warning
            save Status Status
        end
        %% check the condition of the crossway point 11, 9, 13, 44, 4, 52, 42, 36, 25, 23 and get the data for predictor every second
        
        function stepImpl(obj)
            load('t.mat');
            if (get_param(obj.modelName,'SimulationTime') == fix(get_param(obj.modelName,'SimulationTime'))) &&...
                    (get_param(obj.modelName,'SimulationTime') ~= t)
                t = get_param(obj.modelName,'SimulationTime');
                save t t
                Data0 = cell2table(cell(0,14), 'VariableNames', {'maxSpeed_A', 'Speed_A','DistancetoCrosswaypoint_A',  'frontSensorRange_A', 'AEBdistance_A', 'minDeceleration_A','maxSpeed_B','Speed_B','DistancetoCrosswaypoint_B', 'frontSensorRange_B', 'AEBdistance_B', 'minDeceleration_B','A','B'});
                idx = 1;
                for i = 1:10
                    load('trainedModel');
                    if obj.Vehicles(i).pathInfo.currentRoute == 5 &&  obj.Vehicles(i).dynamics.speed ~= 0 % crossway point 11
                        for k = 1:10
                            if (obj.Vehicles(k).pathInfo.currentRoute == 59 || obj.Vehicles(k).pathInfo.currentRoute == 12)...
                                    &&  obj.Vehicles(k).dynamics.speed ~= 0
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_A = norm(Position_A - obj.Map.waypoints(11,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_B = norm(Position_B - obj.Map.waypoints(11,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,DistancetoCrosswaypoint_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,DistancetoCrosswaypoint_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                            end
                        end
                    elseif obj.Vehicles(i).pathInfo.currentRoute == 4 &&  obj.Vehicles(i).dynamics.speed ~= 0 %crossway point9
                        for k = 1:10
                            if (obj.Vehicles(k).pathInfo.currentRoute == 47 || obj.Vehicles(k).pathInfo.currentRoute == 6)...
                                    &&  obj.Vehicles(k).dynamics.speed ~= 0
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_A = norm(Position_A - obj.Map.waypoints(9,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_B = norm(Position_B - obj.Map.waypoints(9,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,DistancetoCrosswaypoint_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,DistancetoCrosswaypoint_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                            end
                        end
                    elseif (obj.Vehicles(i).pathInfo.currentRoute == 14 ||obj.Vehicles(i).pathInfo.currentRoute == 62) ...
                            &&  obj.Vehicles(i).dynamics.speed ~= 0 %crossway point 13
                        for k = 1:10
                            if (obj.Vehicles(k).pathInfo.currentRoute == 15 || obj.Vehicles(k).pathInfo.currentRoute == 61)...
                                    &&  obj.Vehicles(k).dynamics.speed ~= 0
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_A = norm(Position_A - obj.Map.waypoints(13,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_B = norm(Position_B - obj.Map.waypoints(13,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,DistancetoCrosswaypoint_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,DistancetoCrosswaypoint_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                            end
                        end
                    elseif obj.Vehicles(i).pathInfo.currentRoute == 20 &&  obj.Vehicles(i).dynamics.speed ~= 0 %crossway point 44
                        for k = 1:10
                            if obj.Vehicles(k).pathInfo.currentRoute == 71 &&  obj.Vehicles(k).dynamics.speed ~= 0
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_A = norm(Position_A - obj.Map.waypoints(13,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_B = norm(Position_B - obj.Map.waypoints(13,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,DistancetoCrosswaypoint_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,DistancetoCrosswaypoint_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                            end
                        end
                    elseif obj.Vehicles(i).pathInfo.currentRoute == 43 &&  obj.Vehicles(i).dynamics.speed ~= 0 %crossway point 4
                        for k = 1:10
                            if  obj.Vehicles(k).pathInfo.currentRoute == 2 && obj.Vehicles(k).dynamics.speed ~= 0
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_A = norm(Position_A - obj.Map.waypoints(13,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_B = norm(Position_B - obj.Map.waypoints(13,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,DistancetoCrosswaypoint_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,DistancetoCrosswaypoint_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                            end
                        end
                    elseif obj.Vehicles(i).pathInfo.currentRoute == 75 &&  obj.Vehicles(i).dynamics.speed ~= 0 %crossway point 52
                        for k = 1:10
                            if obj.Vehicles(k).pathInfo.currentRoute == 24 &&  obj.Vehicles(k).dynamics.speed ~= 0
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_A = norm(Position_A - obj.Map.waypoints(13,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_B = norm(Position_B - obj.Map.waypoints(13,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,DistancetoCrosswaypoint_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,DistancetoCrosswaypoint_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                            end
                        end
                      
                    elseif obj.Vehicles(i).pathInfo.currentRoute == 16 &&  obj.Vehicles(i).dynamics.speed ~= 0 %crossway point 36
                        for k = 1:10
                            if obj.Vehicles(k).pathInfo.currentRoute == 64 &&  obj.Vehicles(k).dynamics.speed ~= 0
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_A = norm(Position_A - obj.Map.waypoints(13,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_B = norm(Position_B - obj.Map.waypoints(13,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,DistancetoCrosswaypoint_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,DistancetoCrosswaypoint_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                            end
                        end
                    elseif obj.Vehicles(i).pathInfo.currentRoute == 10 &&  obj.Vehicles(i).dynamics.speed ~= 0 %crossway point 25
                        for k = 1:10
                            if obj.Vehicles(k).pathInfo.currentRoute == 66 &&  obj.Vehicles(k).dynamics.speed ~= 0
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_A = norm(Position_A - obj.Map.waypoints(13,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_B = norm(Position_B - obj.Map.waypoints(13,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,DistancetoCrosswaypoint_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,DistancetoCrosswaypoint_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                            end
                        end
                    elseif obj.Vehicles(i).pathInfo.currentRoute == 57 &&  obj.Vehicles(i).dynamics.speed ~= 0 %crossway point 23
                        for k = 1:10
                            if (obj.Vehicles(k).pathInfo.currentRoute == 9 || obj.Vehicles(k).pathInfo.currentRoute == 54)...
                                    &&  obj.Vehicles(k).dynamics.speed ~= 0
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_A = norm(Position_A - obj.Map.waypoints(13,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                DistancetoCrosswaypoint_B = norm(Position_B - obj.Map.waypoints(13,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,DistancetoCrosswaypoint_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,DistancetoCrosswaypoint_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                            end
                        end
                    end
                end
                %% create report in table
                load('Warning.mat');
                load('Status.mat');
                %                                      load('Probability.mat');
                H = height(Data0);
                on = [];
                off = [];
                v = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2];  % 2 means off
                if H ~= 0;
                    [Predict,~] = predict(trainedModel.ClassificationEnsemble,Data0(:,1:12));
                    x = 1;
                    y= (1:10);
                    for z = 1:H
                        if v(Data0.A(z)) ~= 1
                            %When more than 2 vehicles in one
                            %merge, if one of the prediction for
                            %one vehicle shows colliede, the
                            %prediction for this vehicle should be
                            %colliede.
                            v(Data0.A(z)) = Predict(z);
                        end
                        if v(Data0.B(z)) ~= 1
                            v(Data0.B(z)) = Predict(z);
                        end
                        on(x) = Data0.A(z);
                        x = x + 1;
                        on(x) = Data0.B(z);
                        x = x + 1;
                    end
                    off = y(~ismember(y, on));
                    for a = 1:length(off)
                        v(off(a)) = 2;
                    end
                    Warning(t,:) = {v(1), v(2), v(3), v(4), v(5), v(6), v(7), v(8), v(9), v(10)};
                else
                    Warning(t,:) = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
                end
                
                for k = 1:10
                    Status(t,k) = {obj.Vehicles(k).status.collided};
                end
                save Warning Warning
                save Status Status
                %% show Warning in command window
                for k = 1:10
                    if v(k) == 1
                        disp(['Vehicle ',num2str(k),' might collide'])
                    end
                end
            end
        end
    end
end






