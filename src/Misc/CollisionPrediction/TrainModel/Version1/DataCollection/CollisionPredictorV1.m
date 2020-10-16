classdef CollisionPredictorV1 < matlab.System & matlab.system.mixin.Propagates ...
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
        end
        %% check the condition of the crossway point 11, 9, 13, 44, 4, 52, 42, 36, 25, 23 and get the data for predictor every second
        
        function stepImpl(obj)
            load('idx.mat');
            load('Data0.mat');
            load('t.mat');
            if (get_param(obj.modelName,'SimulationTime') == fix(get_param(obj.modelName,'SimulationTime'))) &&...
                    (get_param(obj.modelName,'SimulationTime') ~= t)
                t = get_param(obj.modelName,'SimulationTime');
                save t t                    
                for i = 1:10
                    if (obj.Vehicles(i).pathInfo.currentRoute == 5 ||  obj.Vehicles(i).pathInfo.currentRoute == 46)  % crossway point 11
                        for k = 1:10
                            if (obj.Vehicles(k).pathInfo.currentRoute == 59 || obj.Vehicles(k).pathInfo.currentRoute == 12)
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                Distance_A = norm(Position_A - obj.Map.waypoints(11,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                Distance_B = norm(Position_B - obj.Map.waypoints(11,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,Distance_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,Distance_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                                save idx idx
                                save Data0 Data0
                            end
                        end
                    elseif (obj.Vehicles(i).pathInfo.currentRoute == 4 ||obj.Vehicles(i).pathInfo.currentRoute == 45)  %crossway point9
                        for k = 1:10
                            if (obj.Vehicles(k).pathInfo.currentRoute == 47 || obj.Vehicles(k).pathInfo.currentRoute == 6 || obj.Vehicles(k).pathInfo.currentRoute == 25 || obj.Vehicles(k).pathInfo.lastWaypoint == 82)
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                Distance_A = norm(Position_A - obj.Map.waypoints(9,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                Distance_B = norm(Position_B - obj.Map.waypoints(9,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,Distance_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,Distance_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                                save idx idx
                                save Data0 Data0
                            end
                        end
                 
                    elseif (obj.Vehicles(i).pathInfo.currentRoute == 14 ||obj.Vehicles(i).pathInfo.currentRoute == 62)  %crossway point 13
                        for k = 1:10
                            if (obj.Vehicles(k).pathInfo.currentRoute == 15 || obj.Vehicles(k).pathInfo.currentRoute == 61) 
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                Distance_A = norm(Position_A - obj.Map.waypoints(13,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                Distance_B = norm(Position_B - obj.Map.waypoints(13,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,Distance_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,Distance_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                                save idx idx
                                save Data0 Data0
                            end
                        end
                    elseif (obj.Vehicles(i).pathInfo.currentRoute == 20 ||obj.Vehicles(i).pathInfo.currentRoute == 21) %crossway point 44
                        for k = 1:10
                            if (obj.Vehicles(k).pathInfo.currentRoute == 71)
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                Distance_A = norm(Position_A - obj.Map.waypoints(44,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                Distance_B = norm(Position_B - obj.Map.waypoints(44,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,Distance_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,Distance_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                                save idx idx
                                save Data0 Data0
                            end
                        end
                    elseif (obj.Vehicles(i).pathInfo.currentRoute == 3 ||obj.Vehicles(i).pathInfo.currentRoute == 43)  %crossway point 4
                        for k = 1:10
                            if (obj.Vehicles(k).pathInfo.currentRoute == 42 || obj.Vehicles(k).pathInfo.currentRoute == 2) 
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                Distance_A = norm(Position_A - obj.Map.waypoints(4,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                Distance_B = norm(Position_B - obj.Map.waypoints(4,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,Distance_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,Distance_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                                save idx idx
                                save Data0 Data0
                            end
                        end
                    elseif obj.Vehicles(i).pathInfo.currentRoute == 75   %crossway point 52
                        for k = 1:10
                            if obj.Vehicles(k).pathInfo.currentRoute == 24
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                Distance_A = norm(Position_A - obj.Map.waypoints(52,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                Distance_B = norm(Position_B - obj.Map.waypoints(52,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,Distance_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,Distance_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                                save idx idx
                                save Data0 Data0
                            end
                        end
                    elseif (obj.Vehicles(i).pathInfo.currentRoute == 63 ||obj.Vehicles(i).pathInfo.currentRoute == 16)   %crossway point 36
                        for k = 1:10
                            if obj.Vehicles(k).pathInfo.currentRoute == 64 
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                Distance_A = norm(Position_A - obj.Map.waypoints(36,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                Distance_B = norm(Position_B - obj.Map.waypoints(36,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,Distance_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,Distance_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                                save idx idx
                                save Data0 Data0
                            end
                        end
                    elseif (obj.Vehicles(i).pathInfo.currentRoute == 10||obj.Vehicles(i).pathInfo.currentRoute == 55)    %crossway point 25
                        for k = 1:10
                            if obj.Vehicles(k).pathInfo.currentRoute == 66 
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                Distance_A = norm(Position_A - obj.Map.waypoints(25,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                Distance_B = norm(Position_B - obj.Map.waypoints(25,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,Distance_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,Distance_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                                save idx idx
                                save Data0 Data0
                            end
                        end
                    elseif (obj.Vehicles(i).pathInfo.currentRoute == 57||obj.Vehicles(i).pathInfo.currentRoute == 11)    %crossway point 23
                        for k = 1:10
                            if (obj.Vehicles(k).pathInfo.currentRoute == 9 || obj.Vehicles(k).pathInfo.currentRoute == 54) 
                                Speed_A = obj.Vehicles(i).dynamics.speed;
                                Position_A = obj.Vehicles(i).dynamics.position;
                                Speed_B = obj.Vehicles(k).dynamics.speed;
                                Position_B = obj.Vehicles(k).dynamics.position;
                                maxSpeed_A = obj.Vehicles(i).dynamics.maxSpeed;
                                frontSensorRange_A = obj.Vehicles(i).sensors.frontSensorRange;
                                AEBdistance_A = obj.Vehicles(i).sensors.AEBdistance;
                                minDeceleration_A = obj.Vehicles(i).dynamics.minDeceleration;
                                Distance_A = norm(Position_A - obj.Map.waypoints(23,:));
                                maxSpeed_B = obj.Vehicles(k).dynamics.maxSpeed;
                                frontSensorRange_B = obj.Vehicles(k).sensors.frontSensorRange;
                                AEBdistance_B = obj.Vehicles(k).sensors.AEBdistance;
                                minDeceleration_B = obj.Vehicles(k).dynamics.minDeceleration;
                                Distance_B = norm(Position_B - obj.Map.waypoints(23,:));
                                Data0(idx,:) = {maxSpeed_A,Speed_A,Distance_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
                                    maxSpeed_B,Speed_B,Distance_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,i,k};
                                idx = idx + 1;
                                save idx idx
                                save Data0 Data0
                            end
                        end
                    end
                end
                end
            end
        end
end
