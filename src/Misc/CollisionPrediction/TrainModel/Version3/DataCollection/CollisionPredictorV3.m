classdef CollisionPredictorV3 < matlab.System & matlab.system.mixin.Propagates ...
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
            
            load('Speed.mat')
            load('DistancetoCrosswaypoint.mat')
            load('test.mat');
            %%%%%%%%%%%%%%%%%%Scenario 1:for crossway point 11, 4, 36, 44, 25
            if (obj.Vehicles(1).pathInfo.lastWaypoint == 12 || obj.Vehicles(1).pathInfo.lastWaypoint == 13) && ...              %11
                    (obj.Vehicles(2).pathInfo.lastWaypoint == 31 ||obj.Vehicles(2).pathInfo.lastWaypoint == 32) && test(1) < 1
                
                Speed(1) = obj.Vehicles(1).dynamics.speed;
                save Speed Speed
                test(1) = test(1) + 1;
                save test test
                DistancetoCrosswaypoint(1) = norm(obj.Vehicles(1).dynamics.position - obj.Map.waypoints(11,:));
                save DistancetoCrosswaypoint DistancetoCrosswaypoint
                
                Speed(2) = obj.Vehicles(2).dynamics.speed;
                save Speed Speed
                DistancetoCrosswaypoint(2) = norm(obj.Vehicles(2).dynamics.position - obj.Map.waypoints(11,:));
                save DistancetoCrosswaypoint DistancetoCrosswaypoint
                
            elseif (obj.Vehicles(4).pathInfo.lastWaypoint == 5 || obj.Vehicles(4).pathInfo.lastWaypoint == 6)...
                    && (obj.Vehicles(3).pathInfo.lastWaypoint == 7 || obj.Vehicles(3).pathInfo.lastWaypoint == 8) && ...  %4
                    test(2) < 1
                
                Speed(3) = obj.Vehicles(3).dynamics.speed;
                save Speed Speed
                test(2) = test(2) + 1;
                save test test
                DistancetoCrosswaypoint(3) = norm(obj.Vehicles(3).dynamics.position - obj.Map.waypoints(4,:));
                save DistancetoCrosswaypoint DistancetoCrosswaypoint
                
                Speed(4) = obj.Vehicles(4).dynamics.speed;
                save Speed Speed
                DistancetoCrosswaypoint(4) = norm(obj.Vehicles(4).dynamics.position - obj.Map.waypoints(4,:));
                
            elseif (obj.Vehicles(5).pathInfo.lastWaypoint == 37 || obj.Vehicles(5).pathInfo.lastWaypoint == 38) && ...             %36
                    test(3) < 1
                
                Speed(5) = obj.Vehicles(5).dynamics.speed;
                save Speed Speed
                test(3) = test(3) + 1;
                save test test
                DistancetoCrosswaypoint(5) = norm(obj.Vehicles(5).dynamics.position - obj.Map.waypoints(36,:));
                save DistancetoCrosswaypoint DistancetoCrosswaypoint
                
                Speed(6) = obj.Vehicles(6).dynamics.speed;
                save Speed Speed
                DistancetoCrosswaypoint(6) = norm(obj.Vehicles(6).dynamics.position - obj.Map.waypoints(36,:));
                save DistancetoCrosswaypoint DistancetoCrosswaypoint
                
            elseif (obj.Vehicles(7).pathInfo.lastWaypoint == 45 || obj.Vehicles(7).pathInfo.lastWaypoint == 46) && (obj.Vehicles(8).pathInfo.lastWaypoint == 79) && ...  %44
                    test(4) < 1
                Speed(7) = obj.Vehicles(7).dynamics.speed;
                save Speed Speed
                test(4) = test(4) + 1;
                save test test
                DistancetoCrosswaypoint(7) = norm(obj.Vehicles(7).dynamics.position - obj.Map.waypoints(44,:));
                save DistancetoCrosswaypoint DistancetoCrosswaypoint
                
                Speed(8) = obj.Vehicles(8).dynamics.speed;
                save Speed Speed
                DistancetoCrosswaypoint(8) = norm(obj.Vehicles(8).dynamics.position - obj.Map.waypoints(44,:));
                save DistancetoCrosswaypoint DistancetoCrosswaypoint
                
            elseif (obj.Vehicles(9).pathInfo.lastWaypoint == 26 || obj.Vehicles(9).pathInfo.lastWaypoint == 27) && (obj.Vehicles(10).pathInfo.lastWaypoint == 42) && ...  %25
                    test(5) < 1
                Speed(9) = obj.Vehicles(9).dynamics.speed;
                save Speed Speed
                test(5) = test(5) + 1;
                save test test
                DistancetoCrosswaypoint(9) = norm(obj.Vehicles(9).dynamics.position - obj.Map.waypoints(25,:));
                save DistancetoCrosswaypoint DistancetoCrosswaypoint
                
                Speed(10) = obj.Vehicles(10).dynamics.speed;
                save Speed Speed
                DistancetoCrosswaypoint(10) = norm(obj.Vehicles(10).dynamics.position - obj.Map.waypoints(25,:));
                save DistancetoCrosswaypoint DistancetoCrosswaypoint
            end
            
            %%%%%%%%%%%%%Scenario 2:crossway point 9, 13, 52, 42, 23
% %             if (obj.Vehicles(1).pathInfo.lastWaypoint ==
% %                 10) && ...                                                      %9A
% %                     (obj.Vehicles(2).pathInfo.lastWaypoint == 14 || obj.Vehicles(2).pathInfo.lastWaypoint == 15) && test(1) < 1
% %                 
% %                 Speed(1) = obj.Vehicles(1).dynamics.speed;
% %                 save Speed Speed
% %                 test(1) = test(1) + 1;
% %                 save test test
% %                 DistancetoCrosswaypoint(1) = norm(obj.Vehicles(1).dynamics.position - obj.Map.waypoints(9,:));
% %                 save DistancetoCrosswaypoint DistancetoCrosswaypoint
% %                 
% %                 Speed(2) = obj.Vehicles(2).dynamics.speed;
% %                 save Speed Speed
% %                 DistancetoCrosswaypoint(2) = norm(obj.Vehicles(2).dynamics.position - obj.Map.waypoints(9,:));
% %                 save DistancetoCrosswaypoint DistancetoCrosswaypoint
% %                 
%             if (obj.Vehicles(1).pathInfo.lastWaypoint == 14) &&...                                                                 %9B
%                     (obj.Vehicles(2).pathInfo.lastWaypoint == 55 || obj.Vehicles(2).pathInfo.lastWaypoint == 67) && test(1) < 1
%                 
%                 Speed(1) = obj.Vehicles(1).dynamics.speed;
%                 save Speed Speed
%                 test(1) = test(1) + 1;
%                 save test test
%                 DistancetoCrosswaypoint(1) = norm(obj.Vehicles(1).dynamics.position - obj.Map.waypoints(9,:));
%                 save DistancetoCrosswaypoint DistancetoCrosswaypoint
%                 
%                 Speed(2) = obj.Vehicles(2).dynamics.speed;
%                 save Speed Speed
%                 DistancetoCrosswaypoint(2) = norm(obj.Vehicles(2).dynamics.position - obj.Map.waypoints(9,:));
%                 
%             elseif (obj.Vehicles(3).pathInfo.lastWaypoint == 34 || obj.Vehicles(3).pathInfo.lastWaypoint == 36)...
%                     && (obj.Vehicles(4).pathInfo.lastWaypoint == 35 || obj.Vehicles(4).pathInfo.lastWaypoint == 4) && ...  %13
%                     test(2) < 1
%                 
%                 Speed(3) = obj.Vehicles(3).dynamics.speed;
%                 save Speed Speed
%                 test(2) = test(2) + 1;
%                 save test test
%                 DistancetoCrosswaypoint(3) = norm(obj.Vehicles(3).dynamics.position - obj.Map.waypoints(13,:));
%                 save DistancetoCrosswaypoint DistancetoCrosswaypoint
%                 
%                 Speed(4) = obj.Vehicles(4).dynamics.speed;
%                 save Speed Speed
%                 DistancetoCrosswaypoint(4) = norm(obj.Vehicles(4).dynamics.position - obj.Map.waypoints(13,:));
%                 save DistancetoCrosswaypoint DistancetoCrosswaypoint
%                 
%                 %%%%%%%%%%%%%%%  get the collision data near Waypoint 13 
%             elseif(obj.Vehicles(3).pathInfo.lastWaypoint == 34 || obj.Vehicles(3).pathInfo.lastWaypoint == 36 || obj.Vehicles(3).pathInfo.lastWaypoint == 13)...
%                     && (obj.Vehicles(4).pathInfo.lastWaypoint == 35 || obj.Vehicles(4).pathInfo.lastWaypoint == 4 || obj.Vehicles(4).pathInfo.lastWaypoint == 13)&&...
%                     (obj.Vehicles(3).status.collided == 1 && obj.Vehicles(4).status.collided == 1) && test(5) < 1
%                 
%                 test(5) = test(5) + 1;
%                 save test test
%                 Collision13 = 1;
%                 save Collision13 Collision13
%                 %%%%%%%%%%%%%%%%
%                 
%                 
%             elseif (obj.Vehicles(5).pathInfo.lastWaypoint == 53) && ...  %52
%                     test(3) < 1
%                 
%                 Speed(5) = obj.Vehicles(5).dynamics.speed;
%                 save Speed Speed
%                 test(3) = test(3) + 1;
%                 save test test
%                 DistancetoCrosswaypoint(5) = norm(obj.Vehicles(5).dynamics.position - obj.Map.waypoints(36,:));
%                 save DistancetoCrosswaypoint DistancetoCrosswaypoint
%                 
%                 Speed(6) = obj.Vehicles(6).dynamics.speed;
%                 save Speed Speed
%                 DistancetoCrosswaypoint(6) = norm(obj.Vehicles(6).dynamics.position - obj.Map.waypoints(36,:));
%                 save DistancetoCrosswaypoint DistancetoCrosswaypoint
%                 
%             elseif (obj.Vehicles(7).pathInfo.lastWaypoint == 28) && ...
%                     (obj.Vehicles(8).pathInfo.lastWaypoint == 24 || obj.Vehicles(8).pathInfo.lastWaypoint == 25) && ...  %23
%                     test(4) < 1
%                 Speed(7) = obj.Vehicles(7).dynamics.speed;
%                 save Speed Speed
%                 test(4) = test(4) + 1;
%                 save test test
%                 DistancetoCrosswaypoint(7) = norm(obj.Vehicles(7).dynamics.position - obj.Map.waypoints(44,:));
%                 save DistancetoCrosswaypoint DistancetoCrosswaypoint
%                 
%                 Speed(8) = obj.Vehicles(8).dynamics.speed;
%                 save Speed Speed
%                 DistancetoCrosswaypoint(8) = norm(obj.Vehicles(8).dynamics.position - obj.Map.waypoints(44,:));
%                 save DistancetoCrosswaypoint DistancetoCrosswaypoint
%             end
        end
    end
end
