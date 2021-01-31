classdef VehicleKinematics_WaypointGenerator_stanley < VehicleKinematics
    % This blocks generates waypoints.
    %
    
    % Pre-computed constants
    properties(Access = private)
        vehicle
        map = evalin('base','Map');
        simSpeed = evalin('base','simSpeed');
        modelName = evalin('base','modelName');
        laneWidth = 3.7;
        laneSwitchWayPoints = [];
        laneSwitchStartPoint = [];
        laneSwitchTargetPoint = [];
        laneSwitchStartTime = [];
        safetyGain = 80;
        comfortGain = 0.05;
        laneSwitchTime = 3;
        latOffset = 0;
        trajPolynom = {};
        trajPolynom1 = {};
        trajPolynom2 = {};
        trajPolynom2_1 = {};
        trajPolynom3_1 = {};
        trajPolynom3 = {};
        trajPolynom4 = {};
        trajPolynom5 = {};
        trajPolynom_mod = {};
        trajPolynom_fast = {};
        trajPolynom_slow = {};
        int_error_d =0;
        velocityPolynom = {};
        accPolynom = {};
        jerkPolynom = {};
        drivingMode = 0;
        referencePose = [0; 0; 0];
        adaptive_gain = 1;
        error = 0;
        curvature = 0;
        reference_vlat=0;
    end
    
    methods
        % Constructor
        function obj = VehicleKinematics_WaypointGenerator_stanley(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            %setupImpl@VehicleKinematics(obj);
            obj.vehicle = evalin('base',strcat('Vehicle',int2str(obj.Vehicle_id)));
        end
        
        
        function [poseOut, referencePose,curvature,adaptive_gain] = stepImpl(obj,pose,speed,drivingMode)
            %transfer from local coordinate obj.vehicle.dynamics.speed = v_pos(4);
            pose(3)=pose(3)*180/pi;
            obj.drivingMode = drivingMode;
            
            obj.vehicle.setPosition(obj.map.transformPoseTo3DAnim(pose));
            
            %obj.vehicle.dynamics.position = [pose(1) 0 -pose(2)];
            obj.vehicle.dynamics.orientation = [0 1 0 pose(3)-1.5*pi];
            %referenceWaypoints = obj.referenceWaypoints;
            %%
            
            %This block shouldn't run if the vehicle has reached its destination or collided
            if obj.vehicle.status.collided || obj.vehicle.pathInfo.destinationReached
                
                position= obj.vehicle.dynamics.position; %Output 1: Position of the vehicle
                rotation= obj.vehicle.dynamics.orientation; %Output 2: Rotation angle of the vehicle
                referencePose = obj.referencePose';
                poseOut=pose';
                curvature = 0;
                obj.adaptive_gain = 2/(20*obj.curvature+1);
                adaptive_gain=obj.adaptive_gain;
                obj.vehicle.dynamics.maxSpeed = 0;
                return;
                
            elseif ~obj.vehicle.pathInfo.destinationReached
                % The Vehicle hasn't reached its destination yet
                obj.vehicle.updateActualSpeed(speed); % Vehicle - Set Functions
                %                 poseOut=pose';
                if obj.vehicle.pathInfo.routeCompleted
                    % The Vehicle has completed its Route
                    nextRoute = obj.generateCurrentRoute(obj.vehicle,obj.vehicle.pathInfo.path,obj.vehicle.pathInfo.lastWaypoint);
                    currentTrajectory = obj.generateTrajectoryFromPath(obj.vehicle,obj.vehicle.pathInfo.path);
                    
                    obj.vehicle.setCurrentRoute(nextRoute);              % Vehicle - Set Functions
                    obj.vehicle.setCurrentTrajectory(currentTrajectory); % Vehicle - Set Functions
                    %                     poseOut=pose';
                end
                
                speedAccordingtoSimulation = speed*0.01*obj.simSpeed;
                %0.01 is the sample time -> obj.getSampleTime.SampleTime creates a huge overhead
                
                [~, ~] = obj.takeRoute(obj.vehicle,speedAccordingtoSimulation,obj.vehicle.pathInfo.currentTrajectory);
                %Output 1: Position of the vehicle
                %Output 2: Rotation angle of the vehicle
                
                %obj.vehicle.setPosition(position); % Vehicle - Set Functions
                %obj.vehicle.setOrientation(rotation); % Vehicle - Set Functions
                
            end
            
            %referenceWaypoints = obj.referenceWaypoints;
            
            %             figure(2)
            %             WP = plot(obj.referenceWaypoints(:,1),obj.referenceWaypoints(:,2),'.','color','blue');
            %             pos = plot(obj.vehicle.dynamics.position(1),-obj.vehicle.dynamics.position(3),'.','color','red');
            %             xlim([obj.vehicle.dynamics.position(1)-200 obj.vehicle.dynamics.position(1)+200]);
            %             ylim([-obj.vehicle.dynamics.position(3)-200 -obj.vehicle.dynamics.position(3)+200]);
            referencePose = obj.referencePose';
            poseOut=pose';
            curvature = obj.curvature;
            obj.adaptive_gain = 2/(20*obj.curvature+1);
            adaptive_gain=obj.adaptive_gain;
            obj.vehicle.dynamics.maxSpeed = sqrt(0.7*10/obj.curvature);
        end
        
        function [position, orientation] = takeRoute(obj,car,speed,refRoute)
            RotationVector = refRoute(3,:);
            if ~car.status.canLaneSwitch == 0 %lane switch
                if isempty(obj.trajPolynom)
%                     obj.generateMinJerkTrajectory_fastLaneChange(car);
%                     obj.costFunction_fastLaneChange(car);
%                     obj.generateMinJerkTrajectory_slowLaneChange(car);
%                     obj.costFunction_slowLaneChange(car);
%                     obj.generateMinJerkTrajectory(car);
%                     obj.costFunction_modLaneChange(car);
%                     obj.trajChoose(car);
                        obj.generateMinJerkTrajectory1(car);
                        obj.generateMinJerkTrajectory2(car);
                        obj.generateMinJerkTrajectory2_1(car);
                        obj.generateMinJerkTrajectory3(car);
                        obj.generateMinJerkTrajectory3_1(car);
                        obj.generateMinJerkTrajectory4(car);
                        obj.generateMinJerkTrajectory5(car);
                        obj.costFunction1(car);
                        obj.costFunction2(car);
                        obj.costFunction2_1(car);
                        obj.costFunction3(car);
                        obj.costFunction3_1(car);
                        obj.costFunction4(car);
                        obj.costFunction5(car);
                        obj.trajChoose1(car);
                end
            end
            if (RotationVector(1) == 0) %Straight motion
                [position, orientation] = obj.move_straight(car,speed,refRoute(2,:));
                
            else %Rotational motion
                rotation_angle = RotationVector(1);
                rotation_point = [RotationVector(2) 0 RotationVector(3)];
                P_final = refRoute(2,:);
                %Determine rotation direction: left or right
                if car.pathInfo.currentTrajectory(4,:) == -ones(1,3) % -1 means turn left
                    [position, orientation] = obj.rotate_left(car,speed, rotation_point,rotation_angle,P_final);
                elseif car.pathInfo.currentTrajectory(4,:) == ones(1,3) % 1 means turn right
                    [position, orientation] = obj.rotate_right(car,speed, rotation_point,rotation_angle,P_final);
                end
            end
            
        end
        
        function generateMinJerkTrajectory1(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = 2;
            %% minimum jerk trajectory
            x_f = T*car.dynamics.speed; % Final x coordinate
            if car.status.canLaneSwitch ==1
                y_f = obj.laneWidth; % Final y coordinate
            elseif car.status.canLaneSwitch ==2
                y_f = -obj.laneWidth;
            end
            obj.laneSwitchTargetPoint=[x_f 0 y_f]+obj.laneSwitchStartPoint;
            %%  Minimun jerk trajectory function for the calculation in y direction (Lateral)
            a0=0;
            a1=0;
            a2=0;
            syms a3 a4 a5;
            [a3,a4,a5]=solve([a0+a3*T^3+a4*T^4+a5*T^5==y_f, ... % Boundary condition for lateral displacement
                3*a3*T^2+4*a4*T^3+5*a5*T^4==0, ...              % Boundary condition for lateral speed
                6*a3*T+12*a4*T^2+20*a5*T^3==0,],[a3,a4,a5]);    % Boundary condition for lateral acceleration
            % Solving for coefficients and conversion to double precision
            a3=double(a3);
            a4=double(a4);
            a5=double(a5);
            obj.trajPolynom1 = num2cell([a0 a1 a2 a3 a4 a5]);
            
            
        end
        function generateMinJerkTrajectory2(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = 2.5;
            %% minimum jerk trajectory
            x_f = T*car.dynamics.speed; % Final x coordinate
            if car.status.canLaneSwitch ==1
                y_f = obj.laneWidth; % Final y coordinate
            elseif car.status.canLaneSwitch ==2
                y_f = -obj.laneWidth;
            end
            obj.laneSwitchTargetPoint=[x_f 0 y_f]+obj.laneSwitchStartPoint;
            %%  Minimun jerk trajectory function for the calculation in y direction (Lateral)
            a0=0;
            a1=0;
            a2=0;
            syms a3 a4 a5;
            [a3,a4,a5]=solve([a0+a3*T^3+a4*T^4+a5*T^5==y_f, ... % Boundary condition for lateral displacement
                3*a3*T^2+4*a4*T^3+5*a5*T^4==0, ...              % Boundary condition for lateral speed
                6*a3*T+12*a4*T^2+20*a5*T^3==0,],[a3,a4,a5]);    % Boundary condition for lateral acceleration
            % Solving for coefficients and conversion to double precision
            a3=double(a3);
            a4=double(a4);
            a5=double(a5);
            obj.trajPolynom2 = num2cell([a0 a1 a2 a3 a4 a5]);
            
            
        end
        function generateMinJerkTrajectory2_1(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = 2.75;
            %% minimum jerk trajectory
            x_f = T*car.dynamics.speed; % Final x coordinate
            if car.status.canLaneSwitch ==1
                y_f = obj.laneWidth; % Final y coordinate
            elseif car.status.canLaneSwitch ==2
                y_f = -obj.laneWidth;
            end
            obj.laneSwitchTargetPoint=[x_f 0 y_f]+obj.laneSwitchStartPoint;
            %%  Minimun jerk trajectory function for the calculation in y direction (Lateral)
            a0=0;
            a1=0;
            a2=0;
            syms a3 a4 a5;
            [a3,a4,a5]=solve([a0+a3*T^3+a4*T^4+a5*T^5==y_f, ... % Boundary condition for lateral displacement
                3*a3*T^2+4*a4*T^3+5*a5*T^4==0, ...              % Boundary condition for lateral speed
                6*a3*T+12*a4*T^2+20*a5*T^3==0,],[a3,a4,a5]);    % Boundary condition for lateral acceleration
            % Solving for coefficients and conversion to double precision
            a3=double(a3);
            a4=double(a4);
            a5=double(a5);
            obj.trajPolynom2_1 = num2cell([a0 a1 a2 a3 a4 a5]);
            
            
        end
        function generateMinJerkTrajectory3(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = 3;
            %% minimum jerk trajectory
            x_f = T*car.dynamics.speed; % Final x coordinate
            if car.status.canLaneSwitch ==1
                y_f = obj.laneWidth; % Final y coordinate
            elseif car.status.canLaneSwitch ==2
                y_f = -obj.laneWidth;
            end
            obj.laneSwitchTargetPoint=[x_f 0 y_f]+obj.laneSwitchStartPoint;
            %%  Minimun jerk trajectory function for the calculation in y direction (Lateral)
            a0=0;
            a1=0;
            a2=0;
            syms a3 a4 a5;
            [a3,a4,a5]=solve([a0+a3*T^3+a4*T^4+a5*T^5==y_f, ... % Boundary condition for lateral displacement
                3*a3*T^2+4*a4*T^3+5*a5*T^4==0, ...              % Boundary condition for lateral speed
                6*a3*T+12*a4*T^2+20*a5*T^3==0,],[a3,a4,a5]);    % Boundary condition for lateral acceleration
            % Solving for coefficients and conversion to double precision
            a3=double(a3);
            a4=double(a4);
            a5=double(a5);
            obj.trajPolynom3 = num2cell([a0 a1 a2 a3 a4 a5]);
            
            
        end
        function generateMinJerkTrajectory3_1(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = 3.25;
            %% minimum jerk trajectory
            x_f = T*car.dynamics.speed; % Final x coordinate
            if car.status.canLaneSwitch ==1
                y_f = obj.laneWidth; % Final y coordinate
            elseif car.status.canLaneSwitch ==2
                y_f = -obj.laneWidth;
            end
            obj.laneSwitchTargetPoint=[x_f 0 y_f]+obj.laneSwitchStartPoint;
            %%  Minimun jerk trajectory function for the calculation in y direction (Lateral)
            a0=0;
            a1=0;
            a2=0;
            syms a3 a4 a5;
            [a3,a4,a5]=solve([a0+a3*T^3+a4*T^4+a5*T^5==y_f, ... % Boundary condition for lateral displacement
                3*a3*T^2+4*a4*T^3+5*a5*T^4==0, ...              % Boundary condition for lateral speed
                6*a3*T+12*a4*T^2+20*a5*T^3==0,],[a3,a4,a5]);    % Boundary condition for lateral acceleration
            % Solving for coefficients and conversion to double precision
            a3=double(a3);
            a4=double(a4);
            a5=double(a5);
            obj.trajPolynom3_1 = num2cell([a0 a1 a2 a3 a4 a5]);
            
            
        end
        function generateMinJerkTrajectory4(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = 3.5;
            %% minimum jerk trajectory
            x_f = T*car.dynamics.speed; % Final x coordinate
            if car.status.canLaneSwitch ==1
                y_f = obj.laneWidth; % Final y coordinate
            elseif car.status.canLaneSwitch ==2
                y_f = -obj.laneWidth;
            end
            obj.laneSwitchTargetPoint=[x_f 0 y_f]+obj.laneSwitchStartPoint;
            %%  Minimun jerk trajectory function for the calculation in y direction (Lateral)
            a0=0;
            a1=0;
            a2=0;
            syms a3 a4 a5;
            [a3,a4,a5]=solve([a0+a3*T^3+a4*T^4+a5*T^5==y_f, ... % Boundary condition for lateral displacement
                3*a3*T^2+4*a4*T^3+5*a5*T^4==0, ...              % Boundary condition for lateral speed
                6*a3*T+12*a4*T^2+20*a5*T^3==0,],[a3,a4,a5]);    % Boundary condition for lateral acceleration
            % Solving for coefficients and conversion to double precision
            a3=double(a3);
            a4=double(a4);
            a5=double(a5);
            obj.trajPolynom4 = num2cell([a0 a1 a2 a3 a4 a5]);
            
            
        end
        function generateMinJerkTrajectory5(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = 4;
            %% minimum jerk trajectory
            x_f = T*car.dynamics.speed; % Final x coordinate
            if car.status.canLaneSwitch ==1
                y_f = obj.laneWidth; % Final y coordinate
            elseif car.status.canLaneSwitch ==2
                y_f = -obj.laneWidth;
            end
            obj.laneSwitchTargetPoint=[x_f 0 y_f]+obj.laneSwitchStartPoint;
            %%  Minimun jerk trajectory function for the calculation in y direction (Lateral)
            a0=0;
            a1=0;
            a2=0;
            syms a3 a4 a5;
            [a3,a4,a5]=solve([a0+a3*T^3+a4*T^4+a5*T^5==y_f, ... % Boundary condition for lateral displacement
                3*a3*T^2+4*a4*T^3+5*a5*T^4==0, ...              % Boundary condition for lateral speed
                6*a3*T+12*a4*T^2+20*a5*T^3==0,],[a3,a4,a5]);    % Boundary condition for lateral acceleration
            % Solving for coefficients and conversion to double precision
            a3=double(a3);
            a4=double(a4);
            a5=double(a5);
            obj.trajPolynom5 = num2cell([a0 a1 a2 a3 a4 a5]);
            
            
        end
        function costFunction1(obj,car)
            [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom1{:});
            t=0:0.01:2;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/2;
            ttc_min = 1.4*obj.vehicle.decisionUnit.LaneSwitchTime+0-2/2;
            obj.vehicle.dataLog.costFunction1=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
        function costFunction2(obj,car)
            [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom2{:});
            t=0:0.01:2.5;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/2.5;
            ttc_min = 1.4*obj.vehicle.decisionUnit.LaneSwitchTime+0-2.5/2;
            obj.vehicle.dataLog.costFunction2=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
        function costFunction2_1(obj,car)
            [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom2_1{:});
            t=0:0.01:2.75;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/2.75;
            ttc_min = 1.4*obj.vehicle.decisionUnit.LaneSwitchTime+0-2.75/2;
            obj.vehicle.dataLog.costFunction2_1=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
        function costFunction3(obj,car)
            [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom3{:});
            t=0:0.01:3;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/3;
            ttc_min = 1.4*obj.vehicle.decisionUnit.LaneSwitchTime+0-3/2;
            obj.vehicle.dataLog.costFunction3=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
        function costFunction3_1(obj,car)
            [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom3_1{:});
            t=0:0.01:3.25;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/3.25;
            ttc_min = 1.4*obj.vehicle.decisionUnit.LaneSwitchTime+0-3.25/2;
            obj.vehicle.dataLog.costFunction3_1=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
        function costFunction4(obj,car)
            [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom4{:});
            t=0:0.01:3.5;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/3.5;
            ttc_min = 1.4*obj.vehicle.decisionUnit.LaneSwitchTime+0-3.5/2;
            obj.vehicle.dataLog.costFunction4=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
        function costFunction5(obj,car)
            [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom5{:});
            t=0:0.01:4;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/4;
            ttc_min = 1.4*obj.vehicle.decisionUnit.LaneSwitchTime+0-4/2;
            obj.vehicle.dataLog.costFunction5=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
        function generateMinJerkTrajectory(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = obj.vehicle.decisionUnit.LaneSwitchTime;
            %% minimum jerk trajectory
            x_f = T*car.dynamics.speed; % Final x coordinate
            if car.status.canLaneSwitch ==1
                y_f = obj.laneWidth; % Final y coordinate
            elseif car.status.canLaneSwitch ==2
                y_f = -obj.laneWidth;
            end
            obj.laneSwitchTargetPoint=[x_f 0 y_f]+obj.laneSwitchStartPoint;
            %%  Minimun jerk trajectory function for the calculation in y direction (Lateral)
            a0=0;
            a1=0;
            a2=0;
            syms a3 a4 a5;
            [a3,a4,a5]=solve([a0+a3*T^3+a4*T^4+a5*T^5==y_f, ... % Boundary condition for lateral displacement
                3*a3*T^2+4*a4*T^3+5*a5*T^4==0, ...              % Boundary condition for lateral speed
                6*a3*T+12*a4*T^2+20*a5*T^3==0,],[a3,a4,a5]);    % Boundary condition for lateral acceleration
            % Solving for coefficients and conversion to double precision
            a3=double(a3);
            a4=double(a4);
            a5=double(a5);
            obj.trajPolynom_mod = num2cell([a0 a1 a2 a3 a4 a5]);
            car.dataLog.MinJerkTrajPolynom = {car.dataLog.MinJerkTrajPolynom;obj.trajPolynom};
            obj.velocityPolynom = num2cell([a0 a1 2*a2 3*a3 4*a4 5*a5]);
            obj.accPolynom = num2cell([a0 a1 2*a2 6*a3 12*a4 20*a5]);
            obj.jerkPolynom = num2cell([a0 a1 a2 6*a3 24*a4 60*a5]);
            t=0:0.01:obj.vehicle.decisionUnit.LaneSwitchTime;
            x=car.dynamics.speed*t;
            y=a0+a1*t+a2*t.^2+a3*t.^3+a4*t.^4+a5*t.^5;
            %             y_dot=a1+2*a2*t+3*a3*t^2+4*a4*t^3+5*a5*t^4;
            %             y_ddot=2*a2+6*a3*t+12*a4*t^2+20*a5*t^3;
            %             y_dddot = 6*a3+24*a4*t+60*a5*t^2;
            obj.laneSwitchWayPoints = [x' y'];
            
        end
        function generateMinJerkTrajectory_fastLaneChange(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = obj.vehicle.decisionUnit.LaneSwitchTime*0.5;
            %% minimum jerk trajectory
            x_f = T*car.dynamics.speed; % Final x coordinate
            if car.status.canLaneSwitch ==1
                y_f = obj.laneWidth; % Final y coordinate
            elseif car.status.canLaneSwitch ==2
                y_f = -obj.laneWidth;
            end
            obj.laneSwitchTargetPoint=[x_f 0 y_f]+obj.laneSwitchStartPoint;
            %%  Minimun jerk trajectory function for the calculation in y direction (Lateral)
            a0=0;
            a1=0;
            a2=0;
            syms a3 a4 a5;
            [a3,a4,a5]=solve([a0+a3*T^3+a4*T^4+a5*T^5==y_f, ... % Boundary condition for lateral displacement
                3*a3*T^2+4*a4*T^3+5*a5*T^4==0, ...              % Boundary condition for lateral speed
                6*a3*T+12*a4*T^2+20*a5*T^3==0,],[a3,a4,a5]);    % Boundary condition for lateral acceleration
            % Solving for coefficients and conversion to double precision
            a3=double(a3);
            a4=double(a4);
            a5=double(a5);
            obj.trajPolynom_fast = num2cell([a0 a1 a2 a3 a4 a5]);
            car.dataLog.MinJerkTrajPolynom_fast = {car.dataLog.MinJerkTrajPolynom_fast;obj.trajPolynom};
            %             obj.velocityPolynom = num2cell([a0 a1 2*a2 3*a3 4*a4 5*a5]);
            %             obj.accPolynom = num2cell([a0 a1 2*a2 6*a3 12*a4 20*a5]);
            %             obj.jerkPolynom = num2cell([a0 a1 a2 6*a3 24*a4 60*a5]);
            %             t=0:0.01:obj.vehicle.decisionUnit.LaneSwitchTime;
            %             x=car.dynamics.speed*t;
            %             y=a0+a1*t+a2*t.^2+a3*t.^3+a4*t.^4+a5*t.^5;
            %             %             y_dot=a1+2*a2*t+3*a3*t^2+4*a4*t^3+5*a5*t^4;
            %             %             y_ddot=2*a2+6*a3*t+12*a4*t^2+20*a5*t^3;
            %             %             y_dddot = 6*a3+24*a4*t+60*a5*t^2;
            %             obj.laneSwitchWayPoints = [x' y'];
            
        end
        function generateMinJerkTrajectory_slowLaneChange(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = obj.vehicle.decisionUnit.LaneSwitchTime*2;
            %% minimum jerk trajectory
            x_f = T*car.dynamics.speed; % Final x coordinate
            if car.status.canLaneSwitch ==1
                y_f = obj.laneWidth; % Final y coordinate
            elseif car.status.canLaneSwitch ==2
                y_f = -obj.laneWidth;
            end
            obj.laneSwitchTargetPoint=[x_f 0 y_f]+obj.laneSwitchStartPoint;
            %%  Minimun jerk trajectory function for the calculation in y direction (Lateral)
            a0=0;
            a1=0;
            a2=0;
            syms a3 a4 a5;
            [a3,a4,a5]=solve([a0+a3*T^3+a4*T^4+a5*T^5==y_f, ... % Boundary condition for lateral displacement
                3*a3*T^2+4*a4*T^3+5*a5*T^4==0, ...              % Boundary condition for lateral speed
                6*a3*T+12*a4*T^2+20*a5*T^3==0,],[a3,a4,a5]);    % Boundary condition for lateral acceleration
            % Solving for coefficients and conversion to double precision
            a3=double(a3);
            a4=double(a4);
            a5=double(a5);
            obj.trajPolynom_slow = num2cell([a0 a1 a2 a3 a4 a5]);
            car.dataLog.MinJerkTrajPolynom_slow = {car.dataLog.MinJerkTrajPolynom_slow;obj.trajPolynom};
            %             obj.velocityPolynom = num2cell([a0 a1 2*a2 3*a3 4*a4 5*a5]);
            %             obj.accPolynom = num2cell([a0 a1 2*a2 6*a3 12*a4 20*a5]);
            %             obj.jerkPolynom = num2cell([a0 a1 a2 6*a3 24*a4 60*a5]);
            %             t=0:0.01:obj.vehicle.decisionUnit.LaneSwitchTime;
            %             x=car.dynamics.speed*t;
            %             y=a0+a1*t+a2*t.^2+a3*t.^3+a4*t.^4+a5*t.^5;
            %             %             y_dot=a1+2*a2*t+3*a3*t^2+4*a4*t^3+5*a5*t^4;
            %             %             y_ddot=2*a2+6*a3*t+12*a4*t^2+20*a5*t^3;
            %             %             y_dddot = 6*a3+24*a4*t+60*a5*t^2;
            %             obj.laneSwitchWayPoints = [x' y'];
            
        end
        function costFunction_fastLaneChange(obj,car)
            [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom_fast{:});
            t=0:0.01:obj.vehicle.decisionUnit.LaneSwitchTime*0.5;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/obj.vehicle.decisionUnit.LaneSwitchTime*0.5;
            ttc_min = 2.1*obj.vehicle.decisionUnit.LaneSwitchTime+0-obj.vehicle.decisionUnit.LaneSwitchTime*0.5;
            obj.vehicle.dataLog.costFunction_fastLaneChange=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
        function costFunction_slowLaneChange(obj,car)
            [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom_slow{:});
            t=0:0.01:obj.vehicle.decisionUnit.LaneSwitchTime*2;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/obj.vehicle.decisionUnit.LaneSwitchTime*2;
            ttc_min = 2.1*obj.vehicle.decisionUnit.LaneSwitchTime+0-obj.vehicle.decisionUnit.LaneSwitchTime*2;
            obj.vehicle.dataLog.costFunction_slowLaneChange=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
        function costFunction_modLaneChange(obj,car)
            [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom_mod{:});
            t=0:0.01:obj.vehicle.decisionUnit.LaneSwitchTime;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/obj.vehicle.decisionUnit.LaneSwitchTime;
            ttc_min = 2.1*obj.vehicle.decisionUnit.LaneSwitchTime+0-obj.vehicle.decisionUnit.LaneSwitchTime;
            obj.vehicle.dataLog.costFunction_modLaneChange=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
        function trajChoose(obj,car)
            obj.trajPolynom = obj.trajPolynom_mod;
            obj.laneSwitchTime = obj.vehicle.decisionUnit.LaneSwitchTime;
            if(car.dataLog.costFunction_fastLaneChange<car.dataLog.costFunction_modLaneChange)
                obj.trajPolynom = obj.trajPolynom_fast;
                obj.laneSwitchTime = obj.vehicle.decisionUnit.LaneSwitchTime*0.5;
            end
            if(car.dataLog.costFunction_slowLaneChange<car.dataLog.costFunction_fastLaneChange)
                obj.trajPolynom = obj.trajPolynom_slow;
                obj.laneSwitchTime = obj.vehicle.decisionUnit.LaneSwitchTime*2;
            end
            
        end
        function trajChoose1(obj,car)
            %find the trajectory with minimum cost function value
            A = [car.dataLog.costFunction1 car.dataLog.costFunction2 car.dataLog.costFunction2_1 car.dataLog.costFunction3 car.dataLog.costFunction3_1 car.dataLog.costFunction4 car.dataLog.costFunction5];
            ind = find(A==min(min(A)));
            if(ind==1)
                obj.trajPolynom = obj.trajPolynom1;
                obj.laneSwitchTime = 2;
            end
            if(ind==2)
                obj.trajPolynom = obj.trajPolynom2;
                obj.laneSwitchTime = 2.5;
            end
            if(ind==3)
                obj.trajPolynom = obj.trajPolynom2_1;
                obj.laneSwitchTime = 2.75;
            end
            if(ind==4)
                obj.trajPolynom = obj.trajPolynom3;
                obj.laneSwitchTime = 3;
            end
            if(ind==5)
                obj.trajPolynom = obj.trajPolynom3_1;
                obj.laneSwitchTime = 3.25;
            end
            if(ind==6)
                obj.trajPolynom = obj.trajPolynom4;
                obj.laneSwitchTime = 3.5;
            end
            if(ind==7)
                obj.trajPolynom = obj.trajPolynom5;
                obj.laneSwitchTime = 4;
            end
            obj.trajPolynom = obj.trajPolynom3;
                obj.laneSwitchTime = 3;
        end
        function [position, orientation] = move_straight(obj,car,speed,Destination)
            %% Reference Waypoint Generation
            obj.generateStraightWaypoints(car)
            %%
            %Displacement Vector and determination of its Angle
            if car.pathInfo.routeCompleted == true
                DisplacementVector = Destination- car.dynamics.position;
                ThetaRadian = atan(DisplacementVector(1)/DisplacementVector(3));
                car.dynamics.orientation(4) = ThetaRadian;
                car.dynamics.directionVector = DisplacementVector;
                car.setRouteCompleted(false);
            end
            
            % For Intercardinal directions: Depending on the four quadrants, the problem the atan function is that it
            % is between -pi and pi so that it doesn't correctly cover a 360 degrees
            % angle, therefore the quadrant check and pi addition must take place to
            % make sure that the vehicle rotates correctly
            ThetaRadian = car.dynamics.orientation(4);
            
            % Intercardinal Directions
            if car.dynamics.directionVector(1)<0
                if car.dynamics.directionVector(3)<0
                    ThetaRadian= ThetaRadian+pi;
                end
            elseif car.dynamics.directionVector(1)>0
                if car.dynamics.directionVector(3)<0
                    ThetaRadian= ThetaRadian+pi;
                end
                
                % Vertical and horizontal movements - Cardinal Directions
                % Movement z direction
            elseif car.dynamics.directionVector(1)==0
                
                % Positive z direction
                if car.dynamics.directionVector(3)>0
                    ThetaRadian = 0;
                    
                    % Negative z direction
                elseif car.dynamics.directionVector(3)<0
                    ThetaRadian =-pi;
                end
                
                % Movement x direction
            elseif car.dynamics.directionVector(3)==0
                % Positive x direction
                if car.dynamics.directionVector(1)>0
                    ThetaRadian= pi/2;
                    % Negative x direction
                elseif car.dynamics.directionVector(3)<0
                    ThetaRadian= -pi/2;
                end
                
            end
            
            %             [checkPoint_x,checkPoint_y]=get_vehicle_checkpoint(obj,car);
            %             checkPoint = [checkPoint_x 0 checkPoint_y];
            %
            %             %if  norm(car.dynamics.directionVector/norm(car.dynamics.directionVector))*speed > norm(Destination-car.dynamics.position)
            %             if norm(checkPoint-Destination)< 1+car.pathInfo.laneId*obj.laneWidth % Error tolerance value TODO: check lower numbers
            %
            if car.pathInfo.routeEndDistance <1
                
                car.pathInfo.s = 0;
                
                lastWaypoint = car.map.get_waypoint_from_coordinates(Destination);
                
                car.setRouteCompleted(true); % Vehicle Set
                car.setLastWaypoint(lastWaypoint); % Vehicle Set
                
                nextRoute = obj.generateCurrentRoute(car,car.pathInfo.path,lastWaypoint);
                car.setCurrentRoute(nextRoute); % Vehicle Set
            end
            
            orientation = [ 0 1 0 ThetaRadian];
            % Simple Straight Motion Equation
            position = car.dynamics.position + (car.dynamics.directionVector/norm(car.dynamics.directionVector))*(speed);
        end
        
        function [position, orientation] = rotate_left(obj ,car, speed, rotation_point,rotation_angle,Destination)
            %% Reference Waypoint Generation
            obj.generateLeftRotationWaypoints(car);
            %%
            if car.pathInfo.routeCompleted == true
                
                car.dynamics.cornering.angles = pi; % Vehicle Set
                car.setRouteCompleted(false); % Vehicle Set
                
                point_to_rotate= car.dynamics.position;
                
                car.dynamics.cornering.a=point_to_rotate(1)-rotation_point(1);
                car.dynamics.cornering.b=point_to_rotate(2)-rotation_point(2);
                car.dynamics.cornering.c=point_to_rotate(3)-rotation_point(3);
            end
            
            r = norm(Destination-rotation_point);
            vector_z=[0 0 1];
            
            a = car.dynamics.cornering.a;
            b = car.dynamics.cornering.b;
            c = car.dynamics.cornering.c;
            
            step_length = speed/r;
            car.setRotationAngle(-step_length) % Vehicle Set
            t = car.dynamics.cornering.angles;
            
            %             [checkPoint_x,checkPoint_y]=get_vehicle_checkpoint(obj,car);
            %             checkPoint = [checkPoint_x 0 checkPoint_y];
            %
            %             if  norm(checkPoint - Destination) < 1+car.pathInfo.laneId*obj.laneWidth % TODO Check the value
            %
            if car.pathInfo.routeEndDistance <1
                car.pathInfo.s = 0;
                
                lastWaypoint = car.map.get_waypoint_from_coordinates(Destination);
                
                car.setRouteCompleted(true);% Vehicle Set
                car.setLastWaypoint(lastWaypoint); % Vehicle Set
                
                nextRoute = obj.generateCurrentRoute(car,car.pathInfo.path,lastWaypoint);
                car.setCurrentRoute(nextRoute); % Vehicle Set
                
                car.dynamics.cornering.angles = 0;
                
                position = Destination;
                orientation = car.dynamics.orientation;
            else
                vector_velocity=[-a*sin(t)-cos(t)*c b a*cos(t)-c*sin(t)];
                vector=cross(vector_velocity, vector_z);
                vector=vector/norm(vector);
                theta=acos(dot(vector_velocity, vector_z)/(norm(vector_velocity)*norm(vector_z)));
                
                
                position = [rotation_point(1)-(a*cos(t)-sin(t)*c) rotation_point(2)+b*t rotation_point(3)-(a*sin(t)+c*cos(t))];
                orientation = [vector -theta];
            end
            
        end
        
        function [position, orientation] = rotate_right(obj ,car,speed, rotation_point,rotation_angle,Destination)
            %% Reference Waypoint Generation
            obj.generateRightRotationWaypoints(car);
            %%
            if car.pathInfo.routeCompleted == true
                
                car.setRouteCompleted(false);
                
                car.setCorneringValues(car.dynamics.position, rotation_point)
            end
            
            r = norm(Destination-rotation_point);
            vector_z=[0 0 1];
            
            a = car.dynamics.cornering.a;
            b = car.dynamics.cornering.b;
            c = car.dynamics.cornering.c;
            
            step_length = speed/r;
            car.setRotationAngle(step_length) % Vehicle Set
            
            t = car.dynamics.cornering.angles;
            
            %             [checkPoint_x,checkPoint_y]=get_vehicle_checkpoint(obj,car);
            %             checkPoint = [checkPoint_x 0 checkPoint_y];
            %
            %             if  norm(checkPoint - Destination) < 1+car.pathInfo.laneId*obj.laneWidth % TODO Check the value
            if car.pathInfo.routeEndDistance <1
                car.pathInfo.s = 0;
                
                lastWaypoint = car.map.get_waypoint_from_coordinates(Destination);
                
                car.setRouteCompleted(true);% Vehicle Set
                car.setLastWaypoint(lastWaypoint); % Vehicle Set
                
                nextRoute = obj.generateCurrentRoute(car,car.pathInfo.path,lastWaypoint);
                car.setCurrentRoute(nextRoute);
                
                car.dynamics.cornering.angles = 0;
                
                position = Destination;
                orientation = car.dynamics.orientation;
            else
                vector_velocity=[-a*sin(t)-cos(t)*c b a*cos(t)-c*sin(t)];
                vector=cross(vector_velocity, vector_z);
                vector=vector/norm(vector);
                theta=acos(dot(vector_velocity, vector_z)/(norm(vector_velocity)*norm(vector_z)));
                
                position = [rotation_point(1)+(a*cos(t)-sin(t)*c) rotation_point(2)+b*t rotation_point(3)+(a*sin(t)+c*cos(t))];
                orientation =[vector -theta];
            end
            
            
        end
        
        function generateStraightWaypoints(obj,car)
            
            route = car.pathInfo.currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];
            position_C = car.dynamics.position([1,3]).*[1 -1];
            radian = car.pathInfo.currentTrajectory(3,1);
            [s,vehicle_d,orientation_C,routeLength] = obj.Cartesian2Frenet(route,position_C,radian);
            car.pathInfo.s = s;
            car.pathInfo.routeEndDistance = routeLength-s;
            if(~isempty(obj.trajPolynom))
                t=get_param('MOBATSim','SimulationTime')-obj.laneSwitchStartTime;
                [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom{:});
                if t<=obj.laneSwitchTime
                    obj.latOffset = a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5;
                    obj.reference_vlat= a1+2*a2+3*a3*t^2+4*a4^t^3+5*a5*t^4;
                else
                    obj.latOffset = 0;
                    obj.reference_vlat = 0;
                    car.status.laneSwitchFinish = 1;
                    if car.status.canLaneSwitch ==1
                        car.pathInfo.laneId = car.pathInfo.laneId+1;
                    elseif car.status.canLaneSwitch ==2
                        car.pathInfo.laneId = car.pathInfo.laneId-1;
                    end
                    car.status.canLaneSwitch = 0;
                    car.dataLog.laneSwitchEndTime=[car.dataLog.laneSwitchEndTime get_param('MOBATSim','SimulationTime')];
                    obj.trajPolynom={};
                    
                end
            end
            
            d=obj.laneWidth*(car.pathInfo.laneId-0.5)+obj.latOffset;
            obj.error = d-vehicle_d;
            
            obj.int_error_d = obj.int_error_d+obj.error*0.01;
            %             reference_yaw = atan2(obj.reference_vlat,car.dynamics.speed);
            [targetPosition_C,~] = obj.Frenet2Cartesian(route,s,d,radian);
            obj.referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];
        end
        
        function generateLeftRotationWaypoints(obj,car)
            route = car.pathInfo.currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];
            position_C = car.dynamics.position([1,3]).*[1 -1];
            radian = abs(car.pathInfo.currentTrajectory(3,1));
            [s,vehicle_d,orientation_C,routeLength] = obj.Cartesian2Frenet(route,position_C,radian);
            car.pathInfo.s = s;
            car.pathInfo.routeEndDistance = routeLength-s;
            if(~isempty(obj.trajPolynom))
                t=get_param('MOBATSim','SimulationTime')-obj.laneSwitchStartTime;
                [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom{:});
                if t<=obj.laneSwitchTime
                    obj.latOffset = a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5;
                else
                    obj.latOffset = 0;
                    car.status.laneSwitchFinish = 1;
                    if car.status.canLaneSwitch ==1
                        car.pathInfo.laneId = car.pathInfo.laneId+1;
                    elseif car.status.canLaneSwitch ==2
                        car.pathInfo.laneId = car.pathInfo.laneId-1;
                    end
                    car.status.canLaneSwitch = 0;
                    obj.trajPolynom={};
                end
            end
            
            d=-(obj.laneWidth*(car.pathInfo.laneId-0.5)+obj.latOffset);
            obj.error = d-vehicle_d;
            obj.int_error_d = obj.int_error_d+obj.error*0.01;
            [targetPosition_C,~] = obj.Frenet2Cartesian(route,s,d,radian);
            obj.referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];
        end
        
        function generateRightRotationWaypoints(obj,car)
            route = car.pathInfo.currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];
            position_C = car.dynamics.position([1,3]).*[1 -1];
            radian = -abs(car.pathInfo.currentTrajectory(3,1));
            [s,vehicle_d,orientation_C,routeLength] = obj.Cartesian2Frenet(route,position_C,radian);
            car.pathInfo.s = s;
            car.pathInfo.routeEndDistance = routeLength-s;
            if(~isempty(obj.trajPolynom))
                t=get_param('MOBATSim','SimulationTime')-obj.laneSwitchStartTime;
                [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom{:});
                if t<=obj.laneSwitchTime
                    obj.latOffset = a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5;
                else
                    obj.latOffset = 0;
                    car.status.laneSwitchFinish = 1;
                    if car.status.canLaneSwitch ==1
                        car.pathInfo.laneId = car.pathInfo.laneId+1;
                    elseif car.status.canLaneSwitch ==2
                        car.pathInfo.laneId = car.pathInfo.laneId-1;
                    end
                    car.status.canLaneSwitch = 0;
                    obj.trajPolynom={};
                end
            end
            
            d=obj.laneWidth*(car.pathInfo.laneId-0.5)+obj.latOffset;
            obj.error = d-vehicle_d;
            obj.int_error_d = obj.int_error_d+obj.error*0.01;
            [targetPosition_C,~] = obj.Frenet2Cartesian(route,s,d,radian);
            obj.referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];
            
        end
        
        %         function [x,y] = get_vehicle_checkpoint(obj,car)
        %             position = car.dynamics.position;%read vehicle position Info
        %             orientation = car.dynamics.orientation(4)+1.5*pi; %read vehicle yaw Info and transfer it to world frame
        %             checkPoint_offset = 0.5;%determine the offset distance between vehicle CoG and checkpoint
        %             x=position(1)-checkPoint_offset*cos(orientation);
        %             y=position(3)-checkPoint_offset*sin(orientation);
        %         end
        
        function [position_C,orientation_C] = Frenet2Cartesian(obj,route,s,d,radian)
            
            %this function transfer a position in Frenet coordinate into Cartesian coordinate
            %input:
            %route is a 2x2 array [x_s y_s;x_e y_e]contains the startpoint and the endpoint of the road
            %s is the journey on the reference roadline(d=0)
            %d is the vertical offset distance to the reference roadline,positive d means away from center
            %radian is the radian of the whole curved road,is positive when
            %counterclockwise turns
            %output:
            %position_C is the 1x2 array [x y] in Cartesian coordinate
            %orientation_C is the angle of the tangent vector on the reference roadline
            
            startPoint = route(1,:);
            endPoint = route(2,:);
            if radian == 0
                route_Vector = endPoint-startPoint;
                local_route_Vector_i = route_Vector/norm(route_Vector);
                orientation_C = atan2(local_route_Vector_i(2),local_route_Vector_i(1));
                sideVector = [cos(orientation_C+pi/2) sin(orientation_C+pi/2)];
                position_C = s*local_route_Vector_i+d*sideVector+startPoint;
            else
                r = sqrt((norm(endPoint-startPoint))^2/(1-cos(radian))/2);
                targetVector = (endPoint-startPoint)/norm(endPoint-startPoint);
                beta = atan2(targetVector(2),targetVector(1));
                plumbLength = cos(radian/2)*r;
                plumbVector = [cos(beta+sign(radian)*pi/2) sin(beta+sign(radian)*pi/2)]*plumbLength;
                center = startPoint + targetVector*norm(endPoint-startPoint)/2 + plumbVector;
                startPointVector = startPoint-center;
                startPointVectorAng = atan2(startPointVector(2),startPointVector(1));
                l = r+d;
                lAng = sign(radian)*s/r+startPointVectorAng;
                position_C = l*[cos(lAng) sin(lAng)]+center;
                orientation_C = lAng+sign(radian)*pi/2;
                orientation_C = mod(orientation_C,2*pi);
                orientation_C = orientation_C.*(0<=orientation_C & orientation_C <= pi) + (orientation_C - 2*pi).*(pi<orientation_C & orientation_C<2*2*pi);   % angle in (-pi,pi]
            end
        end
        
        function [s,d,orientation_C,routeLength] = Cartesian2Frenet(obj,route,position_C,radian)
            
            %this function transform a position in Cartesian coordinate into Frenet coordinate
            
            %input:
            %route is a 2x2 array [x_s y_s;x_e y_e]contains the startpoint and the endpoint of the road
            %position_C is the 1x2 array [x y] in Cartesian coordinate
            %radian is the radian of the whole curved road,is positive when
            %counterclockwise turn
            
            %output:
            %orientation_C is the angle of the tangent vector on the reference roadline(d=0)
            %s is the journey on the reference roadline
            %d is the vertical offset distance to the reference roadline,positive d means away from center
            
            startPoint = route(1,:);
            endPoint = route(2,:);
            if radian == 0
                route_Vector = endPoint-startPoint;
                local_route_Vector_i = route_Vector/norm(route_Vector);
                orientation_C = atan2(local_route_Vector_i(2),local_route_Vector_i(1));
                posVector = position_C-startPoint;
                s = dot(posVector,local_route_Vector_i);
                sideVector = [cos(orientation_C+pi/2) sin(orientation_C+pi/2)];
                d = dot(posVector,sideVector);
                routeLength = norm(endPoint-startPoint);
                obj.curvature = 0;
            else
                r = sqrt((norm(endPoint-startPoint))^2/(1-cos(radian))/2);
                targetVector = (endPoint-startPoint)/norm(endPoint-startPoint);
                beta = atan2(targetVector(2),targetVector(1));
                plumbLength = cos(radian/2)*r;
                plumbVector = [cos(beta+sign(radian)*pi/2) sin(beta+sign(radian)*pi/2)]*plumbLength;
                center = startPoint + targetVector*norm(endPoint-startPoint)/2 + plumbVector;
                startPointVector = startPoint-center;
                %                 startPointVectorAng = atan2(startPointVector(2),startPointVector(1));
                %                 startPointVectorAng2pi = mod(startPointVectorAng,2*pi);
                l = position_C-center;
                d = norm(l)-r;
                lAng = atan2(l(2),l(1));
                %                 lAng2pi = mod(lAng,2*pi);
                %                 s = sign(radian)*(lAng2pi-startPointVectorAng2pi)*r;
                start_dot_l = dot(startPointVector,l);
                start_cross_l = sign(radian)*(startPointVector(1)*l(2)-startPointVector(2)*l(1));
                angle = atan2(start_cross_l,start_dot_l);
                if mod(angle,2*pi) > abs(radian)
                    start_cross_l = -(startPointVector(1)*l(2)-startPointVector(2)*l(1));
                    angle = -atan2(start_cross_l,start_dot_l);
                end
                s = angle*r;
                routeLength = abs(radian)*r;
                orientation_C = lAng+sign(radian)*pi/2;
                orientation_C = mod(orientation_C,2*pi);
                orientation_C = orientation_C.*(0<=orientation_C & orientation_C <= pi) + (orientation_C - 2*pi).*(pi<orientation_C & orientation_C<2*2*pi);   % angle in (-pi,pi]
                obj.curvature = 1/r;
            end
        end
        
    end
    %% Standard Simulink Output functions
    methods(Static,Access = protected)
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon("Vehicle.png");
        end
        
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
        
        function [out, out2, out3,out4] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 3];
            out2 = [1 3];
            out3 = [1 1];
            out4 = [1 1];
            
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
        
        function [out,out2, out3,out4] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = 'double';
            out2 = 'double';
            out3 = 'double';
            out4 = 'double';
            
            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end
        
        function [out, out2, out3,out4] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
            out4 = false;
            
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
        
        function [out, out2, out3,out4] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
            out4 = true;
            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
        
        function sts = getSampleTimeImpl(obj)
            % Define sample time type and parameters
            sts = obj.createSampleTime("Type", "Inherited");
        end
    end
    
end
