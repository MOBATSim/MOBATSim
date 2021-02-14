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
        trajPolynom05 = {};
        trajPolynom075 = {};
        trajPolynom09 = {};
        trajPolynom10 = {};
        trajPolynom11 = {};
        trajPolynom125 = {};
        trajPolynom15 = {};
        
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
        
        
        function [poseOut, referencePose] = stepImpl(obj,pose,speed,drivingMode)
            %transfer from local coordinate obj.vehicle.dynamics.speed = v_pos(4);
            pose(3)=pose(3)*180/pi;
            obj.drivingMode = drivingMode;
            
            obj.vehicle.setPosition(obj.map.transformPoseTo3DAnim(pose));
            
            obj.vehicle.dynamics.orientation = [0 1 0 pose(3)-1.5*pi];
            %%
            
            %This block shouldn't run if the vehicle has reached its destination or collided
            if obj.vehicle.status.collided || obj.vehicle.pathInfo.destinationReached
                
                position= obj.vehicle.dynamics.position; %Output 1: Position of the vehicle
                rotation= obj.vehicle.dynamics.orientation; %Output 2: Rotation angle of the vehicle
                referencePose = obj.referencePose';
                poseOut=pose';
                obj.adaptive_gain = 2/(20*obj.curvature+1);
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
                              
            end
            
            referencePose = obj.referencePose';
            poseOut=pose';
            obj.adaptive_gain = 2/(20*obj.curvature+1);%function of calculating adaptive control law G
            obj.vehicle.dynamics.maxSpeed = sqrt(0.7*10/obj.curvature);%function of calculating maximum allowed speed on a curved road
        end
        
        function [position, orientation] = takeRoute(obj,car,speed,refRoute)
            RotationVector = refRoute(3,:);
            if ~car.status.canLaneSwitch == 0 %lane switch
                if isempty(obj.trajPolynom)
                        obj.generateMinJerkTrajectory05(car);
                        obj.generateMinJerkTrajectory075(car);
                        obj.generateMinJerkTrajectory09(car);
                        obj.generateMinJerkTrajectory10(car);
                        obj.generateMinJerkTrajectory11(car);
                        obj.generateMinJerkTrajectory125(car);
                        obj.generateMinJerkTrajectory15(car);
                        obj.costFunction05(car);
                        obj.costFunction075(car);
                        obj.costFunction09(car);
                        obj.costFunction10(car);
                        obj.costFunction11(car);
                        obj.costFunction125(car);
                        obj.costFunction15(car);
                        obj.trajChoose(car);
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
        
        function generateMinJerkTrajectory05(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = car.decisionUnit.LaneSwitchTime*0.5;
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
            obj.trajPolynom05 = num2cell([a0 a1 a2 a3 a4 a5]);
            
            
        end
        function generateMinJerkTrajectory075(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = car.decisionUnit.LaneSwitchTime*0.75;
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
            obj.trajPolynom075 = num2cell([a0 a1 a2 a3 a4 a5]);
            
            
        end
        function generateMinJerkTrajectory09(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = car.decisionUnit.LaneSwitchTime*0.9;
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
            obj.trajPolynom09 = num2cell([a0 a1 a2 a3 a4 a5]);
            
            
        end
        function generateMinJerkTrajectory10(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = car.decisionUnit.LaneSwitchTime*1;
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
            obj.trajPolynom10 = num2cell([a0 a1 a2 a3 a4 a5]);
            
            
        end
        function generateMinJerkTrajectory11(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = car.decisionUnit.LaneSwitchTime*1.1;
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
            obj.trajPolynom11 = num2cell([a0 a1 a2 a3 a4 a5]);
            
            
        end
        function generateMinJerkTrajectory125(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = car.decisionUnit.LaneSwitchTime*1.25;
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
            obj.trajPolynom125 = num2cell([a0 a1 a2 a3 a4 a5]);
            
            
        end
        function generateMinJerkTrajectory15(obj,car)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];
            T = car.decisionUnit.LaneSwitchTime*1.5;
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
            obj.trajPolynom15 = num2cell([a0 a1 a2 a3 a4 a5]);
            
            
        end
        function costFunction05(obj,car)
            [~,~,~,a3,a4,a5]=deal(obj.trajPolynom05{:});
            T = car.decisionUnit.LaneSwitchTime*0.5;
            t=0:0.01:T;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/T;%Mean squared jerk
            ttc_min = 1.4*obj.vehicle.decisionUnit.LaneSwitchTime+0-T/2;%mimimum ttc
            obj.vehicle.dataLog.costFunction05=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
        function costFunction075(obj,car)
            [~,~,~,a3,a4,a5]=deal(obj.trajPolynom075{:});
            T = car.decisionUnit.LaneSwitchTime*0.75;
            t=0:0.01:T;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/T;
            ttc_min = 1.4*obj.vehicle.decisionUnit.LaneSwitchTime+0-T/2;
            obj.vehicle.dataLog.costFunction075=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
        function costFunction09(obj,car)
            [~,~,~,a3,a4,a5]=deal(obj.trajPolynom09{:});
            T = car.decisionUnit.LaneSwitchTime*0.9;
            t=0:0.01:T;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/T;
            ttc_min = 1.4*obj.vehicle.decisionUnit.LaneSwitchTime+0-T/2;
            obj.vehicle.dataLog.costFunction09=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
        function costFunction10(obj,car)
            [~,~,~,a3,a4,a5]=deal(obj.trajPolynom10{:});
            T = car.decisionUnit.LaneSwitchTime*1;
            t=0:0.01:T;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/T;
            ttc_min = 1.4*obj.vehicle.decisionUnit.LaneSwitchTime+0-T/2;
            obj.vehicle.dataLog.costFunction10=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
        function costFunction11(obj,car)
            [~,~,~,a3,a4,a5]=deal(obj.trajPolynom11{:});
            T = car.decisionUnit.LaneSwitchTime*1.1;
            t=0:0.01:T;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/T;
            ttc_min = 1.4*obj.vehicle.decisionUnit.LaneSwitchTime+0-T/2;
            obj.vehicle.dataLog.costFunction11=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
        function costFunction125(obj,car)
            [~,~,~,a3,a4,a5]=deal(obj.trajPolynom125{:});
            T = car.decisionUnit.LaneSwitchTime*1.25;
            t=0:0.01:T;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/T;
            ttc_min = 1.4*obj.vehicle.decisionUnit.LaneSwitchTime+0-T/2;
            obj.vehicle.dataLog.costFunction125=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
        function costFunction15(obj,car)
            [~,~,~,a3,a4,a5]=deal(obj.trajPolynom15{:});
            T = car.decisionUnit.LaneSwitchTime*1.5;
            t=0:0.01:T;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/T;
            ttc_min = 1.4*obj.vehicle.decisionUnit.LaneSwitchTime+0-T/2;
            obj.vehicle.dataLog.costFunction15=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
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
            
        function trajChoose(obj,car)
            %find the trajectory with minimum cost function value
            A = [car.dataLog.costFunction05 car.dataLog.costFunction075 car.dataLog.costFunction09 car.dataLog.costFunction10 car.dataLog.costFunction11 car.dataLog.costFunction125 car.dataLog.costFunction15];
            ind = find(A==min(min(A)));
            if(ind==1)
                obj.trajPolynom = obj.trajPolynom05;
                obj.laneSwitchTime = car.decisionUnit.LaneSwitchTime*0.5;
            end
            if(ind==2)
                obj.trajPolynom = obj.trajPolynom075;
                obj.laneSwitchTime = car.decisionUnit.LaneSwitchTime*0.75;
            end
            if(ind==3)
                obj.trajPolynom = obj.trajPolynom09;
                obj.laneSwitchTime = car.decisionUnit.LaneSwitchTime*0.9;
            end
            if(ind==4)
                obj.trajPolynom = obj.trajPolynom10;
                obj.laneSwitchTime = car.decisionUnit.LaneSwitchTime*1;
            end
            if(ind==5)
                obj.trajPolynom = obj.trajPolynom11;
                obj.laneSwitchTime = car.decisionUnit.LaneSwitchTime*1.1;
            end
            if(ind==6)
                obj.trajPolynom = obj.trajPolynom125;
                obj.laneSwitchTime = car.decisionUnit.LaneSwitchTime*1.25;
            end
            if(ind==7)
                obj.trajPolynom = obj.trajPolynom15;
                obj.laneSwitchTime = car.decisionUnit.LaneSwitchTime*1.5;
            end
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
            

            if car.pathInfo.routeEndDistance <1% consider to reach the endpoint when distance smaller than a threshold. Threshold defined by the user
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
            [targetPosition_C,~] = obj.Frenet2Cartesian(route,s,d,radian);
            obj.referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];
        end
        
        function generateLeftRotationWaypoints(obj,car)
            %this function generates waypoints according to reference
            %trajectory. Waypoints are used as input signal for the Stanley
            %controller.
            route = car.pathInfo.currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];%Start- and endpoint of the current route
            position_Cart = car.dynamics.position([1,3]).*[1 -1];%Position of the vehicle in Cartesian coordinate
            radian = abs(car.pathInfo.currentTrajectory(3,1));%radian of the curved road, is positive for left rotating curved road
            %% transfer Cartesian coordinate into Frenet coordinate
            [s,vehicle_d,orientation_C,routeLength] = obj.Cartesian2Frenet(route,position_Cart,radian); %Coordinate Conversion function
            car.pathInfo.s = s;% Arc length
            car.pathInfo.routeEndDistance = routeLength-s; %distance to the current route's endpoint
            %% apply lane-changing trajectory
            if(~isempty(obj.trajPolynom))% if lane-changing trajectory exists
                t=get_param('MOBATSim','SimulationTime')-obj.laneSwitchStartTime;
                [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom{:});% read polynomials
                if t<=obj.laneSwitchTime%lane-changing is not finished
                    obj.latOffset = a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5;% reference d
                else%lane-changing done
                    obj.latOffset = 0;%reset reference d
                    car.status.laneSwitchFinish = 1;%lane-changing done flag
                    if car.status.canLaneSwitch ==1%left lane-changing
                        car.pathInfo.laneId = car.pathInfo.laneId+1;
                    elseif car.status.canLaneSwitch ==2%right lane-changing
                        car.pathInfo.laneId = car.pathInfo.laneId-1;
                    end
                   car.status.canLaneSwitch = 0;
                    obj.trajPolynom={};%reset polynomial
                end
            end
            
            d=-(obj.laneWidth*(car.pathInfo.laneId-0.5)+obj.latOffset);%negative is only for left rotating vehicle
            obj.error = d-vehicle_d;%lateral offset error
            obj.int_error_d = obj.int_error_d+obj.error*0.01;%integral of lateral offset error
            [targetPosition_C,~] = obj.Frenet2Cartesian(route,s,obj.adaptive_gain*obj.error+d,radian);%Coordinate Conversion function,obj.adaptive_gain*obj.error is for adaptive control
            obj.referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];%Required format for the Stanley controller
        end
        
        function generateRightRotationWaypoints(obj,car)
            route = car.pathInfo.currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];
            position_Cart = car.dynamics.position([1,3]).*[1 -1];
            radian = -abs(car.pathInfo.currentTrajectory(3,1));
            [s,vehicle_d,orientation_C,routeLength] = obj.Cartesian2Frenet(route,position_Cart,radian);
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
            [targetPosition_C,~] = obj.Frenet2Cartesian(route,s,obj.adaptive_gain*obj.error+d,radian);
            obj.referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];
            
        end

      
        function [position_Cart,orientation_Cart] = Frenet2Cartesian(obj,route,s,d,radian)
            
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
            if radian == 0%straight road
                route_Vector = endPoint-startPoint;
                local_route_Vector_i = route_Vector/norm(route_Vector);
                orientation_Cart = atan2(local_route_Vector_i(2),local_route_Vector_i(1));
                sideVector = [cos(orientation_Cart+pi/2) sin(orientation_Cart+pi/2)];
                position_Cart = s*local_route_Vector_i+d*sideVector+startPoint;
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
                position_Cart = l*[cos(lAng) sin(lAng)]+center;
                orientation_Cart = lAng+sign(radian)*pi/2;
                orientation_Cart = mod(orientation_Cart,2*pi);
                orientation_Cart = orientation_Cart.*(0<=orientation_Cart & orientation_Cart <= pi) + (orientation_Cart - 2*pi).*(pi<orientation_Cart & orientation_Cart<2*2*pi);   % angle in (-pi,pi]
            end
        end
        
        function [s,d,orientation_Cart,routeLength] = Cartesian2Frenet(obj,route,position_C,radian)
            
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
            if radian == 0%straight road
                route_Vector = endPoint-startPoint;
                local_route_Vector_i = route_Vector/norm(route_Vector);
                orientation_Cart = atan2(local_route_Vector_i(2),local_route_Vector_i(1));
                posVector = position_C-startPoint;
                s = dot(posVector,local_route_Vector_i);
                sideVector = [cos(orientation_Cart+pi/2) sin(orientation_Cart+pi/2)];
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

                l = position_C-center;
                d = norm(l)-r;
                lAng = atan2(l(2),l(1));

                start_dot_l = dot(startPointVector,l);
                start_cross_l = sign(radian)*(startPointVector(1)*l(2)-startPointVector(2)*l(1));
                angle = atan2(start_cross_l,start_dot_l);
                if mod(angle,2*pi) > abs(radian)
                    start_cross_l = -(startPointVector(1)*l(2)-startPointVector(2)*l(1));
                    angle = -atan2(start_cross_l,start_dot_l);
                end
                s = angle*r;
                routeLength = abs(radian)*r;
                orientation_Cart = lAng+sign(radian)*pi/2;
                orientation_Cart = mod(orientation_Cart,2*pi);
                orientation_Cart = orientation_Cart.*(0<=orientation_Cart & orientation_Cart <= pi) + (orientation_Cart - 2*pi).*(pi<orientation_Cart & orientation_Cart<2*2*pi);   % angle in (-pi,pi]
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
        
        function [out, out2] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 3];
            out2 = [1 3];
            
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
        
        function [out,out2] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = 'double';
            out2 = 'double';

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end
        
        function [out, out2] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
        
        function [out, out2] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
        
        function sts = getSampleTimeImpl(obj)
            % Define sample time type and parameters
            sts = obj.createSampleTime("Type", "Inherited");
        end
    end
    
end
