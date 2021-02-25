classdef V_WPGenerator_Stanley < WaypointGenerator
    % This blocks generates waypoints.
    %
    
    % Pre-computed constants
    properties(Access = private)
        

        adaptiveGain = 1;%adaptive control law G for the Stanley controller
        adaptiveGain_k0 = 2;%k0 of adaptive control law G,FKFS equation 10
        adaptiveGain_g0 = 20;%g0 of adaptive control law G,FKFS equation 10
        latOffsetError = 0;

    end
    
    methods
        % Constructor
        function obj = V_WPGenerator_Stanley(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@WaypointGenerator(obj);  % Inherit the setupImpl function of the Superclass @WaypointGenerator
        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon("WaypointGenerator.png");
        end


        
        
        function [poseOut, referencePose] = stepImpl(obj,pose,speed)
            %transfer from local coordinate obj.vehicle.dynamics.speed = v_pos(4);
            pose(3)=pose(3)*180/pi; % rad to deg
            
            obj.vehicle.setPosition(obj.map.transformPoseTo3DAnim(pose));   % Sets the vehicle position
            obj.vehicle.setYawAngle(pose(3)-1.5*pi);                        % Sets the vehicle yaw angle (4th column of orientation)
            
            %This block shouldn't run if the ego vehicle: (destinationReached or Collided)
            if obj.vehicle.status.collided || obj.vehicle.pathInfo.destinationReached
                
                poseOut=pose';                      % Output1: vehicle's actual pose               
                referencePose = obj.referencePose'; % Output2: Reference pose

                obj.adaptiveGain = obj.adaptiveGain_k0/(obj.adaptiveGain_g0*obj.curvature+1);%adaptive control law G
                return;
            end
            
            
            if ~obj.vehicle.pathInfo.destinationReached
                % The Vehicle hasn't reached its destination yet
                obj.vehicle.updateActualSpeed(speed); % Vehicle - Set Functions

                if obj.vehicle.pathInfo.routeCompleted
                    % The Vehicle has completed its Route
                    nextRoute = obj.generateCurrentRoute(obj.vehicle,obj.vehicle.pathInfo.path,obj.vehicle.pathInfo.lastWaypoint);
                    currentTrajectory = obj.generateTrajectoryFromPath(obj.vehicle,obj.vehicle.pathInfo.path);
                    
                    obj.vehicle.setCurrentRoute(nextRoute);              % Vehicle - Set Functions
                    obj.vehicle.setCurrentTrajectory(currentTrajectory); % Vehicle - Set Functions
                    obj.vehicle.setRouteCompleted(false);                % Vehicle - Set Functions
                    
                end
                
                speedAccordingtoSimulation = speed*0.01*obj.simSpeed;%speed limit on curved road
                %Gámez Serna, C., & Ruichek, Y. (2017). Dynamic Speed Adaptation for Path Tracking Based on Curvature Information and Speed Limits. 
                %Sensors (Basel, Switzerland), 17(6), 1383. https://doi.org/10.3390/s17061383
                %equation 15
                
                %0.01 is the sample time -> obj.getSampleTime.SampleTime creates a huge overhead
                
                obj.takeRoute(obj.vehicle,speedAccordingtoSimulation,obj.vehicle.pathInfo.currentTrajectory);
                %Output 1: Position of the vehicle
                %Output 2: Rotation angle of the vehicle
                              
            end
            
            referencePose = obj.referencePose';
            poseOut=pose';
            obj.adaptiveGain = obj.adaptiveGain_k0/(obj.adaptiveGain_g0*obj.curvature+1);%adaptive control law G
            
            % ISSUE 1: runs on straight road as well and becomes inf
            % ISSUE 2: sets the vehicle's maximum speed
            %obj.vehicle.dynamics.maxSpeed = sqrt(0.7*10/obj.curvature);%function of calculating maximum allowed speed on a curved road
            
        end
        

        function takeRoute(obj,car,speed,refRoute)
            RotationVector = refRoute(3,:);
            
            obj.checkLaneSwitch(car);

            if (RotationVector(1) == 0) %Straight motion
                obj.move_straight(car,speed,refRoute(2,:));
                
            else %Rotational motion

                P_final = refRoute(2,:);
                
                %Determine rotation direction: left or right
                if car.pathInfo.currentTrajectory(4,:) == -ones(1,3) % -1 means turn left
                    obj.rotate_left(car,speed,P_final);
                    
                elseif car.pathInfo.currentTrajectory(4,:) == ones(1,3) % 1 means turn right
                    obj.rotate_right(car,speed,P_final);
                end
            end
            
        end
        
        
        function checkLaneSwitch(obj,car)
            if ~(car.status.canLaneSwitch==0) %lane switch
                if isempty(obj.trajPolynom)
                    
                    candidateTrajectories = []; % Candidate trajectories stored here
                    costsTrajectories = []; % 

                    for deltaTFactor = 0.5:0.25:1.5
                        [cand_trajPolynom, costTraj] = obj.generateMinJerkTrajectory(car,deltaTFactor);
                        candidateTrajectories = [candidateTrajectories; cand_trajPolynom];
                        costsTrajectories = [costsTrajectories costTraj];
                    end

                    obj.chooseReferenceTrajectory(car,candidateTrajectories,costsTrajectories);
                end
            end
        end
        
        
        function [cand_trajPolynom, costTraj]=generateMinJerkTrajectory(obj,car,deltaTfactor)
            obj.laneSwitchStartTime = get_param('MOBATSim','SimulationTime');
            car.dataLog.laneSwitchStartTime = [car.dataLog.laneSwitchStartTime obj.laneSwitchStartTime];%logging lane-switch start time
            obj.laneSwitchStartPoint = car.dynamics.position.*[1 1 -1];%coordinates conversion
            T = car.decisionUnit.LaneSwitchTime*deltaTfactor;%delta_T
            %% minimum jerk trajectory
            x_f = T*car.dynamics.speed; % Final x coordinate
            if car.status.canLaneSwitch ==1
                y_f = obj.laneWidth; % Final y coordinate
            elseif car.status.canLaneSwitch ==2
                y_f = -obj.laneWidth;
            end
            obj.laneSwitchTargetPoint=[x_f 0 y_f]+obj.laneSwitchStartPoint;
            %%  Minimun jerk trajectory function for the calculation in y direction (Lateral)
            syms t; % time
            % matrix with polynom coefficients for  lateral position, speed and acceleration
            %         a0   a1    a2     a3      a4       a5
            d(t) = [  1     t   t^2    t^3     t^4     t^5;
                      0     1   2*t  3*t^2   4*t^3   5*t^4;
                      0     0     2    6*t  12*t^2  20*t^3];
            % Starting conditions
            ti = 0; % time
            d_ti = [0; 0; 0]; % position, speed, acceleration
            % Finish conditions
            tf = T;
            d_tf = [y_f; 0; 0]; % position, speed, acceleration        
            % Solve all linear equations with conditions at t = ti and t = tf
            A = linsolve([d(ti);d(tf)], [d_ti; d_tf]);
            A = double(A);
            a0=A(1);
            a1=A(2);
            a2=A(3);
            a3=A(4);
            a4=A(5);
            a5=A(6);
            cand_trajPolynom = [a0 a1 a2 a3 a4 a5];
            cand_traj_coeffs = [a3, a4 , a5];
            costTraj=obj.calculateCostFunction(car,cand_traj_coeffs);
            
            obj.trajPolynom_candidates = [obj.trajPolynom_candidates; cand_trajPolynom];
        end

        function costTraj = calculateCostFunction(obj,car,candidateTrajectory)
            a3 = candidateTrajectory(1);
            a4 = candidateTrajectory(2);
            a5 = candidateTrajectory(3);

            T = car.decisionUnit.LaneSwitchTime*0.5;
            t=0:0.01:T;
            y_dddot2 = (6*a3+24*a4*t+60*a5*t.^2).^2;
            mean_y_dddot2=sum(y_dddot2*0.01)/T;%Mean squared jerk
            ttc_min = 1.4*obj.vehicle.decisionUnit.LaneSwitchTime+0-T/2;%minimum ttc
            costTraj=obj.comfortGain*mean_y_dddot2+obj.safetyGain/ttc_min;
        end
              
        function chooseReferenceTrajectory(obj,car, candidateTrajectories, costsTrajectories)
            %find the trajectory with minimum cost function value
            
            ind = costsTrajectories==min(min(costsTrajectories)); % TODO: precaution should be taken when two costs are same
            % ind might not be a single number in that case, it should be validated to be only a single number not an array.
            
            obj.trajPolynom = candidateTrajectories(ind,:);
            obj.laneSwitchTime = car.decisionUnit.LaneSwitchTime*0.5;
            
        end
        

        function generateStraightWaypoints(obj,car)
            %this function generates waypoints according to reference
            %trajectory. Waypoints are used as input signal for the Stanley
            %controller.
            route = car.pathInfo.currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];%Start- and endpoint of the current route
            position_Cart = car.dynamics.position([1,3]).*[1 -1];%Position of the vehicle in Cartesian coordinate
            radian = car.pathInfo.currentTrajectory(3,1);%radian of the curved road, is 0 for straight road
            [s,vehicle_d,orientation_C,routeLength] = obj.Cartesian2Frenet(route,position_Cart,radian);%Coordinate Conversion function
            car.pathInfo.s = s;% drived length
            car.pathInfo.routeEndDistance = routeLength-s; %distance to the current route's endpoint
            
            %% If lane-changing trajectory exists
            obj.generateLaneChanging_WPs(car)

            
            % ISSUE: Doesn't have meaning with LaneId-0.5 
            d=obj.laneWidth*(car.pathInfo.laneId-0.5)+obj.latOffset;
            obj.latOffsetError = d-vehicle_d;%lateral offset error
            
            [targetPosition_C,~] = obj.Frenet2Cartesian(route,s,obj.adaptiveGain*obj.latOffsetError+d,radian);%Coordinate Conversion function,obj.adaptiveGain*obj.latOffsetError is for adaptive control
            obj.referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];%Required format for the Stanley controller
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
            %% apply lane-changing trajectory if a Trajectory polynomial has been set
            obj.generateLaneChanging_WPs(car);

            
            d=-(obj.laneWidth*(car.pathInfo.laneId-0.5)+obj.latOffset);%negative is only for left rotating vehicle
            obj.latOffsetError = d-vehicle_d;%lateral offset error
            [targetPosition_C,~] = obj.Frenet2Cartesian(route,s,obj.adaptiveGain*obj.latOffsetError+d,radian);%Coordinate Conversion function,obj.adaptiveGain*obj.latOffsetError is for adaptive control
            obj.referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];%Required format for the Stanley controller
        end
        
        function generateRightRotationWaypoints(obj,car)
            %this function generates waypoints according to reference
            %trajectory. Waypoints are used as input signal for the Stanley
            %controller.
            route = car.pathInfo.currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];%Start- and endpoint of the current route
            position_Cart = car.dynamics.position([1,3]).*[1 -1];%Position of the vehicle in Cartesian coordinate
            radian = -abs(car.pathInfo.currentTrajectory(3,1));%radian of the curved road, is negative for left rotating curved road
            [s,vehicle_d,orientation_C,routeLength] = obj.Cartesian2Frenet(route,position_Cart,radian);%Coordinate Conversion function
            car.pathInfo.s = s;% Arc length
            car.pathInfo.routeEndDistance = routeLength-s;%distance to the current route's endpoint
            
            obj.generateLaneChanging_WPs(car);

            
            d=obj.laneWidth*(car.pathInfo.laneId-0.5)+obj.latOffset;%for left rotating vehicle
            obj.latOffsetError = d-vehicle_d;%lateral offset error
            [targetPosition_C,~] = obj.Frenet2Cartesian(route,s,obj.adaptiveGain*obj.latOffsetError+d,radian);%Coordinate Conversion function,obj.adaptiveGain*obj.latOffsetError is for adaptive control
            obj.referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];%Required format for the Stanley controller
            
        end

        function generateLaneChanging_WPs(obj, car)
            if(~isempty(obj.trajPolynom))% if lane-changing trajectory exists
                
                t=get_param('MOBATSim','SimulationTime')-obj.laneSwitchStartTime;
                
                % Unfortunately there is no better way than this, double2cell -> deal is slower
                a0=obj.trajPolynom(1);% a0
                a1=obj.trajPolynom(2);% a1
                a2=obj.trajPolynom(3);% a2
                a3=obj.trajPolynom(4);% a3
                a4=obj.trajPolynom(5);% a4
                a5=obj.trajPolynom(6);% a5
                
                if t<=obj.laneSwitchTime%lane-changing is not finished
                    obj.latOffset = a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5;% reference delta_d
                else%lane-changing done
                    obj.latOffset = 0;%reset reference delta_d
                    car.status.laneSwitchFinish = 1;%lane-changing done flag
                    if car.status.canLaneSwitch ==1%left lane-changing
                        car.pathInfo.laneId = car.pathInfo.laneId+1;
                    elseif car.status.canLaneSwitch ==2%right lane-changing
                        car.pathInfo.laneId = car.pathInfo.laneId-1;
                    end
                    car.status.canLaneSwitch = 0;%reset flag
                    obj.trajPolynom=[];%reset polynomial
                end
            end
        end
        
    end
    %% Standard Simulink Output functions
    methods(Static,Access = protected)

        
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
