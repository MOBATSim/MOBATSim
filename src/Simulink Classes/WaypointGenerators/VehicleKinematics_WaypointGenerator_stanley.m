classdef VehicleKinematics_WaypointGenerator_stanley < WaypointGenerator
    % This blocks generates waypoints.
    %
    
    % Pre-computed constants
    properties(Access = private)      
        laneWidth = 3.7; % Standard road width
        curvature = 0;%curvature of the current road
     %  laneSwitchWayPoints = [];% Trajectory for pure pursuit controller
        laneSwitchStartPoint = [];
        laneSwitchTargetPoint = [];
        laneSwitchStartTime = [];
        
        safetyGain = 80;%k_2 in cost function(FKFS)
        comfortGain = 0.05;%k_1 in cost function(FKFS)
        laneSwitchTime = 3;%delta_T, choosen by cost function
        latOffset = 0;%variable to save reference delta_d in Frenet coordinate
        trajPolynom_candidates = [];% condidate trajectories
        trajPolynom = [];% Trajectory choosen
        %velocityPolynom = {};% reference velocity for minimum jerk trajectory
        %accPolynom = {};%reference acc for minimum jerk trajectory
        %jerkPolynom = {};%reference jerk for minimum jerk trajectory
        
        referencePose = [0; 0; 0];
        adaptiveGain = 1;%adaptive control law G for the Stanley controller
        adaptiveGain_k0 = 2;%k0 of adaptive control law G,FKFS equation 10
        adaptiveGain_g0 = 20;%g0 of adaptive control law G,FKFS equation 10
        latOffsetError = 0;

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
            setupImpl@WaypointGenerator(obj);  % Inherit the setupImpl function of the Superclass @WaypointGenerator
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
        
        function move_straight(obj,car,speed,Destination)
            %% Reference Waypoint Generation
            obj.generateStraightWaypoints(car)
            %%
            
            if car.pathInfo.routeEndDistance <1
                
                car.pathInfo.s = 0;
                
                lastWaypoint = car.map.get_waypoint_from_coordinates(Destination);
                
                car.setRouteCompleted(true); % Vehicle Set
                car.setLastWaypoint(lastWaypoint); % Vehicle Set
                
                nextRoute = obj.generateCurrentRoute(car,car.pathInfo.path,lastWaypoint);
                car.setCurrentRoute(nextRoute); % Vehicle Set
            end

        end
        
        function rotate_left(obj ,car, speed,Destination)
            %% Reference Waypoint Generation
            obj.generateLeftRotationWaypoints(car);
            %%

            if car.pathInfo.routeEndDistance <1% consider to reach the endpoint when distance smaller than a threshold. Threshold defined by the user
                car.pathInfo.s = 0;%reset s at the end of road
                
                lastWaypoint = car.map.get_waypoint_from_coordinates(Destination);
                
                car.setRouteCompleted(true);% Vehicle Set
                car.setLastWaypoint(lastWaypoint); % Vehicle Set
                
                nextRoute = obj.generateCurrentRoute(car,car.pathInfo.path,lastWaypoint);
                car.setCurrentRoute(nextRoute); % Vehicle Set
                
                
            end
            
        end
        
        function rotate_right(obj ,car,speed,Destination)
            %% Reference Waypoint Generation
            obj.generateRightRotationWaypoints(car);
            %%
            
            if car.pathInfo.routeEndDistance <1% consider to reach the endpoint when distance smaller than a threshold. Threshold defined by the user
                car.pathInfo.s = 0;%reset s at the end of road
                
                lastWaypoint = car.map.get_waypoint_from_coordinates(Destination);
                
                car.setRouteCompleted(true);% Vehicle Set
                car.setLastWaypoint(lastWaypoint); % Vehicle Set
                
                nextRoute = obj.generateCurrentRoute(car,car.pathInfo.path,lastWaypoint);
                car.setCurrentRoute(nextRoute);
                

            end
            
            
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
            %detail information check Frenet.mlx
            startPoint = route(1,:);
            endPoint = route(2,:);
            if radian == 0%straight road
                route_Vector = endPoint-startPoint;
                local_route_Vector_i = route_Vector/norm(route_Vector);
                orientation_Cart = atan2(local_route_Vector_i(2),local_route_Vector_i(1));
                sideVector = [cos(orientation_Cart+pi/2) sin(orientation_Cart+pi/2)];
                position_Cart = s*local_route_Vector_i+d*sideVector+startPoint;
            else
                r = sqrt((norm(endPoint-startPoint))^2/(1-cos(radian))/2);%The radius of the road segment
                targetVector = (endPoint-startPoint)/norm(endPoint-startPoint);
                beta = atan2(targetVector(2),targetVector(1));
                plumbLength = cos(radian/2)*r;
                plumbVector = [cos(beta+sign(radian)*pi/2) sin(beta+sign(radian)*pi/2)]*plumbLength;
                center = startPoint + targetVector*norm(endPoint-startPoint)/2 + plumbVector;%rotation center of the road
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
            %this function follows the similar logic with function Frenet2Cartesian(obj,route,s,d,radian)
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
