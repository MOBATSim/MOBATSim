classdef V_WPGenerator_PurePursuit < WaypointGenerator
    % This blocks generates waypoints.
    %
    
    % Pre-computed constants
    properties(Access = private)      

        %  laneSwitchWayPoints = [];% Trajectory for pure pursuit controller
        

        
        
        referenceWaypoints = zeros(10,3);
        adaptiveGain = 1;%adaptive control law G for the Stanley controller
        adaptiveGain_k0 = 2;%k0 of adaptive control law G,FKFS equation 10
        adaptiveGain_g0 = 20;%g0 of adaptive control law G,FKFS equation 10
        latOffsetError = 0;
        
    end
    
    methods
        % Constructor
        function obj = V_WPGenerator_PurePursuit(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@WaypointGenerator(obj);  % Inherit the setupImpl function of the Superclass @WaypointGenerator
        end
        
        
        function [position, referenceWaypoints] = stepImpl(obj,pose,speed)
            %transfer from local coordinate obj.vehicle.dynamics.speed = v_pos(4);
            %             pose(3)=pose(3)*180/pi; % rad to deg
            
            obj.vehicle.setPosition(obj.map.transformPoseTo3DAnim(pose));   % Sets the vehicle position
            obj.vehicle.setYawAngle(pose(3));                               % Sets the vehicle yaw angle (4th column of orientation)
            
            %This block shouldn't run if the ego vehicle: (destinationReached or Collided)
            if obj.vehicle.status.collided || obj.vehicle.pathInfo.destinationReached
                
                position=pose';                      % Output1: vehicle's actual pose
                referenceWaypoints = obj.referenceWaypoints(:,[1 3]);
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
                end
                
                speedAccordingtoSimulation = speed*0.01*obj.simSpeed;%speed limit on curved road
                %Gámez Serna, C., & Ruichek, Y. (2017). Dynamic Speed Adaptation for Path Tracking Based on Curvature Information and Speed Limits.
                %Sensors (Basel, Switzerland), 17(6), 1383. https://doi.org/10.3390/s17061383
                %equation 15
                
                %0.01 is the sample time -> obj.getSampleTime.SampleTime creates a huge overhead
                
                [~, ~] = obj.takeRoute(obj.vehicle,speedAccordingtoSimulation,obj.vehicle.pathInfo.currentTrajectory);
                %Output 1: Position of the vehicle
                %Output 2: Rotation angle of the vehicle
                
            end
            
            referenceWaypoints = obj.referenceWaypoints(:,[1 3]);
            position=pose';
            obj.adaptiveGain = obj.adaptiveGain_k0/(obj.adaptiveGain_g0*obj.curvature+1);%adaptive control law G
            
            % ISSUE 1: runs on straight road as well and becomes inf
            % ISSUE 2: sets the vehicle's maximum speed
            obj.vehicle.dynamics.maxSpeed = sqrt(0.7*10/obj.curvature);%function of calculating maximum allowed speed on a curved road
            
        end
        
        
        function [position, orientation] = takeRoute(obj,car,speed,refRoute)
            RotationVector = refRoute(3,:);
            
            obj.checkLaneSwitch(car);
            
            if (RotationVector(1) == 0) %Straight motion
                [position, orientation] = obj.move_straight(car,speed,refRoute(2,:));
                
            else %Rotational motion
                rotation_angle = RotationVector(1);
                rotation_point = [RotationVector(2) 0 RotationVector(3)];
                P_final = refRoute(2,:);
                %Determine rotation direction: left or right
                if car.pathInfo.currentTrajectory(4,:) == -ones(1,3) % -1 means turn left
                    [position, orientation] = obj.rotate_left(car,speed, rotation_point,P_final);
                elseif car.pathInfo.currentTrajectory(4,:) == ones(1,3) % 1 means turn right
                    [position, orientation] = obj.rotate_right(car,speed, rotation_point,P_final);
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
            cand_trajPolynom = num2cell([a0 a1 a2 a3 a4 a5]);
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
        
        function [position, orientation] = rotate_left(obj ,car, speed, rotation_point,Destination)
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
                car.pathInfo.s = 0;%reset s at the end of road
                
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
        
        function [position, orientation] = rotate_right(obj ,car,speed, rotation_point,Destination)
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
            if car.pathInfo.routeEndDistance <1% consider to reach the endpoint when distance smaller than a threshold. Threshold defined by the user
                car.pathInfo.s = 0;%reset s at the end of road
                
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
            %this function generates waypoints according to reference
            %trajectory. Waypoints are used as input signal for the Stanley
            %controller.
            route = car.pathInfo.currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];%Start- and endpoint of the current route
            position_Cart = car.dynamics.position([1,3]).*[1 -1];%Position of the vehicle in Cartesian coordinate
            radian = car.pathInfo.currentTrajectory(3,1);%radian of the curved road, is 0 for straight road
            [s,vehicle_d,orientation_C,routeLength] = obj.Cartesian2Frenet(route,position_Cart,radian);%Coordinate Conversion function
            car.pathInfo.s = s;% drived length
            car.pathInfo.routeEndDistance = routeLength-s; %distance to the current route's endpoint
            %             if(~isempty(obj.trajPolynom))% if lane-changing trajectory exists
            %                 t=get_param('MOBATSim','SimulationTime')-obj.laneSwitchStartTime;
            %                 [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom{:});% read polynomials
            %                 if t<=obj.laneSwitchTime%lane-changing is not finished
            %                     obj.latOffset = a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5;% reference delta_d
            %                 else%lane-changing done
            %                     obj.latOffset = 0;%reset reference delta_d
            %                     car.status.laneSwitchFinish = 1;%lane-changing done flag
            %                     if car.status.canLaneSwitch ==1%left lane-changing
            %                         car.pathInfo.laneId = car.pathInfo.laneId+1;
            %                     elseif car.status.canLaneSwitch ==2%right lane-changing
            %                         car.pathInfo.laneId = car.pathInfo.laneId-1;
            %                     end
            %                     car.status.canLaneSwitch = 0;%reset flag
            %                     car.dataLog.laneSwitchEndTime=[car.dataLog.laneSwitchEndTime get_param('MOBATSim','SimulationTime')];%logging endtime
            %                     obj.trajPolynom={};%reset polynomial
            %
            %                 end
            %             end
            
            % ISSUE: Doesn't have meaning with LaneId-0.5
            %d=obj.laneWidth*(car.pathInfo.laneId-0.5)+obj.latOffset;
            d = obj.latOffset;
            obj.latOffsetError = d-vehicle_d;%lateral offset error
            
            WP_s = s:1:s+length(obj.referenceWaypoints)-1;
            for i = 1:1:length(obj.referenceWaypoints)
                [targetPosition_C,~] = obj.Frenet2Cartesian(route,WP_s(i),obj.adaptiveGain*obj.latOffsetError+d,radian);
                obj.referenceWaypoints(i,:) = [targetPosition_C(1) 0 targetPosition_C(2)];
            end
%             [targetPosition_C,~] = obj.Frenet2Cartesian(route,s,obj.adaptiveGain*obj.latOffsetError+d,radian);%Coordinate Conversion function,obj.adaptiveGain*obj.latOffsetError is for adaptive control
            %             obj.referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];%Required format for the Stanley controller
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
%             if(~isempty(obj.trajPolynom))% if lane-changing trajectory exists
%                 t=get_param('MOBATSim','SimulationTime')-obj.laneSwitchStartTime;
%                 [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom{:});% read polynomials
%                 if t<=obj.laneSwitchTime%lane-changing is not finished
%                     obj.latOffset = a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5;% reference delta_d
%                 else%lane-changing done
%                     obj.latOffset = 0;%reset reference delta_d
%                     car.status.laneSwitchFinish = 1;%lane-changing done flag
%                     if car.status.canLaneSwitch ==1%left lane-changing
%                         car.pathInfo.laneId = car.pathInfo.laneId+1;
%                     elseif car.status.canLaneSwitch ==2%right lane-changing
%                         car.pathInfo.laneId = car.pathInfo.laneId-1;
%                     end
%                     car.status.canLaneSwitch = 0;%reset flag
%                     obj.trajPolynom={};%reset polynomial
%                 end
%             end
            % ISSUE: Doesn't have meaning with LaneId-0.5 
            d=-obj.latOffset;%for left rotating vehicle

            obj.latOffsetError = d-vehicle_d;%lateral offset error
            WP_s = s:1:s+length(obj.referenceWaypoints)-1;
            for i = 1:1:length(obj.referenceWaypoints)
                [targetPosition_C,~] = obj.Frenet2Cartesian(route,WP_s(i),obj.adaptiveGain*obj.latOffsetError+d,radian);
                obj.referenceWaypoints(i,:) = [targetPosition_C(1) 0 targetPosition_C(2)];
            end
%             [targetPosition_C,~] = obj.Frenet2Cartesian(route,s,obj.adaptiveGain*obj.latOffsetError+d,radian);%Coordinate Conversion function,obj.adaptiveGain*obj.latOffsetError is for adaptive control
%             obj.referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];%Required format for the Stanley controller
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
%             if(~isempty(obj.trajPolynom))% if lane-changing trajectory exists
%                 t=get_param('MOBATSim','SimulationTime')-obj.laneSwitchStartTime;
%                 [a0,a1,a2,a3,a4,a5]=deal(obj.trajPolynom{:});% read polynomials
%                 if t<=obj.laneSwitchTime%lane-changing is not finished
%                     obj.latOffset = a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5;% reference delta_d
%                 else%lane-changing done
%                     obj.latOffset = 0;%reset reference delta_d
%                     car.status.laneSwitchFinish = 1;%lane-changing done flag
%                     if car.status.canLaneSwitch ==1%left lane-changing
%                         car.pathInfo.laneId = car.pathInfo.laneId+1;
%                     elseif car.status.canLaneSwitch ==2%right lane-changing
%                         car.pathInfo.laneId = car.pathInfo.laneId-1;
%                     end
%                     car.status.canLaneSwitch = 0;%reset flag
%                     obj.trajPolynom={};%reset polynomial
%                 end
%             end
            % ISSUE: Doesn't have meaning with LaneId-0.5 
            d=obj.latOffset;%for right rotating vehicle

            obj.latOffsetError = d-vehicle_d;%lateral offset error
            WP_s = s:1:s+length(obj.referenceWaypoints)-1;
            for i = 1:1:length(obj.referenceWaypoints)
                [targetPosition_C,~] = obj.Frenet2Cartesian(route,WP_s(i),obj.adaptiveGain*obj.latOffsetError+d,radian);
                obj.referenceWaypoints(i,:) = [targetPosition_C(1) 0 targetPosition_C(2)];
            end
%             [targetPosition_C,~] = obj.Frenet2Cartesian(route,s,obj.adaptiveGain*obj.latOffsetError+d,radian);%Coordinate Conversion function,obj.adaptiveGain*obj.latOffsetError is for adaptive control
%             obj.referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];%Required format for the Stanley controller
            
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
            out2 = [10 2];
            
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
