classdef VehicleKinematics_WaypointGenerator < VehicleKinematics
    % This blocks generates waypoints.
    %
    
    % Pre-computed constants
    properties(Access = private)
        vehicle
        map = evalin('base','Map');
        simSpeed = evalin('base','simSpeed');
        modelName = evalin('base','modelName');
        
        referenceWaypoints = zeros(10,3);
    end
    
    methods
        % Constructor
        function obj = VehicleKinematics_WaypointGenerator(varargin)
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
        
        
        function [pose, referenceWaypoints] = stepImpl(obj,pose,speed,drivingMode)
            %transfer from local coordinate obj.vehicle.dynamics.speed = v_pos(4);
            obj.vehicle.dynamics.position = [pose(1) 0 -pose(2)];
            obj.vehicle.dynamics.orientation = [0 1 0 pose(3)-1.5*pi];
            %referenceWaypoints = obj.referenceWaypoints;
            %%
            
            %This block shouldn't run if the vehicle has reached its destination or collided
            if obj.vehicle.status.collided || obj.vehicle.pathInfo.destinationReached
                
                position= obj.vehicle.dynamics.position; %Output 1: Position of the vehicle
                rotation= obj.vehicle.dynamics.orientation; %Output 2: Rotation angle of the vehicle
                referenceWaypoints = obj.referenceWaypoints(:,[1 3]);
                return;
                
            elseif ~obj.vehicle.pathInfo.destinationReached
                % The Vehicle hasn't reached its destination yet
                obj.vehicle.updateActualSpeed(speed); % Vehicle - Set Functions
                
                if obj.vehicle.pathInfo.routeCompleted
                    % The Vehicle has completed its Route
                    nextRoute = obj.generateCurrentRoute(obj.vehicle,obj.vehicle.pathInfo.path,obj.vehicle.pathInfo.lastWaypoint);
                    currentTrajectory = obj.generateTrajectoryFromPath(obj.vehicle,obj.vehicle.pathInfo.path);
                    
                    obj.vehicle.setCurrentRoute(nextRoute);              % Vehicle - Set Functions
                    obj.vehicle.setCurrentTrajectory(currentTrajectory); % Vehicle - Set Functions
                end
                
                speedAccordingtoSimulation = speed*0.01*obj.simSpeed;
                %0.01 is the sample time -> obj.getSampleTime.SampleTime creates a huge overhead
                
                [position, rotation] = obj.takeRoute(obj.vehicle,speedAccordingtoSimulation,obj.vehicle.pathInfo.currentTrajectory);
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
            referenceWaypoints = obj.referenceWaypoints(:,[1 3]);
            
        end
        
        function [position, orientation] = takeRoute(obj,car,speed,refRoute)
            RotationVector = refRoute(3,:);
            
            if RotationVector(1) == 0 %Straight motion
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
        
        function [position, orientation] = move_straight(obj ,car,speed,Destination)
            %% Reference Waypoint Generation
            obj.generateStraightWaypoints(car.dynamics.position, speed, car.pathInfo.currentTrajectory)
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
            
            
            
            %if  norm(car.dynamics.directionVector/norm(car.dynamics.directionVector))*speed > norm(Destination-car.dynamics.position)
            if norm(car.dynamics.position-Destination)< 2 % Error tolerance value TODO: check lower numbers
%                 car.setPosition(Destination); % Vehicle Set
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
            obj.generateLeftRotationWaypoints(car.dynamics.position, speed, car.pathInfo.currentTrajectory, Destination)
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
            
            if  norm(car.dynamics.position - Destination) < 4 % TODO Check the value
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
            obj.generateRightRotationWaypoints(car.dynamics.position, speed, car.pathInfo.currentTrajectory, Destination)
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
            
            if  norm(car.dynamics.position - Destination) < 4 % TODO Check the value
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
        
        function generateStraightWaypoints(obj,vehiclePosition, speed, currentTrajectory)
            
            route_Vector = currentTrajectory(2,:)-currentTrajectory(1,:);
            local_route_Vector_i = route_Vector/norm(route_Vector).*[1 1 -1];
            
            pos_Vector = vehiclePosition-currentTrajectory(1,:);
            local_pos_Vector = pos_Vector.*[1 1 -1];
            
            setoff_distance = dot(local_pos_Vector,local_route_Vector_i);
            local_WP_start_point = (setoff_distance*local_route_Vector_i+currentTrajectory(1,:).*[1 1 -1]);
            
            horizonSteps = 100;
            
            for i = 1:1:length(obj.referenceWaypoints)
                obj.referenceWaypoints(i,:) = local_WP_start_point+norm(i*(speed*horizonSteps))*local_route_Vector_i;
            end
        end
        
        function generateLeftRotationWaypoints(obj,vehiclePosition, speed, currentTrajectory, Destination)
            local_position = vehiclePosition.*[1 1 -1];
            
            step_length = speed;
            
            local_rotation_angle = currentTrajectory(3,1);
            
            %local_rotation_start_point = obj.vehicle.map.waypoints(obj.vehicle.pathInfo.lastWaypoint,:).*[1 1 -1];
            local_rotation_start_point = currentTrajectory(1,:).*[1 1 -1];
            
            r = sqrt((norm(Destination.*[1 1 -1]-local_rotation_start_point))^2/(1-cos(local_rotation_angle))/2);
            
            step_angle = step_length/r;
            
            local_displacement_vector = (Destination.*[1 1 -1]-local_rotation_start_point)/norm(Destination.*[1 1 -1]-local_rotation_start_point);
            local_r_angle = acos(dot(local_displacement_vector,[1 0 0]));
            
            if (local_displacement_vector(3)<0)
                local_r_angle=2*pi-local_r_angle;
            end
            
            local_plumb_length = cos(local_rotation_angle/2)*r;
            local_plumb_vector = [cos(local_r_angle+pi/2) 0 sin(local_r_angle+pi/2)]*local_plumb_length;
            local_rotation_center = local_rotation_start_point + local_displacement_vector*norm(Destination.*[1 1 -1]-local_rotation_start_point)/2 + local_plumb_vector;            
            l = local_position - local_rotation_center;
            local_start_angle = acos(dot(l,[1 0 0])/norm(l));
            if (l(3)<0)
                local_start_angle=2*pi-local_start_angle;
            end
            for i = 1:1:length(obj.referenceWaypoints)
                target_point_P = [r,local_start_angle+step_angle*(i-1)*30];
                target_point_C = local_rotation_center+[r*cos(target_point_P(2)) 0 r*sin(target_point_P(2))];
                obj.referenceWaypoints(i,:) = target_point_C;
            end
        end
        
        function generateRightRotationWaypoints(obj,vehiclePosition, speed, currentTrajectory, Destination)
            local_position = vehiclePosition.*[1 1 -1];
            
            step_length = speed;
            %10 is look ahead faktor
            local_rotation_angle = -currentTrajectory(3,1);
            %rotation angle of the whole curved road
            %local_rotation_start_point = obj.vehicle.map.waypoints(obj.vehicle.pathInfo.lastWaypoint,:).*[1 1 -1];
            local_rotation_start_point = currentTrajectory(1,:).*[1 1 -1];
            %       local_rotation_start_point = local_rotation_start_point*[1,1,-1];
            r = sqrt((norm(Destination.*[1 1 -1]-local_rotation_start_point))^2/(1-cos(local_rotation_angle))/2);
            step_angle = step_length/r;
            local_displacement_vector = (Destination.*[1 1 -1]-local_rotation_start_point)/norm(Destination.*[1 1 -1]-local_rotation_start_point);
            local_r_angle = acos(dot(local_displacement_vector,[1 0 0]));
            if (local_displacement_vector(3)<0)
                local_r_angle=2*pi-local_r_angle;
            end
            local_plumb_length = cos(local_rotation_angle/2)*r;
            local_plumb_vector = [cos(local_r_angle-pi/2) 0 sin(local_r_angle-pi/2)]*local_plumb_length;
            local_rotation_center = local_rotation_start_point + local_displacement_vector*norm(Destination.*[1 1 -1]-local_rotation_start_point)/2 + local_plumb_vector;
            l = local_position - local_rotation_center;
            %d = norm(cross(local_rotation_start_point-local_rotation_center,local_position-local_rotation_center))/norm(local_rotation_start_point-local_rotation_center);
            l_angle = -acos(dot(l,[1 0 0])/norm(l));
            if(l(3)>=0)
                l_angle = l_angle-2*(pi+l_angle);
            end
            for i = 1:1:length(obj.referenceWaypoints)
                target_point_P = [r,mod(l_angle-step_angle*(i-1)*30,-2*pi)];
                target_point_C = local_rotation_center+[r*cos(target_point_P(2)) 0 r*sin(target_point_P(2))];
                obj.referenceWaypoints(i,:) = target_point_C;
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
            out = [3 1];
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
