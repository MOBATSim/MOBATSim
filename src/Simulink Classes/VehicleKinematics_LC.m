classdef VehicleKinematics_LC < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime & matlab.system.mixin.CustomIcon
    % This blocks converts the speed of the vehicle to relevant position and rotation angles to generate the pose value.
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.
    
    % Public, tunable properties
    properties
        Vehicle_id
    end
    
    properties(DiscreteState)
        
    end
    
    % Pre-computed constants
    properties(Access = private)
        vehicle
        map = evalin('base','Map');
        simSpeed = evalin('base','simSpeed');
        modelName = evalin('base','modelName');
    end
    
    methods(Access = protected)
        
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.vehicle = evalin('base',strcat('Vehicle',int2str(obj.Vehicle_id)));
            
        end
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon("Vehicle.png");
        end
        
        
        %% Loop function
        function [position, rotation] = stepImpl(obj,speed,real_speed,X,Y,psi, drivingMode)
            %This block shouldn't run if the vehicle has reached its
            %destination
            v_pos=[X Y psi real_speed];
            if obj.vehicle.status.collided
                %Output 1: Position of the vehicle
                position= obj.vehicle.dynamics.position;
                %Output 2: Rotation angle of the vehicle
                rotation= obj.vehicle.dynamics.orientation;
                return;
            end
            
            if ~obj.vehicle.pathInfo.destinationReached || obj.vehicle.status.collided
                obj.vehicle.dynamics.speed = speed;
                if obj.vehicle.pathInfo.routeCompleted && obj.vehicle.status.stop ==0
                    obj.vehicle.pathInfo.currentRoute = obj.setCurrentRoute(obj.vehicle);
                    obj.vehicle.pathInfo.currentTrajectory = obj.generateTrajectory(obj.vehicle);
                elseif drivingMode == 5
                    
                end
                % TODO Check the kinematic equations
                speedAccordingtoSimulation = speed*0.01*obj.simSpeed;
%                 speedAccordingtoSimulation = speed;%test qihang
                %0.01 is the sample time but it needs to be automatically detected without a big overhead
                %obj.getSampleTime.SampleTime creates a huge overhead
                obj.nextMove(obj.vehicle,speedAccordingtoSimulation,v_pos);
            end
            
            %Output 1: Position of the vehicle
            position= obj.vehicle.dynamics.position;
            %Output 2: Rotation angle of the vehicle
            rotation= obj.vehicle.dynamics.orientation;
            
            
        end
        
        
        
        %% Helper functions
        function currentRoute = setCurrentRoute(~,car)
            %% TODO - check if it works in all situations
            idx = find(car.pathInfo.path==car.pathInfo.lastWaypoint);
            if idx+1<=length(car.pathInfo.path)
                currentRoute = car.map.getRouteIDfromPath([car.pathInfo.path(idx) car.pathInfo.path(idx+1)]);
            end
        end
        
        function currentTrajectory = generateTrajectory(~,car)
            % format of route for vehicle dynamics (translation)
            if (isempty(find((car.map.connections.translation(:,1) == car.pathInfo.path(1) )&(car.map.connections.translation(:,2)== car.pathInfo.path(2)), 1 )) == false)
                index = find((car.map.connections.translation(:,1) == car.pathInfo.path(1) )&(car.map.connections.translation(:,2)== car.pathInfo.path(2)) );
                currentTrajectory = [car.map.waypoints(car.map.connections.translation(index,1),:);
                    car.map.waypoints(car.map.connections.translation(index,2),:);
                    zeros(1,3);
                    zeros(1,3)];
            end
            % format of route for vehicle dynamics (curves)
            if (isempty(find((car.map.connections.circle(:,1) == car.pathInfo.path(1) )&(car.map.connections.circle(:,2)== car.pathInfo.path(2)), 1 )) == false)
                index = find((car.map.connections.circle(:,1) == car.pathInfo.path(1) )&(car.map.connections.circle(:,2)== car.pathInfo.path(2)) );
                currentTrajectory = [car.map.waypoints(car.map.connections.circle(index,1),:);
                    car.map.waypoints(car.map.connections.circle(index,2),:);
                    abs(car.map.connections.circle(index,3)),car.map.connections.circle(index,4),car.map.connections.circle(index,6);
                    -sign(car.map.connections.circle(index,3))*ones(1,3)];
            end
        end
        
        function moveto(obj ,car,speed,Destination,v_pos)
            %% Waypoint generation
            if ~car.dynamics.has_local_trajectory
            obj.generate_straight_move_WPs(car,speed,Destination,v_pos);
            end
            %%
            
            %Displacement Vector and determination of its Angle
            if car.pathInfo.routeCompleted == true
                DisplacementVector = Destination- car.dynamics.position;
                ThetaRadian = atan(DisplacementVector(1)/DisplacementVector(3));
                car.dynamics.orientation(4) = ThetaRadian;
                car.dynamics.directionVector = DisplacementVector;
                car.pathInfo.routeCompleted = false;
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
            
            
            car.dynamics.orientation = [ 0 1 0 ThetaRadian];
            
            % TODO Check if we need to add the sample time of the simulation into this equation below
%             car.dynamics.position = car.dynamics.position + (car.dynamics.directionVector/norm(car.dynamics.directionVector))*(speed);

            car.dynamics.position = car.dynamics.position + (car.dynamics.directionVector/norm(car.dynamics.directionVector))*v_pos(4)*0.01;%test qihang
            
%             if  norm(car.dynamics.directionVector/norm(car.dynamics.directionVector))*speed > norm(Destination-car.dynamics.position)
            if  norm(car.dynamics.directionVector/norm(car.dynamics.directionVector))*v_pos(4)*0.01 > norm(Destination-car.dynamics.position)
                car.dynamics.position = Destination; %Because of the rounding errors may be modified later
                car.pathInfo.routeCompleted=true;
                car.pathInfo.lastWaypoint = car.map.get_waypoint_from_coordinates (Destination);
                
                %% TODO - check if it works in all situations
                idx = find(car.pathInfo.path==car.pathInfo.lastWaypoint);
                if idx+1<=length(car.pathInfo.path)
                    car.pathInfo.currentRoute = car.map.getRouteIDfromPath([car.pathInfo.path(idx) car.pathInfo.path(idx+1)]);
                end
            end
            
            
        end
        
        function rotate_left(obj,car, speed, rotation_point,rotation_angle,Destination,v_pos)
            
            %% WP generation
            if ~car.dynamics.has_local_trajectory
            obj.generate_left_rotation_WPs(car, speed, rotation_point,rotation_angle,Destination,v_pos);
            end
            %%
            r = norm(Destination-rotation_point);
            
            if car.pathInfo.routeCompleted == true
                
                car.dynamics.cornering.angles = pi;
                car.pathInfo.routeCompleted = false;
                
                point_to_rotate= car.dynamics.position;
                
                car.dynamics.cornering.a=point_to_rotate(1)-rotation_point(1);
                car.dynamics.cornering.b=point_to_rotate(2)-rotation_point(2);
                car.dynamics.cornering.c=point_to_rotate(3)-rotation_point(3);
                
            end
            
            vector_z=[0 0 1];
            
            a = car.dynamics.cornering.a;
            b = car.dynamics.cornering.b;
            c = car.dynamics.cornering.c;
            
            step_length = speed/r;
            car.dynamics.cornering.angles = car.dynamics.cornering.angles - step_length;
            t = car.dynamics.cornering.angles;
            
            if  pi-rotation_angle>t
                car.dynamics.position = Destination;
                car.pathInfo.routeCompleted = true;
                car.pathInfo.lastWaypoint = car.map.get_waypoint_from_coordinates (Destination);
                
                %% TODO - check if it works in all situations
                idx = find(car.pathInfo.path==car.pathInfo.lastWaypoint);
                if idx+1<=length(car.pathInfo.path)
                    car.pathInfo.currentRoute = car.map.getRouteIDfromPath([car.pathInfo.path(idx) car.pathInfo.path(idx+1)]);
                end
                %%
                car.dynamics.cornering.angles = 0;
            else
                vector_velocity=[-a*sin(t)-cos(t)*c b a*cos(t)-c*sin(t)];
                vector=cross(vector_velocity, vector_z);
                vector=vector/norm(vector);
                theta=acos(dot(vector_velocity, vector_z)/(norm(vector_velocity)*norm(vector_z)));
                
          %      car.dynamics.position = [rotation_point(1)-(a*cos(t)-sin(t)*c) rotation_point(2)+b*t rotation_point(3)-(a*sin(t)+c*cos(t))];
                car.dynamics.position = [v_pos(1) 0 -v_pos(2)];
                car.dynamics.orientation = [vector -theta];
            end
            
        end
        
        function rotate_right(obj,car,speed, rotation_point,rotation_angle,Destination)
            %% WP generation
            if ~car.dynamics.has_local_trajectory
            obj.generate_right_rotation_WPs(car, speed, rotation_point,rotation_angle,Destination);
            end
            %%
            r = norm(Destination-rotation_point);
            
            if car.pathInfo.routeCompleted == true
                
                car.dynamics.cornering.angles = 0;
                car.pathInfo.routeCompleted = false;
                
                point_to_rotate= car.dynamics.position;
                
                car.dynamics.cornering.a=point_to_rotate(1)-rotation_point(1);
                car.dynamics.cornering.b=point_to_rotate(2)-rotation_point(2);
                car.dynamics.cornering.c=point_to_rotate(3)-rotation_point(3);
                
            end
            vector_z=[0 0 1];
            
            a = car.dynamics.cornering.a;
            b = car.dynamics.cornering.b;
            c = car.dynamics.cornering.c;
            
            step_length = speed/r;
            car.dynamics.cornering.angles = car.dynamics.cornering.angles + step_length;
            t = car.dynamics.cornering.angles;
            
            if  rotation_angle<t
                car.dynamics.position = Destination;
                car.pathInfo.routeCompleted = true;
                car.pathInfo.lastWaypoint = car.map.get_waypoint_from_coordinates (Destination);
                %% TODO - check if it works in all situations
                idx = find(car.pathInfo.path==car.pathInfo.lastWaypoint);
                if idx+1<=length(car.pathInfo.path)
                    car.pathInfo.currentRoute = car.map.getRouteIDfromPath([car.pathInfo.path(idx) car.pathInfo.path(idx+1)]);
                end
                %%
                
                car.dynamics.cornering.angles = 0;
            else
                vector_velocity=[-a*sin(t)-cos(t)*c b a*cos(t)-c*sin(t)];
                vector=cross(vector_velocity, vector_z);
                vector=vector/norm(vector);
                theta=acos(dot(vector_velocity, vector_z)/(norm(vector_velocity)*norm(vector_z)));
                
                car.dynamics.position = [rotation_point(1)+(a*cos(t)-sin(t)*c) rotation_point(2)+b*t rotation_point(3)+(a*sin(t)+c*cos(t))];
                car.dynamics.orientation =[vector -theta];
            end
            
            
        end
        
        function takeRoute(obj,car,speed,refRoute,v_pos)
            if car.status.stop ==1
                return;
            end
            
            %P_init = refRoute(1,:);
            P_final = refRoute(2,:);
            
            RotationVector = refRoute(3,:);
            rotation_angle = RotationVector(1);
            rotation_point = [RotationVector(2) 0 RotationVector(3)];
            
            
            if RotationVector(1) == 0 %Straight motion
                obj.moveto(car,speed,refRoute(2,:),v_pos);
                
            else %Rotational motion
                
                %Determine rotation direction: left or right
                if car.pathInfo.currentTrajectory(4,:) == -ones(1,3) % -1 means turn left
                    obj.rotate_left(car,speed, rotation_point,rotation_angle,P_final,v_pos);
                elseif car.pathInfo.currentTrajectory(4,:) == ones(1,3) % 1 means turn right
                    obj.rotate_right(car,speed, rotation_point,rotation_angle,P_final);
                end
                
                
            end
        end
        
        function nextMove(obj, car, speed, v_pos)
            % Examining 3 different states of the car
            % car.status.stop
            % car.pathInfo.destinationReached
            % car.pathInfo.routeCompleted
            if car.status.stop == true
                if car.pathInfo.destinationReached == true
                    %Car has finished its movement and stays still
                    car.pathInfo.routeCompleted = true;
                    car.dynamics.speed=0;
                    return;
                end
                
            elseif car.status.stop == false
                if car.pathInfo.destinationReached == true
                    % Car has just arrived so it has to stop
                    car.pathInfo.routeCompleted = true;
                    car.setStopStatus(true);
                    car.dynamics.speed = 0;
                    return;
                    
                elseif car.pathInfo.destinationReached == false
                    if car.pathInfo.routeCompleted == true
                        % Car has finished its current route
                        % check if car reached the destination
                        car.checkifDestinationReached();
                        if car.pathInfo.destinationReached == true
                            return;
                        else
                            % Car has to start the next route
                            obj.takeRoute(car,speed,car.pathInfo.currentTrajectory,v_pos);
                        end
                    elseif car.pathInfo.routeCompleted == false
                        % Car has to keep on going on its current route
                        obj.takeRoute(car,speed,car.pathInfo.currentTrajectory,v_pos);
                    end
                end
            end
            
        end
        %% New functions
        function localWPlist = generate_left_rotation_WPs(obj, car, speed, rotation_point,rotation_angle,Destination,v_pos)
%             position = car.dynamics.position;
%             localWPlist = [position(1) -position(3) car.dynamics.orientation(4)];
%             
%             r = norm(Destination-rotation_point);
%             
%             angles = pi;
%             
%             
%             
%             
%             while 1
%                 
%                 point_to_rotate= position;
%                 a=point_to_rotate(1)-rotation_point(1);
%                 b=point_to_rotate(2)-rotation_point(2);
%                 c=point_to_rotate(3)-rotation_point(3);
%                 
%                 vector_z=[0 0 1];
%                 
%                 step_length = speed/r;
%                 angles = angles - step_length;
%                 t = angles;
%                 
%                 if  pi-rotation_angle>t
%                     % Complete list of local waypoints to execute the turn left maneuver
%                     localWPlist = [localWPlist; position(1) -position(3) -theta];
%                     break;
%                 end
%                 vector_velocity=[-a*sin(t)-cos(t)*c b a*cos(t)-c*sin(t)];
%                 vector=cross(vector_velocity, vector_z);
%                 vector=vector/norm(vector);
%                 theta=acos(dot(vector_velocity, vector_z)/(norm(vector_velocity)*norm(vector_z)));
%                 
%                 position = [rotation_point(1)-(a*cos(t)-sin(t)*c) rotation_point(2)+b*t rotation_point(3)-(a*sin(t)+c*cos(t))];
%                 orientation = [vector -theta];
%                 
%                 localWPlist = [position(1) -position(3) -theta];
%                 
%             end
%             xRef = localWPlist(1,1);
%             yRef = localWPlist(1,2);
             position = car.dynamics.position;
%              localWPlist = [position(1) -position(3) -car.dynamics.orientation(4)];
             localWPlist = [position(1) 0 -position(3)];
%              speed = car.dynamics.speed;
             step_length = v_pos(4)*0.01*10;
             local_rotation_angle = pi/2;
             local_rotation_start_point = car.map.waypoints(obj.vehicle.pathInfo.lastWaypoint,:);
%              local_rotation_start_point = local_rotation_start_point*[1,1,-1];
             r = sqrt((norm(Destination-local_rotation_start_point))^2/(1-cos(local_rotation_angle))/2);
             step_angle = step_length/r; 
             local_rotation_point=[rotation_point(1) -rotation_point(3)];
             
             a = localWPlist(3)-local_rotation_point(2);
             
                local_position_P=[r,asin(a/r)];
                target_point_P = [r,asin(a/r)+step_angle];
                target_point_C = local_rotation_point+[r*cos(target_point_P(2)),r*sin(target_point_P(2))];
                 

            xRef = target_point_C(1);
            yRef = target_point_C(2);
            
            save('waypoints','xRef','yRef');
            figure(2);
            plot(xRef,yRef,'.','color','b');
        end
        
        function localWPlist = generate_right_rotation_WPs(obj, car,speed, rotation_point,rotation_angle,Destination)
            localWPlist = [car.dynamics.position(1) -car.dynamics.position(3) car.dynamics.orientation(4)];
            
            while 1
                
                r = norm(Destination-rotation_point);
                
                if car.pathInfo.routeCompleted == true
                    
                    car.dynamics.cornering.angles = 0;
                    car.pathInfo.routeCompleted = false;
                    
                    point_to_rotate= car.dynamics.position;
                    
                    car.dynamics.cornering.a=point_to_rotate(1)-rotation_point(1);
                    car.dynamics.cornering.b=point_to_rotate(2)-rotation_point(2);
                    car.dynamics.cornering.c=point_to_rotate(3)-rotation_point(3);
                    
                end
                vector_z=[0 0 1];
                
                a = car.dynamics.cornering.a;
                b = car.dynamics.cornering.b;
                c = car.dynamics.cornering.c;
                
                step_length = speed/r;
                car.dynamics.cornering.angles = car.dynamics.cornering.angles + step_length;
                t = car.dynamics.cornering.angles;
                
                if  rotation_angle<t
                    car.dynamics.position = Destination;
                    car.pathInfo.routeCompleted = true;
                    car.pathInfo.lastWaypoint = car.map.get_waypoint_from_coordinates (Destination);
                    %% TODO - check if it works in all situations
                    idx = find(car.pathInfo.path==car.pathInfo.lastWaypoint);
                    if idx+1<=length(car.pathInfo.path)
                        car.pathInfo.currentRoute = car.map.getRouteIDfromPath([car.pathInfo.path(idx) car.pathInfo.path(idx+1)]);
                    end
                    %%
                    
                    car.dynamics.cornering.angles = 0;
                    
                    % Complete list of local waypoints to execute the turn left maneuver
                    localWPlist = [localWPlist; car.dynamics.position(1) -car.dynamics.position(3) -theta];
                    break;
                end
                vector_velocity=[-a*sin(t)-cos(t)*c b a*cos(t)-c*sin(t)];
                vector=cross(vector_velocity, vector_z);
                vector=vector/norm(vector);
                theta=acos(dot(vector_velocity, vector_z)/(norm(vector_velocity)*norm(vector_z)));
                
                car.dynamics.position = [rotation_point(1)+(a*cos(t)-sin(t)*c) rotation_point(2)+b*t rotation_point(3)+(a*sin(t)+c*cos(t))];
                car.dynamics.orientation =[vector -theta];
                
                localWPlist = [localWPlist; car.dynamics.position(1) -car.dynamics.position(3) -theta];
            end
        end
        
        function localWPlist = generate_straight_move_WPs(obj, car,speed,Destination,v_pos)
            Destination = [Destination(1) Destination(2) -Destination(3)];
            localWPlist = [car.dynamics.position(1) -car.dynamics.position(3) car.dynamics.orientation(4)];
            
%             DisplacementVector = Destination- car.dynamics.position;
            DisplacementVector = Destination- [v_pos(1) 0 v_pos(2)];
            ThetaRadian = atan(DisplacementVector(1)/DisplacementVector(3));
%             DisplacementVector=[DisplacementVector(1) DisplacementVector(2) -DisplacementVector(3)];%transition matrix between MOBATSim coordi and real coordi


%             Position_Wps = car.dynamics.position;
              Position_Wps = [v_pos(1) 0 v_pos(2)];%test qihang
% 
%             while 1
                
  
%                 Position_Wps = Position_Wps + (DisplacementVector/norm(DisplacementVector))*(speed);
                Position_Wps = Position_Wps + (DisplacementVector/norm(DisplacementVector))*v_pos(4)*0.01;%test qihang
                
%                 if  norm(DisplacementVector/norm(DisplacementVector))*speed > norm(Destination-Position_Wps)
                if  norm(DisplacementVector/norm(DisplacementVector))*v_pos(4)*0.01 > norm(Destination-[v_pos(1) 0 v_pos(2)])%test qihang
%                     localWPlist = [localWPlist; Destination(1) Destination(3) -ThetaRadian];
                    localWPlist = [Destination(1) Destination(3) -ThetaRadian];%test qihang
                    
                    car.dynamics.has_local_trajectory =0;
%                     break;
                else
                
%                 localWPlist = [localWPlist; Position_Wps(1) -Position_Wps(3) -ThetaRadian];
                localWPlist = [ Position_Wps(1) Position_Wps(3) -ThetaRadian];%test qihang
                end
%             end
            
            xRef = localWPlist(1,1);
            yRef = localWPlist(1,2);
            
            save('waypoints','xRef','yRef');
            figure(2);
            plot(xRef,yRef,'.','color','b');
        end
    end
    
    
    
    %% Standard Simulink Output functions
    methods(Static,Access = protected)
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
        
        function [out,out2] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 3];
            out2 = [1 4];
            
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
        
        function [out,out2] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
        
        function [out,out2] = isOutputFixedSizeImpl(~)
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
