classdef VehicleKinematics_Qihang < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime & matlab.system.mixin.CustomIcon
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
        T=2;%time duration of the whole lane-switch procss
        Lane_width = 10;
        local_route_direction = [];
        local_LeftAdjacentLane_direction = [];
        local_RightAdjacentLane_direction = [];
        LaneSwitch_stratrPoint = [];
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
        function [position, rotation, reference_waypoints] = stepImpl(obj,v_pos,drivingMode)

%            vehicle's current pos from the vehicle dynamic model
%            v_pos = [X Y psi speed];
            obj.vehicle.dynamics.position = [v_pos(1) 0 -v_pos(2)];%transfer from local coordinate to MOBATSim coordinate
            obj.vehicle.dynamics.speed = v_pos(4);
            obj.vehicle.dynamics.orientation = [0 1 0 v_pos(3)-1.5*pi];
            
            if obj.vehicle.status.collided
                %Output 1: Position of the vehicle
                position= obj.vehicle.dynamics.position;
                %Output 2: Rotation angle of the vehicle
                rotation= obj.vehicle.dynamics.orientation;
                %Output 3: reference waypoints
                reference_waypoints = obj.vehicle.dynamics.reference_waypoints(:,[1 3]);
                return;
            end
            
            if ~obj.vehicle.pathInfo.destinationReached
                %obj.vehicle.dynamics.speed = v_pos(4);
                if obj.vehicle.pathInfo.routeCompleted && obj.vehicle.status.stop == 0
                   obj.vehicle.pathInfo.currentRoute = obj.setCurrentRoute(obj.vehicle);
                   obj.vehicle.pathInfo.currentTrajectory = obj.generateTrajectory(obj.vehicle);
                elseif drivingMode == 5
                end
                obj.nextMove(obj.vehicle);
            end
            %Output 1: Position of the vehicle
            position= obj.vehicle.dynamics.position;
            %Output 2: Rotation angle of the vehicle
            rotation= obj.vehicle.dynamics.orientation;
            %Output 3: Reference waypoints for pure pursuit controller
            reference_waypoints = obj.vehicle.dynamics.reference_waypoints(:,[1 3]);
%             figure(2)
%             WP = plot(reference_waypoints(:,1),reference_waypoints(:,2),'.','color','blue');
%             pos = plot(obj.vehicle.dynamics.position(1),-obj.vehicle.dynamics.position(3),'.','color','red');
%             xlim([obj.vehicle.dynamics.position(1)-200 obj.vehicle.dynamics.position(1)+200]);
%             ylim([-obj.vehicle.dynamics.position(3)-200 -obj.vehicle.dynamics.position(3)+200]);           
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
        
        
        function nextMove(obj, car)
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
                   % car.dynamics.speed = 0;
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
                            obj.takeRoute(car,car.pathInfo.currentTrajectory);
                        end
                    elseif car.pathInfo.routeCompleted == false
                        % Car has to keep on going on its current route
                        obj.takeRoute(car,car.pathInfo.currentTrajectory);
                    end
                end
            end
            
        end
     
        
        function takeRoute(obj,car,refRoute)
            if car.status.stop ==1
                return;
            end
            
            if car.status.lane_switching == 1
                if isempty(car.dynamics.LaneSwitch_targetPoint)
                    obj.generate_target_point(car);
                    obj.SwitchLane_trajectory_generator(car);
                end
                obj.take_SwitchLane(car);
                return;
            end
            
            P_init = refRoute(1,:);
            P_final = refRoute(2,:);
            
            RotationVector = refRoute(3,:);
            rotation_angle = RotationVector(1);
            rotation_point = [RotationVector(2) 0 RotationVector(3)];
            
            
            if RotationVector(1) == 0 %Straight motion
                obj.moveto(car,refRoute(2,:));
                
            else %Rotational motion
                
                %Determine rotation direction: left or right
                if car.pathInfo.currentTrajectory(4,:) == -ones(1,3) % -1 means turn counterclockwise
                    obj.rotate_counterclockwise(car, rotation_point,rotation_angle,P_final);
                elseif car.pathInfo.currentTrajectory(4,:) == ones(1,3) % 1 means turn right
                    obj.rotate_clockwise(car, rotation_point,-rotation_angle,P_final);
                end
                
                
            end
        end
        
        function generate_target_point(obj,car) 
            obj.local_route_direction = (car.pathInfo.currentTrajectory(2,:)-car.pathInfo.currentTrajectory(1,:)).*[1 1 -1]/norm(car.pathInfo.currentTrajectory(2,:)-car.pathInfo.currentTrajectory(1,:));
            obj.local_LeftAdjacentLane_direction = cross(obj.local_route_direction,[0 1 0]);
            obj.local_RightAdjacentLane_direction = cross([0 1 0],obj.local_route_direction);
            obj.vehicle.dynamics.LaneSwitch_targetPoint=car.dynamics.position.*[1 1 -1]+obj.T*car.dynamics.speed*obj.local_route_direction+obj.local_RightAdjacentLane_direction*obj.Lane_width;
        end
        
        function SwitchLane_trajectory_generator(obj,car) %generates trajectory for lane change
            local_current_point = car.dynamics.position.*[1 1 -1];
            local_target_point = car.dynamics.LaneSwitch_targetPoint;
            a0=car.dynamics.position.*[1 1 -1].*abs(obj.local_RightAdjacentLane_direction);
            a0=a0(3);
            a1=0;
            a2=0;
            
            x_f = car.dynamics.LaneSwitch_targetPoint(1); % Final x coordinate
            y_f = car.dynamics.LaneSwitch_targetPoint(3); % Final y coordinate

            %%  Minimun jerk trajectory function for the calculation in y direction (Lateral)
           
            syms a3 a4 a5;
            [a3,a4,a5]=solve([a0+a1*obj.T+a2*obj.T^2+a3*obj.T^3+a4*obj.T^4+a5*obj.T^5==y_f, ... % Boundary condition for lateral displacement
                a1+2*a2*obj.T+3*a3*obj.T^2+4*a4*obj.T^3+5*a5*obj.T^4==0, ...              % Boundary condition for lateral speed
                2*a2+6*a3*obj.T+12*a4*obj.T^2+20*a5*obj.T^3==0,],[a3,a4,a5]);    % Boundary condition for lateral acceleration
            
            % Solving for coefficients and conversion to double precision
            a3=double(a3);
            a4=double(a4);
            a5=double(a5);
            obj.vehicle.dynamics.LaneSwitch_trajectory = [a0 a1 a2 a3 a4 a5];
        end
       
        function take_SwitchLane(obj,car)
            f='a0+a1*t+a2*t.^2+a3*t.^3+a4*t.^4+a5*t.^5';
        end
       function moveto(obj ,car,Destination)
            %% Waypoint generation
            if ~car.dynamics.has_local_trajectory
            obj.generate_straight_move_WPs(car,Destination);
            end
            %%
            
            %Displacement Vector and determination of its Angle
            if car.pathInfo.routeCompleted == true
                DisplacementVector = Destination- car.dynamics.position;
%                 ThetaRadian = atan(DisplacementVector(1)/DisplacementVector(3));
%                 car.dynamics.orientation(4) = mod(v_pos(3),2*pi)-0.5*pi;
                car.dynamics.directionVector = DisplacementVector;
                car.pathInfo.routeCompleted = false;
            end
            
            % For Intercardinal directions: Depending on the four quadrants, the problem the atan function is that it
            % is between -pi and pi so that it doesn't correctly cover a 360 degrees
            % angle, therefore the quadrant check and pi addition must take place to
            % make sure that the vehicle rotates correctly
%             ThetaRadian = car.dynamics.orientation(4);
%             
%             % Intercardinal Directions
%             if car.dynamics.directionVector(1)<0
%                 if car.dynamics.directionVector(3)<0
%                     ThetaRadian= ThetaRadian+pi;
%                 end
%             elseif car.dynamics.directionVector(1)>0
%                 if car.dynamics.directionVector(3)<0
%                     ThetaRadian= ThetaRadian+pi;
%                 end
%                 
%                 % Vertical and horizontal movements - Cardinal Directions
%                 % Movement z direction
%             elseif car.dynamics.directionVector(1)==0
%                 
%                 % Positive z direction
%                 if car.dynamics.directionVector(3)>0
%                     ThetaRadian = 0;
%                     
%                     % Negative z direction
%                 elseif car.dynamics.directionVector(3)<0
%                     ThetaRadian =-pi;
%                 end
%                 
%                 % Movement x direction
%             elseif car.dynamics.directionVector(3)==0
%                 % Positive x direction
%                 if car.dynamics.directionVector(1)>0
%                     ThetaRadian= pi/2;
%                     % Negative x direction
%                 elseif car.dynamics.directionVector(3)<0
%                     ThetaRadian= -pi/2;
%                 end
%                 
%             end
%             
%             
%             car.dynamics.orientation = [ 0 1 0 ThetaRadian];
            
            % TODO Check if we need to add the sample time of the simulation into this equation below
%             car.dynamics.position = car.dynamics.position + (car.dynamics.directionVector/norm(car.dynamics.directionVector))*(speed);

           % car.dynamics.position = [v_pos(1) 0 -v_pos(2)];%test qihang
            
%             if  norm(car.dynamics.directionVector/norm(car.dynamics.directionVector))*speed > norm(Destination-car.dynamics.position)
            if  norm(car.dynamics.position-Destination)<2
%                 car.dynamics.position = Destination; %Because of the rounding errors may be modified later
                car.pathInfo.routeCompleted=true;
                car.pathInfo.lastWaypoint = car.map.get_waypoint_from_coordinates (Destination);
                
                %% TODO - check if it works in all situations
                idx = find(car.pathInfo.path==car.pathInfo.lastWaypoint);
                if idx+1<=length(car.pathInfo.path)
                    car.pathInfo.currentRoute = car.map.getRouteIDfromPath([car.pathInfo.path(idx) car.pathInfo.path(idx+1)]);
                end
            end
            
            
       end
        
        
       function generate_straight_move_WPs(obj, car,Destination)
                local_destination = Destination.*[1 1 -1];
%                Destination = [Destination(1) Destination(2) -Destination(3)];
%                localWPlist = [car.dynamics.position(1) -car.dynamics.position(3) car.dynamics.orientation(4)];
            
%                DisplacementVector = Destination- car.dynamics.position;
%                DisplacementVector = Destination- [v_pos(1) 0 v_pos(2)];
                route_Vector = car.pathInfo.currentTrajectory(2,:)-car.pathInfo.currentTrajectory(1,:);
                local_route_Vector_i = route_Vector/norm(route_Vector).*[1 1 -1];
            
%                ThetaRadian = atan(local_route_Vector_i(3)/local_route_Vector_i(1));
            
                %unit vector starting with route start point, aiming to
                %vehicle's current location
                pos_Vector = car.dynamics.position-car.pathInfo.currentTrajectory(1,:);
                local_pos_Vector = pos_Vector.*[1 1 -1];
            
                setoff_distance = dot(local_pos_Vector,local_route_Vector_i);
                local_WP_start_point = (setoff_distance*local_route_Vector_i+car.pathInfo.currentTrajectory(1,:).*[1 1 -1]);
            
            

                 for i = 1:1:length(car.dynamics.reference_waypoints)
                     car.dynamics.reference_waypoints(i,:) = local_WP_start_point+norm(i*(car.dynamics.speed*0.01*100))*local_route_Vector_i;
                     %30 is the factor that adjust the reference WP step length
                 end

        end


     function rotate_counterclockwise(obj,car,rotation_point,rotation_angle,Destination)
            
            %% WP generation
            if ~car.dynamics.has_local_trajectory
            obj.generate_rotate_counterclockwise_WPs(car,rotation_point,rotation_angle,Destination);
            end
            %%
            if car.pathInfo.routeCompleted == true
                DisplacementVector = Destination- car.dynamics.position;
%                 ThetaRadian = atan(DisplacementVector(1)/DisplacementVector(3));
%                 car.dynamics.orientation(4) = mod(v_pos(3),2*pi)-0.5*pi;
                car.dynamics.directionVector = DisplacementVector;
                car.pathInfo.routeCompleted = false;
            end
            
            if  norm(car.dynamics.position-Destination)<4
%                 car.dynamics.position = Destination; %Because of the rounding errors may be modified later
                car.pathInfo.routeCompleted=true;
                car.pathInfo.lastWaypoint = car.map.get_waypoint_from_coordinates (Destination);
                
                %% TODO - check if it works in all situations
                idx = find(car.pathInfo.path==car.pathInfo.lastWaypoint);
                if idx+1<=length(car.pathInfo.path)
                    car.pathInfo.currentRoute = car.map.getRouteIDfromPath([car.pathInfo.path(idx) car.pathInfo.path(idx+1)]);
                end
            end
%              position = car.dynamics.position;
%              local_position = [position(1) 0 -position(3)];
%              step_length = v_pos(4)*0.01*10;
%              local_rotation_angle = pi/2;
%              local_rotation_start_point = car.map.waypoints(obj.vehicle.pathInfo.lastWaypoint,:);
%              r = sqrt((norm(Destination-local_rotation_start_point))^2/(1-cos(local_rotation_angle))/2);
%              step_angle = step_length/r; 
%              rotation_center=[rotation_point(1) -rotation_point(3)];
%              
%              l = norm(local_position - [rotation_center(1) 0 rotation_center(2)]);
%              a = local_position(3)-rotation_center(2);
%              
%                 local_position_P=[l,asin(a/l)];
%                 target_point_P = [r,asin(a/l)+step_angle];
%                 target_point_C = rotation_center+[r*cos(target_point_P(2)),r*sin(target_point_P(2))];
%            
%             if  (asin(a/l)+step_angle)>local_rotation_angle
%                 car.dynamics.position = Destination;
%                 car.pathInfo.routeCompleted = true;
%                  car.pathInfo.lastWaypoint = car.map.get_waypoint_from_coordinates (Destination);
%                 %% TODO - check if it works in all situations
%                 idx = find(car.pathInfo.path==car.pathInfo.lastWaypoint);
%                 if idx+1<=length(car.pathInfo.path)
%                     car.pathInfo.currentRoute = car.map.getRouteIDfromPath([car.pathInfo.path(idx) car.pathInfo.path(idx+1)]);
%                 end
            end
        function generate_rotate_counterclockwise_WPs(obj, car, rotation_point,rotation_angle,Destination)

%              position = car.dynamics.position;
%              localWPlist = [position(1) -position(3) -car.dynamics.orientation(4)];
             local_position = car.dynamics.position.*[1 1 -1];
%              speed = car.dynamics.speed;
             step_length = car.dynamics.speed*0.01;
             %10 is look ahead faktor
             local_rotation_angle = car.pathInfo.currentTrajectory(3,1);
             %rotation angle of the whole curved road
             local_rotation_start_point = car.map.waypoints(obj.vehicle.pathInfo.lastWaypoint,:).*[1 1 -1];
%              local_rotation_start_point = local_rotation_start_point*[1,1,-1];
             r = sqrt((norm(Destination.*[1 1 -1]-local_rotation_start_point))^2/(1-cos(local_rotation_angle))/2);
             step_angle = step_length/r; 
             local_displacement_vector = (Destination.*[1 1 -1]-local_rotation_start_point)/norm(Destination.*[1 1 -1]-local_rotation_start_point);
             local_r_angle = acos(dot(local_displacement_vector,[1 0 0]));
             if (local_displacement_vector(3)<0)
                 local_r_angle=2*pi-local_r_angle;
             end
          
             local_plumb_length = cos(local_rotation_angle/2)*r;
             local_plubm_vector = [cos(local_r_angle+pi/2) 0 sin(local_r_angle+pi/2)]*local_plumb_length;
             local_rotation_center = local_rotation_start_point + local_displacement_vector*norm(Destination.*[1 1 -1]-local_rotation_start_point)/2 + local_plubm_vector;
             
             l = local_position - local_rotation_center;
             d = norm(cross(local_rotation_start_point-local_rotation_center,local_position-local_rotation_center))/norm(local_rotation_start_point-local_rotation_center);
             local_start_angle = acos(dot(l,[1 0 0])/norm(l));
             if (l(3)<0)
                 local_start_angle=2*pi-local_start_angle;
             end                
              for i = 1:1:length(car.dynamics.reference_waypoints)
                  target_point_P = [r,local_start_angle+step_angle*(i-1)*30];
                  target_point_C = local_rotation_center+[r*cos(target_point_P(2)) 0 r*sin(target_point_P(2))];
                  car.dynamics.reference_waypoints(i,:) = target_point_C;
              end
        end
        
        function rotate_clockwise(obj,car,rotation_point,rotation_angle,Destination)
            
            %% WP generation
            if ~car.dynamics.has_local_trajectory
            obj.generate_clockwise_rotation_WPs(car,rotation_point,rotation_angle,Destination);
            end
            %%
            if car.pathInfo.routeCompleted == true
                DisplacementVector = Destination- car.dynamics.position;
%                 ThetaRadian = atan(DisplacementVector(1)/DisplacementVector(3));
%                 car.dynamics.orientation(4) = mod(v_pos(3),2*pi)-0.5*pi;
                car.dynamics.directionVector = DisplacementVector;
                car.pathInfo.routeCompleted = false;
            end
            
            if  norm(car.dynamics.position-Destination)<4
%                 car.dynamics.position = Destination; %Because of the rounding errors may be modified later
                car.pathInfo.routeCompleted=true;
                car.pathInfo.lastWaypoint = car.map.get_waypoint_from_coordinates (Destination);
                
                %% TODO - check if it works in all situations
                idx = find(car.pathInfo.path==car.pathInfo.lastWaypoint);
                if idx+1<=length(car.pathInfo.path)
                    car.pathInfo.currentRoute = car.map.getRouteIDfromPath([car.pathInfo.path(idx) car.pathInfo.path(idx+1)]);
                end
            end
%              position = car.dynamics.position;
%              local_position = [position(1) 0 -position(3)];
%              step_length = v_pos(4)*0.01*10;
%              local_rotation_angle = pi/2;
%              local_rotation_start_point = car.map.waypoints(obj.vehicle.pathInfo.lastWaypoint,:);
%              r = sqrt((norm(Destination-local_rotation_start_point))^2/(1-cos(local_rotation_angle))/2);
%              step_angle = step_length/r; 
%              rotation_center=[rotation_point(1) -rotation_point(3)];
%              
%              l = norm(local_position - [rotation_center(1) 0 rotation_center(2)]);
%              a = local_position(3)-rotation_center(2);
%              
%                 local_position_P=[l,asin(a/l)];
%                 target_point_P = [r,asin(a/l)+step_angle];
%                 target_point_C = rotation_center+[r*cos(target_point_P(2)),r*sin(target_point_P(2))];
%            
%             if  (asin(a/l)+step_angle)>local_rotation_angle
%                 car.dynamics.position = Destination;
%                 car.pathInfo.routeCompleted = true;
%                  car.pathInfo.lastWaypoint = car.map.get_waypoint_from_coordinates (Destination);
%                 %% TODO - check if it works in all situations
%                 idx = find(car.pathInfo.path==car.pathInfo.lastWaypoint);
%                 if idx+1<=length(car.pathInfo.path)
%                     car.pathInfo.currentRoute = car.map.getRouteIDfromPath([car.pathInfo.path(idx) car.pathInfo.path(idx+1)]);
%                 end
            end
        function generate_clockwise_rotation_WPs(obj, car, rotation_point,rotation_angle,Destination)

%              position = car.dynamics.position;
%              localWPlist = [position(1) -position(3) -car.dynamics.orientation(4)];
             local_position = car.dynamics.position.*[1 1 -1];
%              speed = car.dynamics.speed;
             step_length = car.dynamics.speed*0.01;
             %10 is look ahead faktor
             local_rotation_angle = -car.pathInfo.currentTrajectory(3,1);
             %rotation angle of the whole curved road
             local_rotation_start_point = car.map.waypoints(obj.vehicle.pathInfo.lastWaypoint,:).*[1 1 -1];
%              local_rotation_start_point = local_rotation_start_point*[1,1,-1];
             r = sqrt((norm(Destination.*[1 1 -1]-local_rotation_start_point))^2/(1-cos(local_rotation_angle))/2);
             step_angle = step_length/r; 
             local_displacement_vector = (Destination.*[1 1 -1]-local_rotation_start_point)/norm(Destination.*[1 1 -1]-local_rotation_start_point);
             local_r_angle = acos(dot(local_displacement_vector,[1 0 0]));
             if (local_displacement_vector(3)<0)
                 local_r_angle=2*pi-local_r_angle;
             end
             local_plumb_length = cos(local_rotation_angle/2)*r;
             local_plubm_vector = [cos(local_r_angle-pi/2) 0 sin(local_r_angle-pi/2)]*local_plumb_length;
             local_rotation_center = local_rotation_start_point + local_displacement_vector*norm(Destination.*[1 1 -1]-local_rotation_start_point)/2 + local_plubm_vector;
             
             l = local_position - local_rotation_center;
             d = norm(cross(local_rotation_start_point-local_rotation_center,local_position-local_rotation_center))/norm(local_rotation_start_point-local_rotation_center);
             l_angle = -acos(dot(l,[1 0 0])/norm(l));
             if(l(3)>=0)
                 l_angle = l_angle-2*(pi+l_angle);
             end
              for i = 1:1:length(car.dynamics.reference_waypoints)
                  target_point_P = [r,mod(l_angle-step_angle*(i-1)*30,-2*pi)];
                  target_point_C = local_rotation_center+[r*cos(target_point_P(2)) 0 r*sin(target_point_P(2))];
                  car.dynamics.reference_waypoints(i,:) = target_point_C;
              end
        end
    end
    
    
    
    %% Standard Simulink Output functions
    methods(Static,Access = protected)
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
        
        function [out,out2,out3] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 3];
            out2 = [1 4];
            out3 = [10 2];
            
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
        
        function [out,out2,out3] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = 'double';
            out2 = 'double';
            out3 = 'double';
            
            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end
        
        function [out,out2,out3] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
            
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
        
        function [out,out2,out3] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
            
            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
        
        function sts = getSampleTimeImpl(obj)
            % Define sample time type and parameters
            sts = obj.createSampleTime("Type", "Inherited");
        end
    
        end
end