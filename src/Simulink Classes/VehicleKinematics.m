classdef VehicleKinematics < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime & matlab.system.mixin.CustomIcon
    % This blocks converts the speed of the vehicle to relevant position and rotation angles to generate the pose value.
    %
 
    % Public, tunable properties
    properties
        Vehicle_id
    end
    
    % Pre-computed constants
    properties(Access = private)
        vehicle
        map = evalin('base','Map');
        simSpeed = evalin('base','simSpeed');
        modelName = evalin('base','modelName');
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.vehicle = evalin('base',strcat('Vehicle',int2str(obj.Vehicle_id)));
        end
        
        function [position, rotation] = stepImpl(obj,speed)
            %This block shouldn't run if the vehicle has reached its destination or collided
            if obj.vehicle.status.collided || obj.vehicle.pathInfo.destinationReached
                
                position= obj.vehicle.dynamics.position; %Output 1: Position of the vehicle
                rotation= obj.vehicle.dynamics.orientation; %Output 2: Rotation angle of the vehicle
                return;
                
            elseif ~obj.vehicle.pathInfo.destinationReached
                
                obj.vehicle.updateActualSpeed(speed);
                
                if obj.vehicle.pathInfo.routeCompleted && obj.vehicle.status.stop ==0
                    obj.vehicle.setCurrentRoute(obj.generateCurrentRoute(obj.vehicle));
                    
                    currentTrajectory = obj.generateTrajectoryFromPath(obj.vehicle,obj.vehicle.pathInfo.path);
                    
                    obj.vehicle.setCurrentTrajectory(currentTrajectory)
                end
                % TODO Check the kinematic equations
                speedAccordingtoSimulation = speed*0.01*obj.simSpeed;
                %0.01 is the sample time but it needs to be automatically detected without a big overhead
                %obj.getSampleTime.SampleTime creates a huge overhead
                obj.nextMove(obj.vehicle,speedAccordingtoSimulation);
                
                position= obj.vehicle.dynamics.position; %Output 1: Position of the vehicle
                rotation= obj.vehicle.dynamics.orientation; %Output 2: Rotation angle of the vehicle
            end
            
        end
        
        function currentRoute = generateCurrentRoute(~,car)
            %% TODO - check if it works in all situations
            idx = find(car.pathInfo.path==car.pathInfo.lastWaypoint);
            if idx+1<=length(car.pathInfo.path)
                currentRoute = car.map.getRouteIDfromPath([car.pathInfo.path(idx) car.pathInfo.path(idx+1)]);
            else
                disp('Current Route error');
            end
        end
        
        function currentTrajectory = generateTrajectoryFromPath(~,car,path)
            
            % format of route for vehicle dynamics (translation)
            if (isempty(find((car.map.connections.translation(:,1) == path(1) )&(car.map.connections.translation(:,2)== path(2)), 1 )) == false)
                index = find((car.map.connections.translation(:,1) == path(1) )&(car.map.connections.translation(:,2)== path(2)) );
                currentTrajectory = [car.map.waypoints(car.map.connections.translation(index,1),:);
                    car.map.waypoints(car.map.connections.translation(index,2),:);
                    zeros(1,3);
                    zeros(1,3)];
            end
            % format of route for vehicle dynamics (curves)
            if (isempty(find((car.map.connections.circle(:,1) == path(1) )&(car.map.connections.circle(:,2)== path(2)), 1 )) == false)
                index = find((car.map.connections.circle(:,1) == path(1) )&(car.map.connections.circle(:,2)== path(2)) );
                currentTrajectory = [car.map.waypoints(car.map.connections.circle(index,1),:);
                    car.map.waypoints(car.map.connections.circle(index,2),:);
                    abs(car.map.connections.circle(index,3)),car.map.connections.circle(index,4),car.map.connections.circle(index,6);
                    -sign(car.map.connections.circle(index,3))*ones(1,3)];
            end
        end
        
        function takeRoute(obj,car,speed,refRoute)
            if car.status.stop ==1
                return;
            end
            
            %P_init = refRoute(1,:);
            P_final = refRoute(2,:);
            
            RotationVector = refRoute(3,:);
            rotation_angle = RotationVector(1);
            rotation_point = [RotationVector(2) 0 RotationVector(3)];
            
            
            if RotationVector(1) == 0 %Straight motion
                obj.moveto(car,speed,refRoute(2,:));
                
            else %Rotational motion
                
                %Determine rotation direction: left or right
                if car.pathInfo.currentTrajectory(4,:) == -ones(1,3) % -1 means turn left
                    obj.rotate_left(car,speed, rotation_point,rotation_angle,P_final);
                elseif car.pathInfo.currentTrajectory(4,:) == ones(1,3) % 1 means turn right
                    obj.rotate_right(car,speed, rotation_point,rotation_angle,P_final);
                end
                
                
            end
        end
        
        function nextMove(obj, car, speed)
            % Examining 3 different states of the car
            % car.status.stop
            % car.pathInfo.destinationReached
            % car.pathInfo.routeCompleted
            if car.status.stop == true
                if car.pathInfo.destinationReached == true
                    %Car has finished its movement and stays still
                    car.pathInfo.routeCompleted = true;
                    car.updateActualSpeed(0);
                    return;
                end
                
            elseif car.status.stop == false
                if car.pathInfo.destinationReached == true
                    % Car has just arrived so it has to stop
                    car.pathInfo.routeCompleted = true;
                    car.setStopStatus(true);
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
                            obj.takeRoute(car,speed,car.pathInfo.currentTrajectory);
                        end
                    elseif car.pathInfo.routeCompleted == false
                        % Car has to keep on going on its current route
                        obj.takeRoute(car,speed,car.pathInfo.currentTrajectory);
                    end
                end
            end
            
        end
        
        function moveto(~,car,speed,Destination)
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
            car.dynamics.position = car.dynamics.position + (car.dynamics.directionVector/norm(car.dynamics.directionVector))*(speed);
            
            
            if  norm(car.dynamics.directionVector/norm(car.dynamics.directionVector))*speed > norm(Destination-car.dynamics.position)
                car.dynamics.position = Destination; %Because of the rounding errors may be modified later
                car.pathInfo.routeCompleted=true;
                car.pathInfo.lastWaypoint = car.map.get_waypoint_from_coordinates (Destination);
                
                %% TODO - check if it works in all situations
                idx = find(car.pathInfo.path==car.pathInfo.lastWaypoint);
                if idx+1<=length(car.pathInfo.path)
                    car.setCurrentRoute(car.map.getRouteIDfromPath([car.pathInfo.path(idx) car.pathInfo.path(idx+1)]));
                end
            end
            
            
        end
        
        function rotate_left(~,car, speed, rotation_point,rotation_angle,Destination)
            
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
                    car.setCurrentRoute(car.map.getRouteIDfromPath([car.pathInfo.path(idx) car.pathInfo.path(idx+1)]));
                end
                %%
                car.dynamics.cornering.angles = 0;
            else
                vector_velocity=[-a*sin(t)-cos(t)*c b a*cos(t)-c*sin(t)];
                vector=cross(vector_velocity, vector_z);
                vector=vector/norm(vector);
                theta=acos(dot(vector_velocity, vector_z)/(norm(vector_velocity)*norm(vector_z)));
                
                car.dynamics.position = [rotation_point(1)-(a*cos(t)-sin(t)*c) rotation_point(2)+b*t rotation_point(3)-(a*sin(t)+c*cos(t))];
                car.dynamics.orientation = [vector -theta];
            end
            
        end
        
        function rotate_right(~,car,speed, rotation_point,rotation_angle,Destination)
            
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
                    car.setCurrentRoute(car.map.getRouteIDfromPath([car.pathInfo.path(idx) car.pathInfo.path(idx+1)]));
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
