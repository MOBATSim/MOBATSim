classdef V_WaypointGenerator < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime & matlab.system.mixin.CustomIcon
    % Untitled Add summary here
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
        modelName = evalin('base','modelName');
        
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.vehicle = evalin('base',strcat('Vehicle',int2str(obj.Vehicle_id)));
        end

        function [poseOut, referencePose] = stepImpl(obj, pose, speed)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
                        %transfer from local coordinate obj.vehicle.dynamics.speed = v_pos(4);
            pose(3)=pose(3)*180/pi; % rad to deg
            
            obj.vehicle.setPosition(obj.map.transformPoseTo3DAnim(pose));
            
            obj.vehicle.setYawAngle(pose(3)-1.5*pi);
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
                
                speedAccordingtoSimulation = speed*0.01;
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

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
