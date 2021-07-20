classdef V_WPGenerator_Stanley < WaypointGenerator
    % This blocks generates waypoints.
    %
    
    % Pre-computed constants
    properties(Access = private)
        referencePose = [0; 0; 0];
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
        
        
        
        
        function [poseOut, referencePose] = stepImpl(obj,pose,speed,changeLane)
            
            obj.registerVehiclePoseAndSpeed(obj.vehicle,pose,speed); % Sets/Registers vehicle current Pose and speed

            pose(3)=pose(3)*180/pi; % rad to deg
            
            %This block shouldn't run if the ego vehicle: (destinationReached or Collided)
            if obj.vehicle.status.collided || obj.vehicle.pathInfo.destinationReached
                
                poseOut=pose';                      % Output1: vehicle's actual pose
                referencePose = obj.referencePose'; % Output2: Reference pose
                
                return;
            elseif obj.vehicle.pathInfo.routeCompleted % The Vehicle determines the initial trajectory and route
                obj.vehicle.setInitialRouteAndTrajectory();
            end
              
            
            obj.takeRoute(obj.vehicle,obj.vehicle.pathInfo.currentTrajectory);
            %Output 1: Position of the vehicle
            %Output 2: Rotation angle of the vehicle
            

            
            referencePose = obj.referencePose';
            poseOut=pose';
            
        end
        
        
        function takeRoute(obj,car,refRoute)
            RotationVector = refRoute(3,:);
                        
            if (RotationVector(1) == 0) %Straight motion
                obj.move_straight(car,refRoute(2,:));
                
            else %Rotational motion
                
                P_final = refRoute(2,:);
                
                %Determine rotation direction: left or right
                if car.pathInfo.currentTrajectory(4,:) == -ones(1,3) % -1 means turn left
                    obj.rotate_left(car,P_final);
                    
                elseif car.pathInfo.currentTrajectory(4,:) == ones(1,3) % 1 means turn right
                    obj.rotate_right(car,P_final);
                end
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
            
            
            % ISSUE: Doesn't have meaning with LaneId-0.5
            %d=obj.laneWidth*(car.pathInfo.laneId-0.5)+obj.latOffset;
            d = obj.latOffset;
            latOffsetError = d-vehicle_d;%lateral offset error
            
            [targetPosition_C,~] = obj.Frenet2Cartesian(route,s,latOffsetError+d,radian);%Coordinate Conversion function
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
            
            % ISSUE: Doesn't have meaning with LaneId-0.5
            d=-obj.latOffset;%negative is only for left rotating vehicle
            
            latOffsetError = d-vehicle_d;%lateral offset error
            [targetPosition_C,~] = obj.Frenet2Cartesian(route,s,latOffsetError+d,radian);%Coordinate Conversion function
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
            
            % ISSUE: Doesn't have meaning with LaneId-0.5
            d=obj.latOffset;%for right rotating vehicle
            
            latOffsetError = d-vehicle_d;%lateral offset error
            [targetPosition_C,~] = obj.Frenet2Cartesian(route,s,latOffsetError+d,radian);%Coordinate Conversion function
            obj.referencePose = [targetPosition_C(1); targetPosition_C(2); orientation_C*180/pi];%Required format for the Stanley controller
            
        end

        function move_straight(obj,car,Destination)
            %% Reference Waypoint Generation
            obj.generateStraightWaypoints(car)
            car.checkWaypointReached(Destination);
        end
        
        function rotate_left(obj, car, Destination)
            %% Reference Waypoint Generation
            obj.generateLeftRotationWaypoints(car);
            car.checkWaypointReached(Destination);
        end
        
        function rotate_right(obj, car, Destination)
            %% Reference Waypoint Generation
            obj.generateRightRotationWaypoints(car);
            car.checkWaypointReached(Destination);
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
