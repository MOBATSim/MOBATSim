classdef VehicleSafetyFilter < matlab.System & handle & matlab.system.mixin.Propagates
    % VEHICLESAFETYFILTER Limit target acceleration and steering angle
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
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.vehicle = evalin('base', "Vehicles(" + obj.Vehicle_id + ")");
        end

        function [Acc,Stop,Steering] = stepImpl(obj,Acc,Steering)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            if obj.vehicle.status.collided || obj.vehicle.pathInfo.destinationReached
                Acc =0;
                Steering = 0;
                Stop = 1;                
            else                
                Stop = 0;
                
                %[Acc, Stop] = obj.injectFaultsScenario(Acc, Stop);
                
            end
            
            
            % safe values in vehicle properties
            obj.vehicle.dynamics.acceleration = Acc;
            obj.vehicle.dynamics.steeringAngle = Steering;
        end
        
        function [Acc, Stop] = injectFaultsScenario(obj, Acc, Stop)
            % This is a test for fault injection with a reactive action
            % tested with scenario "Vehicle Safety Filter Test"
            
            
            if obj.vehicle.id == 3 % following vehicle
                % choose acc so that stopping with acc  = -3 is possible
                % before the current position of vehicle in front
                
                % get distance to stop
                curSpeed = obj.vehicle.dynamics.speed;
                minAcceleration = -2;
                defaultSafeDistance = 8;
                % calculate distance to stop
                distToStop = 0.5*-curSpeed^2/minAcceleration;
                
                % get distance to vehicle in front
                deltaDistance = obj.vehicle.sensors.distanceToLeadingVehicle;
                
                % brake when to near to vehicle
                if (distToStop+defaultSafeDistance) > deltaDistance
                    Acc = minAcceleration;
                end
                              
              
            % add uncertainty to acceleration of leading
            elseif obj.vehicle.id == 1 % leading vehicle
                
                meanValue = 0;
                variance = 1;
                Acc_wnoise = Acc + sqrt(variance)*randn(size(Acc)) + meanValue;
                
                Acc = Acc_wnoise;
                
                % suddenly stop at 7 seconds
                if get_param('MOBATSim','SimulationTime') >= 7
                    Stop = 1;
                end
                if Stop == 1
                    Acc = 0;
                end
                
                
            end
            
            
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

        function [out,out2,out3] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [1 1];
            out2 = [1 1];
            out3 = [1 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out,out2,out3] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "double";
            out2 = "double";
            out3 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out,out2,out3] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out,out2,out3] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
end
