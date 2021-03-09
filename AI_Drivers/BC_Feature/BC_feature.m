classdef BC_feature < matlab.System
    % Untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties

    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        net
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            loadednet = load('./AI_Drivers/BC_Feature/BC_feature.mat');
            obj.net = loadednet;
        end

        function predictedData = stepImpl(obj,DataInput)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            DataInput1 = DataInput';
            predictedData = predict(obj.net.net,DataInput1,'MiniBatchSize',1);         
        end
        
        function [out1] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [1 1];
        end

        function [out1] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out1 = 'single';
        end

        function [out1] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out1 = false;
        end

        function [out1] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out1 = true;
        end
        
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
    end
end
