classdef load_waypoints< matlab.System
    % untitled3 Add summary here
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

    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function [xRef, yRef] = stepImpl(obj)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            temp = load('waypoints');
            xRef=temp.xRef;
            yRef=temp.yRef;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
