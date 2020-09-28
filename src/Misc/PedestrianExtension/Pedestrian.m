classdef Pedestrian < handle
    %PEDESTRIAN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        position
        orientation
        speed
    end
    
    methods
        function obj = Pedestrian(position,speed)
            %PEDESTRIAN Construct an instance of this class
            %   Detailed explanation goes here
            obj.position = position;
            obj.orientation = (3/2)*pi;
            obj.speed = speed;
        end
        
        function newPosition = walk(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            newPosition = obj.Property1 + inputArg;
        end
    end
end

