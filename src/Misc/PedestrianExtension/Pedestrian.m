classdef Pedestrian < handle
    %PEDESTRIAN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        position
        orientation
        speed
    end
    
    methods
        function obj = Pedestrian(position,speed,orientation)
            %PEDESTRIAN Construct an instance of this class
            %   Detailed explanation goes here
            obj.position = position;            
            obj.speed = speed;
            obj.orientation = orientation
%             (3/2)*pi;
        end
        
        function newPosition = walk(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            newPosition = obj.position(2) + obj.orientation*obj.speed;
            obj.position(2)=newPosition;
        end
    end
end

