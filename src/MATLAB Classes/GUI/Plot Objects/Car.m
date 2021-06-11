classdef Car < ActivatablePlotObject
    %CAR  A graphical plotted component displaying a car in bep
    %   Detailed explanation goes here
    %
    %    __
    %   |  |
    %   |  |            ^ x
    %   |__|            |
    %               y <- 
    
    properties (Access = public)
        position    % position of origin [x y]
        dimension   % dimensions of object [width length]
        yaw         % angle in rad of car in x,y-plane
        originOffset% offset of the origin from the middle
    end
    
    properties (Access = private)
        polygon     % plotted polygon showing the car area      
    end
    
    methods
        function obj = Car(axes, position, dimension, yaw, option)
            %CAR generate an area that displays a car in the plot
            %   xPos, yPos      % origin, located at lower end of x axis and middle of the
            %                   % y axis of the component
            %   width, length   % dimensions of coverage area
            %   active          % activated at start?
            arguments
                axes                (1,1) matlab.ui.control.UIAxes
                position            (1,2) double = [0 0]        % [x y]
                dimension           (1,2) double = [0.1 0.1]    % [width length]
                yaw                 (1,1) double = 0            % angle in rad of car in x,y-plane
                option.faceColor    (1,3) double = 'orange'
                option.faceAlpha    (1,1) double = 1
                option.edgeColor    (1,3) double = 'orange' 
                option.edgeAlpha    (1,1) double = 1
                option.originOffset (1,2) double = [0 0]        % offset of the rotation center from geometric center
                option.active       (1,1) logical = false       % object active after generation
            end
            
            % Set properties
            obj.position = position;
            obj.dimension = dimension;
            obj.yaw = yaw;
            obj.originOffset = option.originOffset;
            
            
            width = dimension(1);
            length = dimension(2);
            % make a polygon for the car
            poly = polyshape([0 -width/2; ...
                               0 width/2; ...
                               length width/2; ...
                               length -width/2]);
            % move to x,y - position
            poly = translate(poly, position);
            % rotate with yaw angle around the rotation center (geometric
            % center + offset)
            poly = rotate(poly, rad2deg(yaw), [width/2+option.originOffset(1) length/2+option.originOffset(2)]);
            % Create colored coverage area
            hold(axes, 'on');
            obj.polygon = plot(axes, poly, ...
                                     'FaceColor', option.faceColor, ...
                                     'EdgeColor', option.edgeColor, ...
                                     'FaceAlpha', option.faceAlpha, ...
                                     'EdgeAlpha', option.edgeAlpha);
            
            % set super class properties
            obj.initialize(option.active);
        end
        
        function update(obj, position, yaw)
            % update the x start and end positions
                        
            if ~obj.Active
                % deactivate component
                obj.setVisibility(false);
                return;
            end
            
            deltaPosition = obj.position-position;
            deltaYaw = obj.yaw-yaw;
            
            
            if deltaPosition ~= 0
                % translate to new position
                translate(obj.polygon.Shape, deltaPosition);
            end
            if deltaYaw ~= 0
                % rotate to new angle around origin in the middle
                rotate(obj.polygon.Shape, rad2deg(yaw), [obj.dimension(1)/2+obj.originOffset(1) ...
                                                         obj.dimension(2)/2+obj.originOffset(2)]);
            end
            
            % Show all components
            obj.setVisibility(true);
        end
        
    end
    
    methods (Access = protected)
        %% protected methods
        
        function setVisibility(obj, visible)
            % Set the visibility

            obj.polygon.Visible = visible;
        end
        
    end
end

