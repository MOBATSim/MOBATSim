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
        offset      % offset of the origin from the lower left edge             
    end
    
    properties (Access = private)
       polygon     % plotted polygon showing the car area
    end
    
    methods
        function obj = Car(axes, position, yaw, dimension,  option)
            %CAR generate an area that displays a car in the plot
            %   xPos, yPos      % origin, located at lower end of x axis and middle of the
            %                   % y axis of the component
            %   width, length   % dimensions of coverage area
            %   active          % activated at start?
            arguments
                axes                (1,1) matlab.ui.control.UIAxes
                position            (1,2) double = [0 0]        % [x y]                
                yaw                 (1,1) double = 0            % angle in rad of car in x,y-plane
                dimension           (1,2) double = [0.1 0.1]    % [width length]
                option.faceColor    (:,:)        = 'magenta'
                option.faceAlpha    (1,1) double = 1
                option.edgeColor    (:,:)        = 'magenta'
                option.edgeAlpha    (1,1) double = 1
                option.originOffset (1,2) double = [0 0]        % offset of the rotation center from geometric center
                option.active       (1,1) logical = false       % object active after generation
            end
            
            % Set properties
            obj.position = position;
            obj.dimension = dimension;
            obj.yaw = yaw;            
            
            width = dimension(1);
            length = dimension(2);
            % calculate offset from left lower edge to origin of the car
            % (rotation center)
            offset = [length/2+option.originOffset(1) width/2+option.originOffset(2)];
            % make a polygon with origin at (0,0)
            poly = polyshape([-offset(1) -offset(2); ...
                              -offset(1)  offset(2); ...
                               offset(1)  offset(2); ...
                               offset(1) -offset(2)]);
            obj.offset = offset; 

            % move to x,y - position
            poly = translate(poly, position); 
            % rotate with yaw angle around the rotation center (geometric
            % center + offset)
            poly = rotate(poly, rad2deg(yaw), position+offset);
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
        
        function update(obj, newPosition, newYaw)
            % update the position and angle
                        
            if ~obj.Active
                % dont update when not active
                return;
            end
                   
            % repaint at new position
            obj.polygon.Shape = polyshape([newPosition(1)-obj.offset(1) newPosition(2)-obj.offset(2); ...
                newPosition(1)-obj.offset(1) newPosition(2)+obj.offset(2); ...
                newPosition(1)+obj.offset(1) newPosition(2)+obj.offset(2); ...
                newPosition(1)+obj.offset(1) newPosition(2)-obj.offset(2)]);
            obj.position = newPosition;
            
            % rotate to new angle around origin
            obj.polygon.Shape = rotate(obj.polygon.Shape, rad2deg(newYaw), newPosition+obj.offset);
            obj.yaw = newYaw;
         
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

