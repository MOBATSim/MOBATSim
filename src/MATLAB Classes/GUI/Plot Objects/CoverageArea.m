classdef CoverageArea < ActivatablePlotObject
    %COVERAGEAREA  A graphical plotted component that shows a covered area
    %   Detailed explanation goes here
    %
    %    __
    %   |  |
    %   |  |            ^ x
    %   |__|            |
    %               y <- 
    properties (Access = public)
        origin      % position of origin [x y]
        dimension   % dimensions of object [width length]
    end
    properties (Access = private)
        polygon     % plotted polygon showing the coverage area
    end
    
    methods
        function obj = CoverageArea(axes, origin, dimension, option)
            %COVERAGEAREA generate an area that shows the coverage specified by
            % distance in front of the ego vehicle
            arguments
                axes                (1,1) matlab.ui.control.UIAxes
                origin              (1,2) double = [0 0]        % [x y]
                dimension           (1,2) double = [0.1 0.1]    % [width length]
                option.faceColor    (1,:)  = 'cyan'
                option.faceAlpha    (1,1) double = 0.3
                option.edgeColor    (1,:)  = 'cyan'
                option.edgeAlpha    (1,1) double = 0.3
                option.active       (1,1) logical = false       % object active after generation
            end
            
            % set properties
            obj.origin = origin;
            obj.dimension = dimension;
            
            % make a polygon for the coverage area
            width = dimension(1);
            length = dimension(2);
            % dont generate a polygon without area, does not work
            if width == 0, width = 0.1; end
            if length == 0, length = 0.1; end
            poly = polyshape([0 -width/2; ...
                               0 width/2; ...
                               length width/2; ...
                               length -width/2]);
                           
            % move to x,y - position
            xPos = obj.origin(1);
            yPos = obj.origin(2);
            poly = translate(poly, [xPos, yPos]);            
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
        
        function update(obj, xStart, xEnd)
            % update the x start and end positions
                        
            if ~obj.Active
                % dont update when not active
                return;
            end
            
            if xStart == xEnd
                % deactivate when there is no area to plot
                obj.setVisibility(false);
            else
                % Draw with new points (dont change the vertices, this is
                % buggy with small numbers)
                width = obj.dimension(1);
                obj.polygon.Shape = polyshape([xStart -width/2; ...
                               xStart width/2; ...
                               xEnd width/2; ...
                               xEnd -width/2]);

                % Show all components
                obj.setVisibility(true);
            end
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

