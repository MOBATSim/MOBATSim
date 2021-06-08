classdef CoverageArea < ActivatablePlotObject
    %COVERAGEAREA  A graphical plotted component that shows a covered area
    %   Detailed explanation goes here
    %
    %    __
    %   |  |
    %   |  |            ^ x
    %   |__|            |
    %               y <- 
    
    properties (Access = private)
        polygon     % plotted polygon showing the coverage area
        width       % width of the plotted polygon
    end
    
    methods
        function obj = CoverageArea(axes,xPos, yPos, width, length, faceColor, faceAlpha, edgeColor, edgeAlpha, active)
            %COVERAGEAREA generate an area that shows the coverage specified by
            % distance in front of the ego vehicle
            %   xPos, yPos      % origin, located at lower end of x axis and middle of the
            %                   % y axis of the component
            %   width, length   % dimensions of coverage area
            %   active          % activated at start?
            
            % set active when not an input
            if nargin < 10
                 active = false;
            end
            
            % polygons with no width or length are not generated properly
            if width == 0, width = 0.1; end
            if length == 0, length = 0.1; end
            
            % make a polygon for the coverage area
            poly = polyshape([0 -width/2; ...
                               0 width/2; ...
                               length width/2; ...
                               length -width/2]);
            obj.width = width;
            % move to x,y - position
            poly = translate(poly, [xPos, yPos]);            
            % Create colored coverage area
            hold(axes, 'on');
            obj.polygon = plot(axes, poly, ...
                                     'FaceColor', faceColor, ...
                                     'EdgeColor', edgeColor, ...
                                     'FaceAlpha', faceAlpha, ...
                                     'EdgeAlpha', edgeAlpha);
            
            % set super class properties
            obj.initialize(active);
        end
        
        function update(obj, xStart, xEnd)
            % update the x start and end positions
                        
            if ~obj.Active
                % deactivate component
                obj.setVisibility(false);
                return;
            end
            
            if xStart == xEnd
                % deactivate when there is no area to plot
                obj.setVisibility(false);
            else
                % Draw with new points (dont change the vertices, this is
                % buggy with small numbers)
                obj.polygon.Shape = polyshape([xStart -obj.width/2; ...
                               xStart obj.width/2; ...
                               xEnd obj.width/2; ...
                               xEnd -obj.width/2]);

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

