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
    end
    
    methods
        function obj = CoverageArea(axes,xPos, yPos, width, length, color, edgeColor, active)
            %COVERAGEAREA generate an area that shows the coverage specified by
            % distance in front of the ego vehicle
            %   xPos, yPos      % origin, located at lower end of x axis and middle of the
            %                   % y axis of the component
            %   width, length   % dimensions of coverage area
            %   active          % activated at start?
            
            % polygons with no width or length are not generated properly
            if width == 0, width = 0.1; end
            if length == 0, length = 0.1; end
            
            % make a polygon for the coverage area
            poly = polyshape([0 -width/2; ...
                               0 width/2; ...
                               length width/2; ...
                               length -width/2]);
            % move to x,y - position
            poly = translate(poly, [xPos, yPos]);            
            % Create colored coverage area
            hold(axes, 'on');
            obj.polygon = plot(axes, poly, ...
                                     'FaceColor', color, ...
                                     'EdgeColor', edgeColor, ...
                                     'FaceAlpha', 0.5, ...
                                     'EdgeAlpha', 0.5);
            
            % set active
            if (nargin == 8) && (active == true)
                obj.Active = true;
            else
                obj.Active = false;
            end
        end
        
        function update(obj, xPosStart, xPosEnd)
            % update the x start and end positions
                        
            if ~obj.Active
                % deactivate component
                obj.setVisibility(false);
                return;
            end
            
            if xPosStart == xPosEnd
                % deactivate when there is no area to plot
                obj.setVisibility(false);
            else
                % Update the lower points
                obj.polygon.Shape.Vertices(1:2,1) = xPosStart;
                % Update the upper points
                obj.polygon.Shape.Vertices(3:4,1) = xPosEnd;
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

