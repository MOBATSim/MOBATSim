classdef Arrow < ActivatablePlotObject
    %ARROW An arrow with flexible length
    %   Detailed explanation goes here
    %
    %   /|\
    %    |          ^ x
    %    |          |
    %           y <- 
    
    properties (Access = private)
        headLength      % length of the x or y part of one side of the arrow head
        head            % line that represents the head of the arrow
        shaft           % line that represents the shaft of the arrow
    end
    
    methods
        function obj = Arrow(axes, xPos, yPos, color, active)
            %ARROW Construct an arrow with origin at the arrow end
            %   Detailed explanation goes here
            
            length = 3; % default length
            obj.headLength = 2; % length of the x/y part of one side of the arrow head
            
            % make a arrow with 2 lines, one with sharp edges
            obj.head = line(axes,[xPos+length-obj.headLength xPos+length xPos+length-obj.headLength], ...
                                 [yPos+obj.headLength yPos yPos-obj.headLength], ...
                                 'Color', color, ...
                                 'LineWidth', 1.3);
            obj.shaft = line(axes,[xPos xPos+length], ...
                                  [yPos yPos], ...
                                  'Color', color, ...
                                  'LineWidth', 1.3);
            
            % set active
            if (nargin == 5) && (active == true)
                obj.Active = true;
            else
                obj.Active = false;
            end
        end
        
        function update(obj, length)
            % update the length of the arrow
            % when length is infinite, arrow is deactivated
            
            
            if ~obj.Active
                % deactivate component
                obj.setVisibility(false);
                return;
            end
            
            if length == inf
                % deactivate arrow when length is to long (infinite)
                obj.setVisibility(false);
            else
                
                % Update positions
                obj.shaft.XData(2) = obj.shaft.XData(1) + length;
                obj.head.XData(1) = obj.shaft.XData(2) - obj.headLength;
                obj.head.XData(2) = obj.shaft.XData(2);
                obj.head.XData(3) = obj.shaft.XData(2) - obj.headLength;
                % Show all components
                obj.setVisibility(true);
            end
        end
        
    end
    
    methods (Access = protected)
        %% protected methods
        
        function setVisibility(obj, visible)
            % Set the visibility of the arrow
            
            if visible
                % make all parts visible
                obj.head.Visible = true;
                obj.shaft.Visible = true;
            else
                % make all parts invisible
                obj.head.Visible = false;
                obj.shaft.Visible = false;
            end
        end
        
    end
end

