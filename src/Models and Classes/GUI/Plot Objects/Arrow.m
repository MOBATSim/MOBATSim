classdef Arrow < ActivatablePlotObject
    %ARROW An arrow with flexible length
    %   Detailed explanation goes here
    %
    %   /|\
    %    |          ^ x
    %    |          |
    %           y <- 
    
    % Contributors: Johannes Pintscher
    
    properties (Access = public)
        origin      % position of origin [x y]
        headLength  % length of the x or y part of one side of the arrow head
    end
           
    properties (Access = private)        
        head            % line that represents the head of the arrow
        shaft           % line that represents the shaft of the arrow
    end
    
    methods
        function obj = Arrow(axes, origin, headLength, option)
            %ARROW Construct an arrow with origin at the arrow end
            %   Detailed explanation goes here
            arguments
                axes                (1,1) matlab.ui.control.UIAxes
                origin              (1,2) double = [0 0]        % [x y]
                headLength          (1,1) double = 2
                %option.labelText    (1,1) string = ''
                option.color        (1,:)  = 'black'
                option.active       (1,1) logical = false       % object active after generation
            end
            
            % set properties
            obj.origin = origin;
            obj.headLength = headLength;
            
            length = 3; % default length
            
            % make a arrow with 2 lines, one with sharp edges
            xPos = obj.origin(1);
            yPos = obj.origin(2);
            obj.head = line(axes,[xPos+length-obj.headLength xPos+length xPos+length-obj.headLength], ...
                                 [yPos+obj.headLength yPos yPos-obj.headLength], ...
                                 'Color', option.color, ...
                                 'LineWidth', 1.3);
            obj.shaft = line(axes,[xPos xPos+length], ...
                                  [yPos yPos], ...
                                  'Color', option.color, ...
                                  'LineWidth', 1.3);
            
            % set super class properties
            obj.initialize(option.active);
        end
        
        function update(obj, length)
            % update the length of the arrow
            % when length is infinite, arrow is deactivated
            
            
            if ~obj.Active
                % dont update when not active
                return;
            end
            
            if (length == inf) || (length == 0)
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
            
            obj.head.Visible = visible;
            obj.shaft.Visible = visible;

        end
        
    end
end

