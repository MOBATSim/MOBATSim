classdef Ruler < ActivatablePlotObject
    %RULER A graphical plotted component that shows the distance between to objects
    %   Is designed for use in a birds-eye-plot from driving scenario
    %   toolbox
    % 
    %                 ^ 
    %                 | x-Axis
    %
    %                _ _
    %                 |
    %                 | label        
    % y-Axis <-      _|_             
    %              
    properties (Access = public)
        origin      % position of origin [x y]
        width       % width of ruler
        labelText   % text for describing the ruler measurement
    end
    
    properties (Access = private)
        lowerEnd    % lower cross beam
        middle      % middle connection part
        upperEnd    % upper cross beam
        label       % label that shows content near the middle part of the ruler       
    end
    
    methods
        function obj = Ruler(axes, origin, width, option)
            %RULER Construct an instance with origin at center lower end
            %and plot it into the parent axis component
            %   Detailed explanation goes here
            arguments
                axes                (1,1) matlab.ui.control.UIAxes
                origin              (1,2) double = [0 0]        % [x y]
                width               (1,1) double = 3
                option.length       (1,1) double = 20
                option.labelText    (1,1) string = ''
                option.color        (1,:)  = 'black'
                option.active       (1,1) logical = false       % object active after generation
            end

             % set properties
            obj.origin = origin;
            obj.width = width;
            obj.labelText = option.labelText;

            % position lines to draw a ruler
            xPos = obj.origin(1);
            yPos = obj.origin(2);
            obj.lowerEnd = line(axes,[xPos xPos],[yPos-width/2 yPos+width/2]);
            obj.middle = line(axes,[xPos xPos+option.length],[yPos yPos]);
            obj.upperEnd = line(axes,[xPos+option.length xPos+option.length],[yPos-width/2 yPos+width/2]);
            % change line width
            obj.lowerEnd.LineWidth = 1;
            obj.middle.LineWidth = 1;
            obj.upperEnd.LineWidth = 1;
            % add label
            obj.labelText = string(option.labelText);
            obj.label = text(axes, xPos+option.length/2, yPos-0.5, option.labelText);
                       
            % set super class properties
            obj.initialize(option.active);
        end
        
        function update(obj, length)
            % update the length of the ruler
            % when length is infinite, ruler is deactivated
            
            
            if ~obj.Active
                % dont update when not active
                return;
            end
                
            if length == inf
                % deactivate ruler when length is to long (infinite)
                obj.setVisibility(false);
            else
                
                % Update positions
                obj.middle.XData(2) = obj.middle.XData(1) + length;
                obj.upperEnd.XData(1) = obj.lowerEnd.XData(1) + length;
                obj.upperEnd.XData(2) = obj.lowerEnd.XData(2) + length;
                
                obj.label.Position(1) = obj.lowerEnd.XData(1) + length/2;
                obj.label.String = obj.labelText + sprintf(" %.2f m",length);
                % Show all components
                obj.setVisibility(true);
            end
        end
       
    end
    
    methods (Access = protected)
        %% protected methods
        
        function setVisibility(obj, visible)
            % Set the visibility of the ruler object
            
            obj.lowerEnd.Visible = visible;
            obj.middle.Visible = visible;
            obj.upperEnd.Visible = visible;
            obj.label.Visible = visible;
        end
        
    end
end

