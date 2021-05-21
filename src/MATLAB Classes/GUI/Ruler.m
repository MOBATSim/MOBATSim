classdef Ruler < handle
    %RULER A graphical plotted component that shows the distance between to objects
    %   Is designed for use in a birds-eye-plot from driving scenario
    %   toolbox
    
    properties (Access=private)
        lowerEnd    % lower cross beam
        middle      % middle connection part
        upperEnd    % upper cross beam
        label       % label that shows content near the middle part of the ruler
        visible     % visibility of the ruler
    end
    
    methods
        function obj = Ruler(parent, xPos, yPos, width, label)
            %RULER Construct an instance with origin at center lower end
            %and plot it into the parent axis component
            %   Detailed explanation goes here
            
            length = 20; % default value
            % position lines to draw a ruler
            obj.lowerEnd = line(parent,[xPos xPos],[yPos-width yPos+width]);
            obj.middle = line(parent,[xPos xPos+length],[yPos yPos]);
            obj.upperEnd = line(parent,[xPos+length xPos+length],[yPos-width yPos+width]);
            % change line width
            obj.lowerEnd.LineWidth = 1;
            obj.middle.LineWidth = 1;
            obj.upperEnd.LineWidth = 1;
            % add label
            obj.label = text(parent, xPos+length/2,yPos-1.5,label);
            % visiblity is true by default of these components
            obj.visible = true;
        end
        
        function updateLength(obj, length)
            % update the length of the ruler with value in bep units
            % when length is infinite, ruler is deactivated
            
            if length == inf
                % deactivate ruler when length is infinite
                obj.setVisibility(false);
            else
                obj.setVisibility(true);
                % Update positions
                obj.middle.XData(2) = obj.middle.XData(1) + length;
                obj.upperEnd.XData(1) = obj.lowerEnd.XData(1) + length;
                obj.upperEnd.XData(2) = obj.lowerEnd.XData(2) + length;
                
                obj.label.Position(1) = obj.lowerEnd.XData(1) + length/2;
            end
        end
        %% setter/getter
        function setVisibility(obj, visible)
            % Set the visibility of the ruler object
            
            if visible && ~obj.visible
                % make all parts visible
                obj.lowerEnd.Visible = true;
                obj.middle.Visible = true;
                obj.upperEnd.Visible = true;
                obj.label.Visible = true;
                % set property
                obj.visible = true;
            elseif ~visible && obj.visible
                % make all part invisible
                obj.lowerEnd.Visible = false;
                obj.middle.Visible = false;
                obj.upperEnd.Visible = false;
                obj.label.Visible = false;
                obj.visible = false;
            end
        end

    end
end

