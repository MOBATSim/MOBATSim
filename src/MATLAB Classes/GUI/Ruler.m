classdef Ruler < handle
    %RULER A graphical plotted component that shows the distance between to objects
    %   Is designed for use in a birds-eye-plot from driving scenario
    %   toolbox
       
    properties (Access = private)
        lowerEnd    % lower cross beam
        middle      % middle connection part
        upperEnd    % upper cross beam
        label       % label that shows content near the middle part of the ruler
        labelText   % text for describing the ruler measurement
        visible     % visibility of the ruler components
        active      % ruler must be activated to be shown
        toLong      % ruler length is to long to show (inf)
    end
    
    methods
        function obj = Ruler(axes, xPos, yPos, width, labelText, active)
            %RULER Construct an instance with origin at center lower end
            %and plot it into the parent axis component
            %   Detailed explanation goes here
            
            length = 20; % default value
            % position lines to draw a ruler
            obj.lowerEnd = line(axes,[xPos xPos],[yPos-width yPos+width]);
            obj.middle = line(axes,[xPos xPos+length],[yPos yPos]);
            obj.upperEnd = line(axes,[xPos+length xPos+length],[yPos-width yPos+width]);
            % change line width
            obj.lowerEnd.LineWidth = 1;
            obj.middle.LineWidth = 1;
            obj.upperEnd.LineWidth = 1;
            % add label
            obj.labelText = labelText;
            obj.label = text(axes, xPos+length/2, yPos-0.5, labelText);
            % visiblity is true by default of these components
            obj.visible = true;
            
            % activate component
            if nargin == 6
                obj.setActive(active);
            end
        end
        
        function setLength(obj, length)
            % update the length of the ruler with value in bep units
            % when length is infinite, ruler is deactivated
            
            % only update if active
            if ~obj.active
                return;
            end
                
            if length == inf
                % deactivate ruler when length is to long (infinite)
                obj.toLong = true;
                obj.setVisibility();
            else
                obj.toLong = false;
                obj.setVisibility();
                % Update positions
                obj.middle.XData(2) = obj.middle.XData(1) + length;
                obj.upperEnd.XData(1) = obj.lowerEnd.XData(1) + length;
                obj.upperEnd.XData(2) = obj.lowerEnd.XData(2) + length;
                
                obj.label.Position(1) = obj.lowerEnd.XData(1) + length/2;
                obj.label.String = obj.labelText + sprintf(" %.2f m",length);
            end
        end
        %% Setter/Getter
        function setActive(obj, active)
            % activate/deactivate the ruler
            % when not active, the ruler is not shown
            
            % Set property
            obj.active = active;
            
            % Actualize visibilty
            obj.setVisibility();
        end
        
        function active = getActive(obj)
            
            active = obj.active;
        end
    end
    
    methods (Access = private)
        %% private methods
        
        function setVisibility(obj)
            % Set the visibility of the ruler object
            
            if obj.active && ~obj.toLong && ~obj.visible
                % make all parts visible, when active and not to long
                obj.lowerEnd.Visible = true;
                obj.middle.Visible = true;
                obj.upperEnd.Visible = true;
                obj.label.Visible = true;
                % set property
                obj.visible = true;
            elseif (~obj.active || obj.toLong) && obj.visible
                % make all part invisible, if not active or to long
                obj.lowerEnd.Visible = false;
                obj.middle.Visible = false;
                obj.upperEnd.Visible = false;
                obj.label.Visible = false;
                % set property
                obj.visible = false;
            end
        end
        
    end
end

