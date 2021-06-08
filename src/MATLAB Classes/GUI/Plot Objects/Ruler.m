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
       
    properties (Access = private)
        lowerEnd    % lower cross beam
        middle      % middle connection part
        upperEnd    % upper cross beam
        label       % label that shows content near the middle part of the ruler
        labelText   % text for describing the ruler measurement
    end
    
    methods
        function obj = Ruler(axes, xPos, yPos, width, labelText, active)
            %RULER Construct an instance with origin at center lower end
            %and plot it into the parent axis component
            %   Detailed explanation goes here
            
                        % set active
            if nargin < 6
                active = false;
            end 
            
            length = 20; % default value
            % position lines to draw a ruler
            obj.lowerEnd = line(axes,[xPos xPos],[yPos-width/2 yPos+width/2]);
            obj.middle = line(axes,[xPos xPos+length],[yPos yPos]);
            obj.upperEnd = line(axes,[xPos+length xPos+length],[yPos-width/2 yPos+width/2]);
            % change line width
            obj.lowerEnd.LineWidth = 1;
            obj.middle.LineWidth = 1;
            obj.upperEnd.LineWidth = 1;
            % add label
            obj.labelText = string(labelText);
            obj.label = text(axes, xPos+length/2, yPos-0.5, obj.labelText);
                       
            % set super class properties
            obj.initialize(active);
        end
        
        function update(obj, length)
            % update the length of the ruler
            % when length is infinite, ruler is deactivated
            
            
            if ~obj.Active
                % deactivate component
                obj.setVisibility(false);
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

