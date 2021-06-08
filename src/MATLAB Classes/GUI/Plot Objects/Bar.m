classdef Bar < ActivatablePlotObject
    %BAR A graphical plotted horizontal line with a defined distance to origin
    %   Is designed for use in a birds-eye-plot from driving scenario
    %   toolbox
    %
    %                 ^ 
    %                 | x-Axis
    %
    %                ___ label
    %                
    %                       
    % y-Axis <-                   
    %              
       
    properties (Access = private)
        hLine       % graphical horizontal line
        origin      % origin of the where the x-distance to the line is measured from
        label       % label that shows content
    end
    
    methods
        function obj = Bar(axes, xPos, yPos, width, color, labelText, active)
            %BAR Construct an instance with origin at xPos,yPos and
            %distance in x direction
            %   Detailed explanation goes here
            
            % set active
            if nargin < 7
                active = false;
            end 
            
            distance = 0; % default value
            % set origin
            obj.origin = [xPos yPos];
            % position the bar
            obj.hLine = line(axes,[xPos+distance xPos+distance],[yPos-width/2 yPos+width/2]);
            % change line width
            obj.hLine.LineWidth = 1;
            % set color
            obj.hLine.Color = color;
            % add label
            obj.label = text(axes, xPos+distance, yPos-width/2-0.5, string(labelText));
            
            % set super class properties
            obj.initialize(active);
        end
        
        function update(obj, distance)
            % update the length of the line
            % when distance is infinite, line is deactivated
            
            
            if ~obj.Active
                % deactivate component
                obj.setVisibility(false);
                return;
            end
                
            if distance == inf
                % deactivate ruler when length is to long (infinite)
                obj.setVisibility(false);
            else
                
                % Update positions
                obj.hLine.XData(1) = obj.origin(1) + distance;
                obj.hLine.XData(2) = obj.origin(1) + distance;
                obj.label.Position(1) = obj.origin(1) + distance;
                
                % Show all components
                obj.setVisibility(true);
            end
        end
       
    end
    
    methods (Access = protected)
        %% protected methods
        
        function setVisibility(obj, visible)
            % Set the visibility of the ruler object
            
            obj.hLine.Visible = visible;
            obj.label.Visible = visible;
        end
        
    end
end

