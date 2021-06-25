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
    
    properties (Access = public)
        origin      % position of origin [x y]
        width       % width of bar
    end
       
    properties (Access = private)
        hLine       % graphical horizontal line
        label       % label that shows content
    end
    
    methods
        function obj = Bar(axes, origin, width, option)
            %BAR Construct an instance with origin, defined width and at
            %least a label next to bar
            %   Detailed explanation goes here
            arguments
                axes                (1,1) matlab.ui.control.UIAxes
                origin              (1,2) double = [0 0]        % [x y]
                width               (1,1) double = 3
                option.labelText    (1,1) string = ''
                option.color        (1,:)  = 'green'
                option.active       (1,1) logical = false       % object active after generation
            end

            % set properties
            obj.origin = origin;
            obj.width = width;             
            
            
            % position the bar
            xPos = obj.origin(1);
            yPos = obj.origin(2);
            obj.hLine = line(axes,[xPos xPos],[yPos-width/2 yPos+width/2]);
            % change line width
            obj.hLine.LineWidth = 1;
            % set color
            obj.hLine.Color = option.color;
            % add label
            obj.label = text(axes, xPos, yPos-width/2-0.5, option.labelText);
            
            % set super class properties
            obj.initialize(option.active);
        end
        
        function update(obj, distance)
            % update the length of the line
            % when distance is infinite, line is deactivated
            
            
            if ~obj.Active
                % dont update when not active
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

