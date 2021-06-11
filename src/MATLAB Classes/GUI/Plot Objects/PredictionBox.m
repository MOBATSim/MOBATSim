classdef PredictionBox < ActivatablePlotObject
    %PredictionBox A graphical plotted arrangement consisting of horizontal lines and a coverage area
    %   Is designed for use in a birds-eye-plot from driving scenario
    %   toolbox
    %
    %                 ^ 
    %                 | x-Axis
    %                   
    %                 ___
    %                |___|
    %                |___|
    %                |___|       
    % y-Axis <-                   
    %              
  
    properties (Access = public)
        origin      % position of origin [x y]
        width       % width of prediction box
    end
    
    properties (Access = private)
        bars            (:,:) Bar    % array of object Bar
        coverageArea            % object Coverage Area
    end
    
    methods
        function obj = PredictionBox(axes, origin, width, option)
            %PredictionBox Construct an instance with origin at xPos,yPos and
            %distance in x direction
            %   Detailed explanation goes here
            arguments
                axes                (1,1) matlab.ui.control.UIAxes
                origin              (1,2) double = [0 0]        % [x y]
                width               (1,1) double = 3
                option.colors       (:,:) = 'blue'              % one bar for every color-label pair
                option.labels       (1,:) string = ''           % one bar for every color-label pair
                option.active       (1,1) logical = false       % object active after generation
            end         
          
            % set properties
            obj.origin = origin;
            obj.width = width;
            
            % construct all bars
            for i=1:min([length(option.colors) length(option.labels)]) % one bar for every color-label pair
                obj.bars(end+1) = Bar(axes, origin, width, 'color',option.colors(i), 'labelText',option.labels(i), 'active',option.active);
            end
            % construct coverage area
            obj.coverageArea = CoverageArea(axes, origin, ... % start position
                                                  [width 0], ...
                                                  'faceColor', [0 1 0], ... % green
                                                  'faceAlpha', 0.15, ...
                                                  'edgeColor', [0 1 0], ... % green
                                                  'edgeAlpha', 0.15, ...
                                                  'active', option.active);
            
            % set super class properties
            obj.initialize(option.active);
        end
        
        function update(obj, distances)
            % update the length of the line
            % when distance is infinite, line is deactivated
            
            
            if ~obj.Active
                % deactivate component
                obj.setVisibility(false);
                return;
            end
                
            % Update bars
            for i=1:length(distances)
                obj.bars(i).update(distances(i));
            end
            %
            
            % Update coverage area
            xLowestBar = obj.origin(1) + min(distances); % x value of the bar with lowest position
            xHighestBar = obj.origin(1) + max(distances); % x value of the bar with highest position
            obj.coverageArea.update(xLowestBar, xHighestBar);
            
            % Show all components
            obj.setVisibility(true);
        end
       
    end
    
    methods (Access = protected)
        %% protected methods
        
        function setVisibility(obj, visible)
            % Set the visibility of the ruler object
            
            for i=1:length(obj.bars)
                obj.bars(i).setVisibility(visible);
            end
            obj.coverageArea.setVisibility(visible);
        end
    end
end

