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
       
    properties (Access = private)
        bars            % array of object Bar
        coverageArea    % object Coverage Area
        origin     % origin of the where the x-distance to the line is measured from
    end
    
    methods
        function obj = PredictionBox(axes, xPos, yPos, width, colors, labels, active)
            %PredictionBox Construct an instance with origin at xPos,yPos and
            %distance in x direction
            %   Detailed explanation goes here
            
            % set active
            if nargin < 7
                active = false;
            end            
          
            % set origin
            obj.origin = [xPos yPos];
            % construct all bars
            for i=1:min([length(colors) length(labels)]) % one bar for every color-label pair
                obj.bars = [obj.bars Bar(axes, xPos, yPos, width, colors(i), labels(i), active)];
            end
            % construct coverage area
            obj.coverageArea = CoverageArea(axes, xPos, yPos, ... % start position
                                                  width, 0, ... % dimensions
                                                  'green', 0.15, ... % inner color
                                                  'green', 0.15, ... % edge color
                                                  active);
            
            % set super class properties
            obj.initialize(active);
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

