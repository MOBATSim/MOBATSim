classdef VehicleAnalysingWindow < handle
    %VEHICLEANALYSINGWINDOW Analysing window for vehicles
    %   Detailed explanation goes here TODO: write a detailed explanation
    
    properties
        egoVehicle  % vehicle the camera should focus on
        vehicles    % all other vehicles that should be shown in the analysis
        gui         % contains all parts of the gui
            % fig                   figure of the analysis window
            % grid                  grid to align all components of the gui
            % vehicleSelectionDd    drop-down selection menu for the actual vehicle
            % axes                  axes to plot the vehicle data into
            % plots                 contains all plotted data
                % egoVehicle        the vehicle to show in ego perspective
                % vehicles          all other vehicles
      
    end
    
    methods
        function obj = VehicleAnalysingWindow(vehicles, egoVehicleId)
            %VEHICLEANALYSINGWINDOW Construct an instance of this class
            %   Detailed explanation goes here
            
            % get ego vehicle
            i = 1:length(vehicles);
            obj.egoVehicle = vehicles([vehicles(i).id] == egoVehicleId);
            % get all vehicles
            obj.vehicles = vehicles;
            

            % generate window 
            obj.gui = generateGui(obj);
         
            
            % plot vehicle data
            obj.plotVehicles(obj.vehicles);
        end
        
        function gui = generateGui(~)
            %GENERATEGUI Generate the vehicle analysing window
            %   using uifigure
           
            % find old analysing window and close it
            close(findall(groot,'Type','figure','Tag','vehicleAnalysingWindow_tag')); % close last window
            % generate ui figure              
            gui.fig = uifigure('Name','Vehicle Analysing Window');
            % tune ui figure properties
            gui.fig.Visible = 'on';
            gui.fig.Tag = 'vehicleAnalysingWindow_tag';
            
            % generate grid layout
            gui.grid = uigridlayout(gui.fig);
            gui.grid.RowHeight = {22,22,'1x'};
            gui.grid.ColumnWidth = {150,'1x'};
            % generate drop-downs
            gui.vehicleSelectionDd = uidropdown(gui.grid,'Items',{'Vehicle 1','Vehicle 2'});
            % TODO: one dropdown item per vehicle
            % TODO: get nextWaypointBlocked! visible on screen
            %dd1.Items = {'1','2'};
            %generate tabs
            %tabgp = uitabgroup(obj.fig,'Position',[.05 .05 .3 .8]);
            %tab1 = uitab(tabgp,'Title','Vehicle 1');
            %tab2 = uitab(tabgp,'Title','Vehicle 2');
            %tab1 = uitab('Title','Vehicle 1');
            % generate vehicle plot placeholder by using axes
            gui.axes = uiaxes(gui.grid);
            gui.axes.Layout.Row = 3;
            gui.axes.Layout.Column = 2;
            % set the unit length to the same length
            daspect(gui.axes,[1 1 1]);
        end
        
        function plotVehicles(obj, vehicles)
            % initalize all needed vehicles
            
            % find analysing window plot
            axes = obj.gui.axes; % axes to plot into
            hold(axes,'on')
            
            % plot all vehicles
            for i = 1 : length(vehicles)          
                % get lower left edge from vehicle
                [leftPosition, lowerPosition] = obj.getVehicleEdge(vehicles(i));
                
                if vehicles(i).id == obj.egoVehicle.id
                    % plot ego vehicle
                    obj.gui.plots.egoVehicle = rectangle(axes, 'Position', [leftPosition lowerPosition vehicles(i).physics.size(2) vehicles(i).physics.size(3)], ...
                        'FaceColor', 'cyan', 'EdgeColor', '#0072BD'); % colors for infill and edge
                else
                    % plot other vehicle
                    obj.gui.plots.vehicles(i) = rectangle(axes, 'Position', [leftPosition lowerPosition vehicles(i).physics.size(2) vehicles(i).physics.size(3)], ...
                        'FaceColor', '#EDB120', 'EdgeColor', '#D95319'); % colors for infill and edge
                end  
            end

            % focus plot on ego vehicle
            obj.focusOnVehicle(obj.egoVehicle)
        end
        
        function update(obj)
            % get new position from every vehicle and update plot
            obj.updateVehiclePositions(obj.vehicles);
            % update ego vehicle focus
            obj.focusOnVehicle(obj.egoVehicle);
        end
        
        function updateVehiclePositions(obj, vehicles)
            % update positions from all vehicles
            
            for i = 1 : length(vehicles)
                % get lower left edge from vehicle
                [leftPosition, lowerPosition] = obj.getVehicleEdge(vehicles(i));
                
                if vehicles(i).id == obj.egoVehicle.id
                    % update plot position of ego vehicle
                    obj.gui.plots.egoVehicle.Position(1) = leftPosition;
                    obj.gui.plots.egoVehicle.Position(2) = lowerPosition;
                else
                    % update plot positions of all other vehicles
                    obj.gui.plots.vehicles(i).Position(1) = leftPosition;
                    obj.gui.plots.vehicles(i).Position(2) = lowerPosition;
                end
            end
        end
        
        function focusOnVehicle(obj, vehicle)
            % focus plot on vehicle
            obj.gui.axes.XLim = [vehicle.dynamics.position(1)-10 vehicle.dynamics.position(1)+10];
            obj.gui.axes.YLim = [vehicle.dynamics.position(3)-5 vehicle.dynamics.position(3)+30];
        end
        
        
        function [left, lower] = getVehicleEdge(~, vehicle)
            % get vehicle lower left edge for plotting rectangles
            % TODO: add rotation but this needs polygons
        
            left = vehicle.dynamics.position(1) - vehicle.physics.size(2)/2;  % x-Position - width/2
            lower = vehicle.dynamics.position(3) - vehicle.physics.size(3)/2; % y-Position - length/2
        end
        
    end
end

