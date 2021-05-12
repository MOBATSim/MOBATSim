classdef VehicleAnalysingWindow < handle
    %VEHICLEANALYSINGWINDOW Analysing window for vehicles
    %   Detailed explanation goes here TODO: write a detailed explanation
    
    properties
        egoVehicleId    % vehicle the camera should focus on
        vehicles        % all vehicles that should be shown in the analysis
        modelName       % simulation model name
        gui             % contains all parts of the gui
            % fig                   figure of the analysis window
            % grid                  grid to align all components of the gui
            % vehicleSelectionDd    drop-down selection menu for the actual vehicle
            % axes                  used as a container for a plot
            % bep                   birds eye plot
            % outlinePlotter        plots the outlines of the cars
            % lanePlotter           plots the outlines of the roads
        scenario        % contains cars and roads that should be plotted
        emergencyBrakeDistance      % distance to stop ego vehicle TODO: move this to a safety part
        plotEmBrake     % plotted area for emergency brake
    end 
    
    methods
        function obj = VehicleAnalysingWindow(vehicles, egoVehicleId)
            %VEHICLEANALYSINGWINDOW Construct an instance of this class
            %   Detailed explanation goes here
            
            % setup properties
            obj.setup();
            
            % get ego vehicle id
            obj.egoVehicleId = egoVehicleId;
            % get all vehicles
            obj.vehicles = vehicles;

            % generate window 
            obj.gui = generateGui(obj);
            
            % get scenario with road network
            obj.getRoadScenario();
            
            % add vehicles to the driving scenario
            obj.addVehiclesToScenario(obj.vehicles);
            
            % initialize gui
            obj.update();
        end
 
        function setup(obj)
            % Setup this object
            
            % find old analysing window and close it
            close(findall(groot,'Type','figure','Tag','vehicleAnalysingWindow_tag')); % close last window
            
            % setup object properties
            obj.modelName = evalin('base','modelName');
            
        end
        
        function gui = generateGui(obj)
            %GENERATEGUI Generate the vehicle analysing window
            %   using uifigure
           
            % Ui figure              
            gui.fig = uifigure('Name','Vehicle Analysing Window');
            gui.fig.Visible = 'on';
            gui.fig.Tag = 'vehicleAnalysingWindow_tag';
            
            % Grid layout
            gui.grid = uigridlayout(gui.fig);
            gui.grid.RowHeight = {'fit','fit','1x'};
            gui.grid.ColumnWidth = {300,'1x'};
            % generate drop-downs
            gui.vehicleSelectionDd = uidropdown(gui.grid,'Items',{'<b style="color:red;">Out of service!</b>','<em>Vehicle 1</em>','Vehicle 1','Vehicle 2','Vehicle 4'}, ...
                                                         'ValueChangedFcn',@(dd,event) obj.vehicleSelectionCallback(dd));
            % TODO: make the vehicle item generation automated with
            % vehicles(i).name
            %gui.vehicleSelectionDd.Interpreter = 'html';
            % TODO: one dropdown item per vehicle
            %dd1.Items = {'1','2'};
            %generate tabs
            %tabgp = uitabgroup(obj.fig,'Position',[.05 .05 .3 .8]);
            %tab1 = uitab(tabgp,'Title','Vehicle 1');
            %tab2 = uitab(tabgp,'Title','Vehicle 2');
            %tab1 = uitab('Title','Vehicle 1');
            
            % Pause button
            gui.pauseBtn = uibutton(gui.grid, 'state', ...
                                              'ValueChangedFcn', @(btn,event) obj.pauseSimTime(btn));
            gui.pauseBtn.Text = 'Pause';
            gui.pauseBtn.Layout.Row = 1;
            gui.pauseBtn.Layout.Column = 2;
            %gui.pauseBtn.HorizontalAlignment = 'right';
            
            %% Area showing all important variables
            % Panel for ego vehicle
            gui.panelEgoVehicle = uipanel('Parent',gui.grid, 'Title','Ego vehicle',...
                                          'FontSize',14, 'BackgroundColor','white');
            gui.panelEgoVehicle.BorderType = 'none';                          
            gui.panelEgoVehicle.Layout.Row = 3;
            gui.panelEgoVehicle.Layout.Column = 1;
            % Subgrid
            gui.subgridVariables = uigridlayout(gui.panelEgoVehicle);
            gui.subgridVariables.RowHeight = {'fit','fit','fit','fit','1x'};
            gui.subgridVariables.ColumnWidth = {'fit',100,'fit'};
            
            % Add variable entrys to subgrid
            
            gui.entrySimTime = obj.generateVariableEntry(gui, 'Simulation time', 's');
            gui.entryEgoVelocity = obj.generateVariableEntry(gui, 'Ego vehicle velocity', 'm/s');       
                       
            % Emergency Brake Distance
            gui.lblEmBrakeDist = uilabel(gui.subgridVariables, 'Text','Emergency brake distance: ');
            gui.lblEmBrakeDist.Layout.Row = 3;
            gui.lblEmBrakeDist.Layout.Column = 1;
            gui.lblEmBrakeDist.WordWrap = 'on';
            gui.valueEmBrakeDist = uilabel(gui.subgridVariables, 'Text','0');
            gui.valueEmBrakeDist.HorizontalAlignment = 'right';
            gui.cbEmBrakeDist = uicheckbox(gui.subgridVariables, 'Text','');
            
            gui.testField = obj.generateVariableEntry(gui, 'test field', 'mm/s');
            
            % Check box tree
            gui.tree = uitree(gui.subgridVariables);%'checkbox'); TODO: activate in v2021a
            gui.tree.Layout.Row = 5;
            gui.tree.Layout.Column = [1,3];
            
            gui.categoryAreas = uitreenode(gui.tree, 'Text','Shown Areas');
            gui.nodeEmBrake = uitreenode(gui.categoryAreas, 'Text','Emergency Brake Distance');
            expand(gui.tree);
            %% Birds eye plot
            % Axes as plot container
            gui.axes = uiaxes(gui.grid);
            gui.axes.Layout.Row = 3;
            gui.axes.Layout.Column = 2;
            
            % birds eye plot in axes container
            gui.bep = birdsEyePlot('Parent', gui.axes, 'XLim',[-50 100], 'YLim',[-20 20]);
            legend(gui.axes,'off');
            
            
            % Plotters for different aspects
            gui.outlinePlotter = outlinePlotter(gui.bep);
            gui.lanePlotter = laneBoundaryPlotter(gui.bep);%,'DisplayName','Road');
        end
        
        function variableEntry = generateVariableEntry(~, gui, title, unit)
            % generate a subgridVariabeles entry for showing an variable value
            % returns a structure with following content:
                % .lbl      name of the variable
                % .value    value of the variable
                % .unit     unit of the value
            
            % Name field
            variableEntry.lbl = uilabel(gui.subgridVariables, 'Text',title + ": ");
            variableEntry.lbl.Layout.Column = 1;
            variableEntry.lbl.WordWrap = 'on';
            
            % Value field
            variableEntry.value =  uilabel(gui.subgridVariables, 'Text','-');
            variableEntry.value.HorizontalAlignment = 'right';
            
            % Unit field
            variableEntry.unit = uilabel(gui.subgridVariables, 'Text',unit);
        end
        
        function getRoadScenario(obj)
            % get a scenario with the road network
            
            obj.scenario = scenario_map_v1();
        end
        
        
        function addVehiclesToScenario(obj, vehicles)
            % add vehicles for plotting with properties of the
            % simulated vehicles to the scenario
            
            for i = 1 : length(vehicles)
                vehicle(obj.scenario, ...
                    'ClassID', 1, ... % group 1 means cars
                    'Name', vehicles(i).id, ...
                    'Length', vehicles(i).physics.size(3), ...
                    'Width', vehicles(i).physics.size(2), ...
                    'Height', vehicles(i).physics.size(1), ...
                    'RearOverhang', vehicles(i).physics.size(3)/2); % This moves the origin to the middle of the car
            end
        end
        
        %% Update functions
        function update(obj)
            % update gui
            
            % update all shown values
            obj.updateValueArea();
            
            % update birds eye view
            obj.updatePlot();
        end
           
        function updatePlot(obj)
            % update all plotted objects
            
            
            % get vehicles poses
            i = 1:length(obj.vehicles);
            positions = cat(1,cat(2,obj.vehicles(i).dynamics).position);
            orientations = cat(1,cat(2,obj.vehicles(i).dynamics).orientation);
            % coordinate transformation for plot
            x = -positions(:,3); % x = negative y-Position of vehicle
            y = -positions(:,1); % y = negative x-Position of vehicle
            yaw = orientations(:,4) + pi; % rotate by 180 degrees
            % update vehicle position and orientation
            obj.updateVehiclePose(x, y, yaw);
            
            % redraw vehicles
            [position,yaw,Length,width,originOffset,color] = targetOutlines(obj.scenario.Actors(obj.egoVehicleId));
            plotOutline(obj.gui.outlinePlotter, position, yaw, Length, width, ...
                'OriginOffset',originOffset, 'Color',color);
            
            % redraw vehicles from ego vehicle pose
            rb = roadBoundaries(obj.scenario.Actors(obj.egoVehicleId));
            plotLaneBoundary(obj.gui.lanePlotter,rb);
            
            % plot coverage areas
            % plot emergency brake area
            % width - size of vehicle
            % length - emergency brake distance
            vehicleWidth = obj.vehicles(obj.egoVehicleId).physics.size(2); 
            vehicleLength = obj.vehicles(obj.egoVehicleId).physics.size(3);
            
            p1 = [vehicleLength/2 -vehicleWidth/2];
            p2 = [vehicleLength/2 vehicleWidth/2];
            p3 = [vehicleLength/2+obj.emergencyBrakeDistance vehicleWidth/2];
            p4 = [vehicleLength/2+obj.emergencyBrakeDistance -vehicleWidth/2];
            if (obj.gui.cbEmBrakeDist.Value == 1) && (p1(1) ~= p4(1)) 
                polyg = polyshape([p1(1) p2(1) p3(1) p4(1)], [p1(2) p2(2) p3(2) p4(2)]);
                obj.plotEmBrake = plot(obj.gui.axes, polyg, 'FaceColor','cyan', 'EdgeColor','blue', 'FaceAlpha',0.5);
            else
                delete(obj.plotEmBrake);
            end
        end
        
        
        function updateVehiclePose(obj, x, y, yaw)
            % update positions from all vehicles
            % yaw is in rad
            
            for i = 1 : length(obj.vehicles)
                % set position of actor
                obj.scenario.Actors(i).Position = [x(i) y(i) 0];
                % set rotation of actor
                obj.scenario.Actors(i).Yaw = yaw(i)/pi*180;
            end
        end
        
        function updateValueArea(obj)
            % Update all values in gui with simulation values
            
            % Update variable entrys
           obj.updateVariableEntry(obj.gui.entrySimTime, obj.getCurrentSimTime());
           obj.updateVariableEntry(obj.gui.entryEgoVelocity, obj.vehicles(obj.egoVehicleId).dynamics.speed);
           %obj.gui.valueEgoVelocity.Text = obj.vehicles(obj.egoVehicleId).dynamics.speed + " m/s";
           % emergency brake distance
           actSpeed = obj.vehicles(obj.egoVehicleId).dynamics.speed;
           minAccleration = obj.vehicles(obj.egoVehicleId).dynamics.minDeceleration;
           obj.emergencyBrakeDistance = obj.calculateEmergencyBrakeDistance(actSpeed, minAccleration);
           obj.gui.valueEmBrakeDist.Text = obj.emergencyBrakeDistance + " m";
        end
        
        function updateVariableEntry(~, variableEntry, value)
            % update the value of a variable entry
            % variableEntry must have following structure
                % .lbl      name of the variable
                % .value    value of the variable
                % .unit     unit of the value
            
            variableEntry.value.Text = string(value); % write the value to the matching label
        end
 
        %%
        function currentSimTime = getCurrentSimTime(obj)
            % Get current simulation time
            
            currentSimTime = get_param(obj.modelName,'SimulationTime');
        end
        
        function pauseSimTime(~, btn)
            % pause/resume simulation execution
            
            if btn.Value == true
                uiwait % pause simulation
            else
                uiresume % unpause simulation
            end
        end
        
        
        function plottedPoly = plotPolygon(obj, polyshape, color, edgeColor)
            % draw a polygon into the birds eye plot
            
            plottedPoly = plot(obj.gui.axes, polyshape, 'FaceColor',color, 'EdgeColor',edgeColor);
        end
        
        function emergencyBrakeDistance = calculateEmergencyBrakeDistance(~, actSpeed, minAcceleration)
            % Calculate the distance a vehicle needs to stop TODO: move
            % this to a safety component, should not be in UI
            
            emergencyBrakeDistance = 0.5*-actSpeed^2/minAcceleration;
        end
        
        
        
        function changeEgoVehicle(obj, egoVehicleId)
            % Change ego vehicle at GUI
            
            obj.egoVehicleId = egoVehicleId;
            % Update GUI with new ego vehicle
            obj.update();
        end
        
        %% Callbacks
        function vehicleSelectionCallback(obj, dropdown)
            % Callback of the vehicle selection dropdown menu
            % TODO: make this more generic when entry generation is
            % automated
            switch dropdown.Value
                case 'Vehicle 1'
                    newEgoVehicleId = 1;
                case 'Vehicle 2'
                    newEgoVehicleId = 2;
                case 'Vehicle 4'
                    newEgoVehicleId = 4;
                otherwise
                    newEgoVehicleId = 9;
            end
            % Change the ego vehicle at GUI
            obj.changeEgoVehicle(newEgoVehicleId);
        end
        
    end
end

