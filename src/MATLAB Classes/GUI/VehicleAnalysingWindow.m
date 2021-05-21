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
        timeToPause     % simulation should pause when simulation time is at this value
        currentSimTime  % current simulation time
        
        frontCar        % all information regarding the car infront of the ego vehicle
            % relative distance
            % relative speed
    end 
    
    methods
        function obj = VehicleAnalysingWindow(vehicles, egoVehicleId)
            %VEHICLEANALYSINGWINDOW Construct an instance of this class
            %   Detailed explanation goes here
            
            % setup properties
            obj.setup(vehicles, egoVehicleId);
                        
            % setup the driving scenario
            obj.scenario = obj.setupScenario(scenario_map_v1(), obj.vehicles);
            
            % generate GUI 
            obj.gui = obj.generateGui(obj.vehicles);
                        
            % initialize GUI
            obj.changeViewToEgoVehicle();  
            obj.gui.fig.Visible = 'on'; % show window
        end
 
        function setup(obj, vehicles, egoVehicleId)
            % Setup this object
            
            % find old analysing window and close it
            close(findall(groot,'Type','figure','Tag','vehicleAnalysingWindow_tag')); % close last window
            
            % setup object properties
            obj.modelName = evalin('base','modelName');
            obj.setTimeToPause(0);
            obj.vehicles = vehicles;
            obj.setEgoVehicleId(egoVehicleId);
        end
        
        function gui = generateGui(obj, vehicles)
            %GENERATEGUI Generate the vehicle analysing window
            %   using uifigure
           
            % Ui figure              
            gui.fig = uifigure('Name','Vehicle Analysing Window');
            gui.fig.Visible = 'off'; % don't show window during generation
            gui.fig.Tag = 'vehicleAnalysingWindow_tag';
            
            % Grid layout
            gui.grid = uigridlayout(gui.fig);
            gui.grid.RowHeight = {'fit','fit',150,'1x'};
            gui.grid.ColumnWidth = {300,'1x'};
            % Vehicle selection drop down
            i = 1:length(obj.vehicles);
            gui.vehicleSelectionDd = uidropdown(gui.grid, 'ValueChangedFcn',@(dd,event) obj.vehicleSelectionCallback(dd),...
                                                          'Items', {vehicles(i).name}, ...
                                                          'ItemsData', [vehicles(i).id]);                                       

            %generate tabs TODO: use tabs
            %tabgp = uitabgroup(obj.fig,'Position',[.05 .05 .3 .8]);
            %tab1 = uitab(tabgp,'Title','Vehicle 1');
            %tab2 = uitab(tabgp,'Title','Vehicle 2');
            %tab1 = uitab('Title','Vehicle 1');
            % TODO: use the html interpreter for coloring words: <b style="color:red;">Out of service!</b>','<em>Vehicle 1</em>
            % Pause button
            gui.pauseBtn = uibutton(gui.grid, 'state', ...
                                              'ValueChangedFcn', @(btn,event) obj.pauseSimTimeCallback(btn), ...
                                              'Text', 'Pause', ...
                                              'BackgroundColor', [0.9290 0.6940 0.1250]); % kind of orange, yellow            
            %% Simulation time area           
            % Subgrid simulation time
            gui.subgridSimTime = uigridlayout(gui.grid);
            gui.subgridSimTime.Layout.Row = 2;
            gui.subgridSimTime.Layout.Column = 1;
            gui.subgridSimTime.RowHeight = {'fit'};
            gui.subgridSimTime.ColumnWidth = {'fit','fit','fit','1x',110};
            % Variable entry for simulation time
            gui.entrySimTime = obj.generateVariableEntry(gui.subgridSimTime, 'Simulation time', 's', false);
            % Pause at a specified time as edit field
            gui.fieldSimTime = uieditfield(gui.subgridSimTime,'numeric',...
                                                              'ValueDisplayFormat', 'Pause at t = %.2f s',...
                                                              'ValueChangedFcn',@(txt,event) obj.setTimeToPause(txt.Value));
            gui.fieldSimTime.Limits = [0 inf];
            gui.fieldSimTime.Layout.Column = 5;
                    
            %% Area showing all important variables of ego vehicle
            % Panel for ego vehicle
            gui.panelEgoVehicle = uipanel('Parent',gui.grid, 'Title','Ego vehicle',...
                                          'FontSize',14, 'BackgroundColor','white');
            gui.panelEgoVehicle.BorderType = 'none';                          
            gui.panelEgoVehicle.Layout.Row = 3;
            gui.panelEgoVehicle.Layout.Column = 1;
            % Subgrid
            gui.subgridVariables = uigridlayout(gui.panelEgoVehicle);
            gui.subgridVariables.RowHeight = {'fit','fit','fit','fit','fit','fit','fit','fit','fit',20};
            gui.subgridVariables.ColumnWidth = {'fit','1x','fit','fit'};
            
            % Variable entrys to subgrid
            gui.entryVelocity = obj.generateVariableEntry(gui.subgridVariables, 'Velocity', 'm/s', false);
            gui.entryDeltaDistance = obj.generateVariableEntry(gui.subgridVariables, 'Delta distance', 'm', true);
            gui.entryDeltaSpeed = obj.generateVariableEntry(gui.subgridVariables, 'Delta velocity', 'm/s', true);
            gui.entryEmBrakeDistance = obj.generateVariableEntry(gui.subgridVariables, 'Emergency brake distance', 'm', true);
            gui.entryDrivingMode = obj.generateVariableEntry(gui.subgridVariables, 'Driving mode', '', false);
            % Check box tree
            gui.tree = uitree(gui.subgridVariables);%'checkbox'); TODO: activate in v2021a
            gui.tree.Layout.Row = 10;
            gui.tree.Layout.Column = [1,3];
            gui.tree.Visible = false;
            
            gui.categoryAreas = uitreenode(gui.tree, 'Text','Shown Areas');
            gui.nodeEmBrake = uitreenode(gui.categoryAreas, 'Text','Emergency Brake Distance');
            %expand(gui.tree);
            %% Safe space from selected driving mode
            % TODO: maybe use tabs for more graphs.
            gui.axesSafeSpace = uiaxes(gui.grid);
            gui.axesSafeSpace.Layout.Row = 4;
            gui.axesSafeSpace.Layout.Column = 1;
            gui.axesSafeSpace.XLim = [0 20];
            gui.axesSafeSpace.YLim = [0 20];
            title(gui.axesSafeSpace, 'Safe space');
            xlabel(gui.axesSafeSpace, 'relative distance in m');
            ylabel(gui.axesSafeSpace, 'relative velocity in m/s');
            % set axes background color to red
            set(gui.axesSafeSpace, 'color', '#CC0033'); % kind of red
            
            % Create plot
            % Create colored areas showing safe sets
            minDeceleration = -9.15;
            midDeceleration = -6.15;
            minSafeDistance = 1;
            midSafeDistance = 2;
            gui.areaSafeSet = obj.generateAreaVelocity(gui.axesSafeSpace, minDeceleration, minSafeDistance, '#33FF33'); % bright green
            gui.areaSafeTerminalSet = obj.generateAreaVelocity(gui.axesSafeSpace, midDeceleration, midSafeDistance, '#009900'); % dark green
           
            % Point showing actual set of vehicle
            % TODO: make a function for that
            hold(gui.axesSafeSpace, 'on');
            gui.pointActualSet = plot(gui.axesSafeSpace, 10, 10,'b--o');
            hold(gui.axesSafeSpace, 'on');
            gui.arrow = quiver(gui.axesSafeSpace, 10, 10, -1, 1, 1.5);         
            
            %% Birds eye plot
            % Axes as plot container
            gui.axesBep = uiaxes(gui.grid);
            gui.axesBep.Layout.Row = [3,4];
            gui.axesBep.Layout.Column = 2;
            
            % Birds eye plot in axes container
            gui.bep = birdsEyePlot('Parent', gui.axesBep, 'XLim',[-50 100], 'YLim',[-20 20]);
            legend(gui.axesBep,'off');
            
            % Different elements to plot
            gui.rulerDistance = Ruler(gui.axesBep, 0, -3, 3, 'Distance');
            
            % Plotters for different aspects
            gui.outlinePlotter = outlinePlotter(gui.bep);
            gui.lanePlotter = laneBoundaryPlotter(gui.bep);
            
        end
        
        function variableEntry = generateVariableEntry(obj, parent, title, unit, checkboxNeeded)
            % generate a subgridVariabeles entry for showing an variable value
            % returns a structure with following content:
                % .lbl      name of the variable
                % .value    value of the variable
                % .unit     unit of the value
            
            % Name field
            variableEntry.lbl = uilabel(parent, 'Text',title + ": ");
            variableEntry.lbl.Layout.Column = 1;
            variableEntry.lbl.WordWrap = 'on';
            
            % Value field
            variableEntry.value =  uilabel(parent, 'Text','-');
            variableEntry.value.HorizontalAlignment = 'right';
            
            % Unit field
            variableEntry.unit = uilabel(parent, 'Text',unit);
            variableEntry.unit.HorizontalAlignment = 'left';
            
            % Check box - mandatory
            variableEntry.cb = uicheckbox(parent, 'Text','');
            if checkboxNeeded
                % redraw plot to change visibility when checkbox value is
                % changed
                variableEntry.cb.ValueChangedFcn = @(cbx,event) obj.updatePlot();
            else
                % disable checkbox
                variableEntry.cb.Visible = false; 
            end
        end
        
        function areaVelocity = generateAreaVelocity(~, axes, minDeceleration, safeDistance, color)
            % generate an area showing the velocities that allow
            % deceleration with minDeceleration
            
            % Function describing maximal velocity at minDeceleration
            distance = linspace(0,20);
            velocity = sqrt(2*(distance-safeDistance)*-minDeceleration);
            
            % Create colored veloctiy area
            hold(axes, 'on');
            areaVelocity = area(axes, distance, ...
                                      velocity, ...
                                      'FaceColor', color, ...
                                      'EdgeAlpha', 0.2);
            
        end
        
        function roadScenario = setupScenario(~, roadScenario, vehicles)
            % Setup the scenario with a road network
            
            % Add vehicles with properties of the simulated vehicles to scenario
            for i = 1 : length(vehicles)
                vehicle(roadScenario, ...
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

            %% Get information
            % get new simulation time
            obj.getCurrentSimTime();
            
            % get front car information
            [obj.frontCar.relativeDistance, obj.frontCar.relativeSpeed] = obj.check_leadingVehicle(obj.vehicles, obj.vehicles(obj.egoVehicleId));
            
            %% Set information
            % update all shown values
            obj.updateValueArea();
            
            % check if simulation should pause
            obj.checkTimeToPause();

            % update birds eye view
            obj.updatePlot();
            
        end
           
        function updatePlot(obj)
            % update all plotted objects
            
            %% update vehicles
            % get vehicles poses
            i = 1:length(obj.vehicles);
            positions = cat(1,cat(2,obj.vehicles(i).dynamics).position);
            orientations = cat(1,cat(2,obj.vehicles(i).dynamics).orientation);
            % update vehicle position and orientation
            obj.updateVehiclePose(positions(:,1), positions(:,3), orientations(:,4));
            % redraw vehicles
            [position,yaw,Length,width,originOffset,color] = targetOutlines(obj.scenario.Actors(obj.egoVehicleId));
            plotOutline(obj.gui.outlinePlotter, position, yaw, Length, width, ...
                'OriginOffset',originOffset, 'Color',color);
            %% update roads
            % redraw roads from ego vehicle pose
            rb = roadBoundaries(obj.scenario.Actors(obj.egoVehicleId)); % Maybe use laneBoundaries and only show a part of the map
            plotLaneBoundary(obj.gui.lanePlotter,rb);

            %% 
            % update length of distance ruler
            obj.gui.rulerDistance.updateLength(obj.frontCar.relativeDistance);
            
            
            %% update coverage areas
            % plot coverage areas
            % plot emergency brake area
            % TODO: make this better and nicer
            if obj.gui.entryEmBrakeDistance.cb.Value == 1 % use a callback for activation, so area is also shown, when simulation is paused
                % width - size of vehicle
                % length - emergency brake distance
                vehicleWidth = obj.vehicles(obj.egoVehicleId).physics.size(2);
                vehicleLength = obj.vehicles(obj.egoVehicleId).physics.size(3);
                
                p1 = [vehicleLength/2 -vehicleWidth/2];
                p2 = [vehicleLength/2 vehicleWidth/2];
                p3 = [vehicleLength/2+obj.emergencyBrakeDistance vehicleWidth/2];
                p4 = [vehicleLength/2+obj.emergencyBrakeDistance -vehicleWidth/2];
                
                if p1(1) ~= p4(1) % show only if area has a length
                    
                    polyg = polyshape([p1(1) p2(1) p3(1) p4(1)], [p1(2) p2(2) p3(2) p4(2)]);
                    obj.plotEmBrake = plot(obj.gui.axesBep, polyg, 'FaceColor','cyan', 'EdgeColor','blue', 'FaceAlpha',0.5);
                end
            else
                delete(obj.plotEmBrake);
            end
        end      
        
        function updateVehiclePose(obj, x, y, yaw)
            % update positions from all vehicles
            % yaw is in rad
            
            for i = 1 : length(obj.vehicles)
                % set position of actor
                % change positions from mobatsim coordinate system
                obj.scenario.Actors(i).Position = [-y(i) -x(i) 0];
                % set rotation of actor
                % rotate from mobatsim coordinate system
                yaw(i) = yaw(i) - pi/2;
                % limit angle from -pi to pi
                if yaw(i) > pi
                    yaw(i) = yaw(i) - 2*pi;
                elseif  yaw(i) < pi
                    yaw(i) = yaw(i) + 2*pi;
                end
                obj.scenario.Actors(i).Yaw = yaw(i)/pi*180;
            end
        end
        
        function updateValueArea(obj)
            % Update all values in gui with simulation values
            
            % calculate current emergency brake distance
            obj.emergencyBrakeDistance = obj.calculateEmergencyBrakeDistance();
            
            % Update variable entrys
            obj.updateVariableEntry(obj.gui.entrySimTime, obj.currentSimTime);
            obj.updateVariableEntry(obj.gui.entryVelocity, obj.vehicles(obj.egoVehicleId).dynamics.speed);
            obj.updateVariableEntry(obj.gui.entryDeltaDistance, obj.frontCar.relativeDistance);
            obj.updateVariableEntry(obj.gui.entryDeltaSpeed, obj.frontCar.relativeSpeed);
            obj.updateVariableEntry(obj.gui.entryEmBrakeDistance, obj.emergencyBrakeDistance);
            obj.updateVariableEntry(obj.gui.entryDrivingMode, obj.vehicles(obj.egoVehicleId).status.drivingMode);
        end
        
        function updateVariableEntry(~, variableEntry, value)
            % update the value of a variable entry
            % variableEntry must have following structure
                % .lbl      name of the variable
                % .value    value of the variable
                % .unit     unit of the value
            
            variableEntry.value.Text = string(value); % write the value to the matching label
        end
        
        function changeViewToEgoVehicle(obj)
            % change GUI view to the actual ego vehicle
            
            % Update vehicle selction dropdown
            obj.gui.vehicleSelectionDd.Value = obj.egoVehicleId;
            % Update GUI with new ego vehicle
            obj.update();
        end
 
        %%
        
        function checkTimeToPause(obj)
            % check and pause simulation
            
           if (obj.timeToPause > 0) && (obj.currentSimTime == obj.timeToPause)
               obj.gui.pauseBtn.Value = 1; % set pause button to 'paused'
               uiwait; % pause
           end    
        end
        
        function plottedPoly = plotPolygon(obj, polyshape, color, edgeColor)
            % draw a polygon into the birds eye plot
            
            plottedPoly = plot(obj.gui.axesBep, polyshape, 'FaceColor',color, 'EdgeColor',edgeColor);
        end
        
        function emergencyBrakeDistance = calculateEmergencyBrakeDistance(obj)
            % Calculate the distance a vehicle needs to stop with min deceleration
            %TODO: move this to a safety component, should not be in UI
            
            % get current dynamic parameters
            curSpeed = obj.vehicles(obj.egoVehicleId).dynamics.speed;
            minAcceleration = obj.vehicles(obj.egoVehicleId).dynamics.minDeceleration;
            % calculate distance
            emergencyBrakeDistance = 0.5*-curSpeed^2/minAcceleration;
        end
        
        function getCurrentSimTime(obj)
            % Get current simulation time
            
            obj.currentSimTime = get_param(obj.modelName,'SimulationTime');
        end
        

        %% Callbacks
        function vehicleSelectionCallback(obj, dropdown)
            % Callback of the vehicle selection dropdown menu
            
            % Change the ego vehicle according to the selected name
            obj.setEgoVehicleId(dropdown.Value);
            obj.update();        
        end
        
        function pauseSimTimeCallback(~, btn)
            % pause/resume simulation execution
            
            if btn.Value == true
                uiwait % pause simulation
            else
                uiresume % unpause simulation
            end
        end
        
        %% Setter/Getter
        
        function setTimeToPause(obj, time)
            % set the time to pause simulation
            
            obj.timeToPause = time;
        end
                
        function setEgoVehicleId(obj, egoVehicleId)
            % Set the ego vehicle id
            
            obj.egoVehicleId = egoVehicleId;
        end

        %% Copied functions from other code parts
        
        %VehicleSensors
        
        function [relativeDistance, relativeSpeed, ttc, leadingVehicle] = check_leadingVehicle(~, Vehicles, vehicle)
            %this function checks if there's a leading vehicle ahead, if
            %multiple leading vehicles exist, set nearest vehicle to be the
            %leading vehicle
            leadingVehicle = [];
            relativeDistance = inf;
            relativeSpeed = inf;
            ego_route = vehicle.pathInfo.currentRoute;% Search vehicle on this route
            
            
            %%
            %traverse leading Vehicle on current route
            for vehicle_ = Vehicles
                if vehicle_.id == vehicle.id
                    break;
                end
                if isequal(vehicle_.pathInfo.currentRoute,ego_route)&&(vehicle_.pathInfo.s>vehicle.pathInfo.s)%If this vehicle is on the same route and ahead of the ego vehicle
                    if isempty(leadingVehicle)%If no leading vehicle exists, set this vehicle to be the leading vehicle
                        leadingVehicle = vehicle_;
                    elseif vehicle_.pathInfo.s<leadingVehicle.pathInfo.s %if there's already a leading vehicle, find the closest one
                        leadingVehicle = vehicle_;
                    end
                end
            end
            
            
            if ~isempty(leadingVehicle)
                relativeSpeed = vehicle.dynamics.speed-leadingVehicle.dynamics.speed;
                relativeDistance = leadingVehicle.pathInfo.s-vehicle.pathInfo.s;
                ttc = relativeDistance/relativeSpeed;
            else
                %continue search next route
                idx = find(vehicle.pathInfo.path==vehicle.pathInfo.lastWaypoint);
                if idx+2<=length(vehicle.pathInfo.path) % Next Route
                    nextRoute = vehicle.map.getRouteIDfromPath([vehicle.pathInfo.path(idx+1) vehicle.pathInfo.path(idx+2)]);
                else % Destination Reached // CurrentRoute stays the same
                    nextRoute = vehicle.pathInfo.currentRoute;
                end
                for vehicle_ = Vehicles
                    if nextRoute == vehicle.pathInfo.currentRoute || vehicle_.id == vehicle.id
                        break;
                    end
                    
                    if isequal(vehicle_.pathInfo.currentRoute,nextRoute)
                        if isempty(leadingVehicle)
                            leadingVehicle = vehicle_;
                        elseif vehicle_.pathInfo.s<leadingVehicle.pathInfo.s
                            leadingVehicle = vehicle_;
                        end
                    end
                end
                if ~isempty(leadingVehicle)
                    relativeSpeed = vehicle.dynamics.speed-leadingVehicle.dynamics.speed;
                    relativeDistance = leadingVehicle.pathInfo.s+vehicle.pathInfo.routeEndDistance;
                    ttc = relativeDistance/relativeSpeed;
                else
                    ttc = 1000;
                    leadingVehicle = [];
                end
            end
            
        end
    end
end

