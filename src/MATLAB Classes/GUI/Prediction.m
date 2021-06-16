classdef VehicleAnalysingWindow < handle
    %VEHICLEANALYSINGWINDOW Analysing window for vehicles
    %   Detailed explanation goes here TODO: write a detailed explanation
    
    properties
        vehicles        % all vehicles that should be shown in the analysis
        egoVehicle      % vehicle the camera focus on
        leadingVehicle  % vehicle in front of ego vehicle if there is one
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
        timeToPause     % simulation should pause when simulation time is at this value
        currentSimTime  % current simulation time
        relativeDistance            % distance to the leadingVehicle
        relativeSpeed   % speed difference to leadingVehicle
        safeSpace       % contains variables needed for safe space calculation
            % ts            % time step width in sec
            % nrSteps       % number of steps to predict in the future
        ts              % time between two discrete calculation steps
        k               % number of steps to predict in the future
    end 
    
    methods
        function obj = VehicleAnalysingWindow(vehicles, egoVehicleId)
            %VEHICLEANALYSINGWINDOW Construct an instance of this class
            %   Detailed explanation goes here
            % TODO: maybe input simulation time, when analysing window is
            % called and simulation does not run
            
            % setup properties
            obj.setup(vehicles, egoVehicleId);                              
            
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
            obj.vehicles = vehicles;
            obj.setEgoVehicle(egoVehicleId);
            obj.leadingVehicle = [];   
            obj.scenario = obj.setupScenario(scenario_map_v1(), obj.vehicles); % setup the driving scenario
            obj.emergencyBrakeDistance = obj.calculateEmergencyBrakeDistance();
            obj.setTimeToPause(0);
            obj.setCurrentSimTime(0);
            obj.safeSpace.ts = 0.1; % TODO: find good default value
            obj.safeSpace.nrSteps = 10; % TODO: find good default value
            obj.ts = 0.1;
            obj.k = 10;
        end
        
        function gui = generateGui(obj, vehicles)
            %GENERATEGUI Generate the vehicle analysing window
            %   using uifigure
           
            % Ui figure              
            gui.fig = uifigure('Name','Vehicle Analysing Window', 'WindowStyle','normal');
            gui.fig.Visible = 'off'; % don't show window during generation
            gui.fig.Tag = 'vehicleAnalysingWindow_tag';
            gui.fig.Position = [681 309 630 720];
            
            % Grid layout
            gui.grid = uigridlayout(gui.fig);
            gui.grid.RowHeight = {'fit','fit','fit','fit','1x'};
            gui.grid.ColumnWidth = {300,'1x','2x'};
            
            %% Header 
            % Drop down - vehicle selection
            gui.vehicleSelectionDd = uidropdown(gui.grid, 'ValueChangedFcn',@(dd,event) obj.vehicleSelectionCallback(dd),...
                                                          'Items', {vehicles(1:length(obj.vehicles)).name}, ...
                                                          'ItemsData', [vehicles(1:length(obj.vehicles)).id]);                                       
            
            % Pause button
            gui.pauseBtn = uibutton(gui.grid, 'state', ...
                                              'Text', 'Pause', ...
                                              'ValueChangedFcn', @(btn,event) obj.pauseSimTimeCallback(btn), ...
                                              'BackgroundColor', [0.9290 0.6940 0.1250]); % kind of orange, yellow
            gui.pauseBtn.Layout.Column = [2,3];
            
            if obj.currentSimTime <= 0
                % at start used as start button
                gui.pauseBtn.Text = 'Start Simulation'; 
            end
            
            %generate tabs TODO: use tabs
            %tabgp = uitabgroup(obj.fig,'Position',[.05 .05 .3 .8]);
            %tab1 = uitab(tabgp,'Title','Vehicle 1');
            %tab2 = uitab(tabgp,'Title','Vehicle 2');
            % TODO: use the html interpreter for coloring words: <b style="color:red;">Out of service!</b>','<em>Vehicle 1</em>

            %% Simulation time area
            
            % Grid
            gui.subgridSimTime = uigridlayout(gui.grid, 'RowHeight', {'fit'}, ...
                                                        'ColumnWidth', {'fit','fit','fit','1x',110});
            gui.subgridSimTime.Layout.Row = 2;
            gui.subgridSimTime.Layout.Column = 1;
            
            % Variable entry
            gui.entrySimTime = obj.generateVariableEntry(gui.subgridSimTime, 'Simulation time', 's', false);
            
            % Edit field
            gui.fieldSimTime = uieditfield(gui.subgridSimTime,'numeric', ...
                                                              'ValueDisplayFormat', 'Pause at t = %.2f s', ...
                                                              'Limits', [0 inf], ...
                                                              'ValueChangedFcn',@(txt,event) obj.setTimeToPause(txt.Value));
            gui.fieldSimTime.Layout.Column = 5;
                    
            %% Ego vehicle
            % Panel
            gui.panelEgoVehicle = uipanel(gui.grid, 'Title', 'Ego vehicle', ...
                                                    'FontSize', 14, ...
                                                    'BackgroundColor', 'white', ...
                                                    'BorderType', 'none');                         
            gui.panelEgoVehicle.Layout.Row = 3;
            gui.panelEgoVehicle.Layout.Column = 1;
            
            % Subgrid for different kind of elements like entry or
            % interaction
            gui.gridEgoPanel = uigridlayout(gui.panelEgoVehicle, 'RowHeight', {'fit','fit','fit','fit'});
            % Subgrid for variable entries TODO: rename grid (see
            % interaction)
            gui.subgridEgoVehicle = uigridlayout(gui.gridEgoPanel, 'RowHeight', {'fit','fit','fit','fit','fit','fit','fit','fit','fit',20}, ...
                                                                      'ColumnWidth', {'fit','1x','fit','fit'});
            
            % Variable entrys
            gui.entryVelocity = obj.generateVariableEntry(gui.subgridEgoVehicle, 'Velocity', 'm/s', true);
            gui.entryEmBrakeDistance = obj.generateVariableEntry(gui.subgridEgoVehicle, 'Emergency brake distance', 'm', true);
            gui.entryDrivingMode = obj.generateVariableEntry(gui.subgridEgoVehicle, 'Driving mode', '', false);
            gui.entryPredictionBox = obj.generateVariableEntry(gui.subgridEgoVehicle, 'Prediction box for 10 steps', '', true);
            
            % Subgrid for interaction elements
            gui.subgridEgoInteraction = uigridlayout(gui.gridEgoPanel, 'RowHeight', {'fit','fit','fit','fit','fit','fit','fit','fit','fit',20}, ...
                                                                      'ColumnWidth', {'fit','fit','fit','fit'});
            
            gui.interactionTimeStep = obj.generateInteractionElement(gui.subgridEgoInteraction, obj.ts, 'Ts', 0.7);
                                                                  
                                                                  %TODO: clean this up
            gui.sliderTimeStep = uislider(gui.subgridEgoVehicle, 'Value', obj.ts, ...
                                                                 'ValueChangedFcn',@(sld,event) obj.timeStepSliderCallback(sld));
            gui.sliderTimeStep.Layout.Column = 1;
            gui.sliderTimeStep.Limits = [0 0.7];
            
            
            % Edit fields
            gui.fieldTimeStep = uieditfield(gui.subgridEgoVehicle,'numeric', ...
                                                                'ValueDisplayFormat', '%.2f s', ...
                                                                'Limits', [0 inf], ...
                                                                'LowerLimitInclusive', false, ...
                                                                'Value', obj.ts, ...
                                                                'ValueChangedFcn',@(txt,event) obj.editTimeStepCallback(txt));
            gui.fieldTimeStep.Layout.Column = [2,3];
                                                            
            gui.labelTimeStep = uilabel(gui.subgridEgoVehicle, 'Text', 'Ts');
                                                            
            gui.sliderNrSteps = uislider(gui.subgridEgoVehicle, 'Value', obj.k, ...
                                                                 'ValueChangedFcn',@(sld,event) obj.nrStepsSliderCallback(sld));
            gui.sliderNrSteps.Layout.Column = [1,3];
            gui.sliderNrSteps.Limits = [0 50];
            gui.labelNrSteps = uilabel(gui.subgridEgoVehicle, 'Text', 'k');
            % Check box tree TODO: use this to activate elements shown in
            % bep plot and not the checkboxes behind the entries
            %gui.tree = uitree(gui.subgridEgoVehicle);%'checkbox'); TODO: activate in v2021a
            %gui.tree.Layout.Row = 10;
            %gui.tree.Layout.Column = [1,3];
            %gui.tree.Visible = false;
            
            %gui.categoryAreas = uitreenode(gui.tree, 'Text','Shown Areas');
            %gui.nodeEmBrake = uitreenode(gui.categoryAreas, 'Text','Emergency Brake Distance');
            %expand(gui.tree);
            
            %% Leading vehicle
            
            % Panel
            gui.panelLeadingVehicle = uipanel(gui.grid, 'Title', 'Leading vehicle', ...
                                                        'FontSize', 14, ...
                                                        'BackgroundColor', 'white', ...
                                                        'BorderType', 'none');
            gui.panelLeadingVehicle.Layout.Row = 4;
            gui.panelLeadingVehicle.Layout.Column = 1;
            
            % Grid
            gui.subgridLeadingVehicle = uigridlayout(gui.panelLeadingVehicle, 'RowHeight', {'fit','fit','fit','fit','fit','fit','fit','fit','fit'}, ...
                                                                              'ColumnWidth', {'fit','1x','fit','fit'});
                                                                 
            % Variable entrys
            gui.entryDeltaDistance = obj.generateVariableEntry(gui.subgridLeadingVehicle, 'Delta distance', 'm', true);
            gui.entryDeltaSpeed = obj.generateVariableEntry(gui.subgridLeadingVehicle, 'Delta velocity', 'm/s', false);
            
            % Edit fields
            gui.fieldTs = uieditfield(gui.subgridLeadingVehicle,'numeric', ...
                                                                'ValueDisplayFormat', 'Ts = %.2f s', ...
                                                                'Limits', [0 inf], ...
                                                                'LowerLimitInclusive', false, ...
                                                                'Value', obj.safeSpace.ts, ...
                                                                'ValueChangedFcn',@(txt,event) obj.editTimeStepCallback(txt));
            gui.fieldTs.Layout.Row = 3;
            gui.fieldTs.Layout.Column = 1;
            
            gui.fieldNrSteps = uieditfield(gui.subgridLeadingVehicle,'numeric', ...
                                                                     'ValueDisplayFormat', 'Number of steps = %.f', ...
                                                                     'Limits', [0 inf], ...
                                                                     'LowerLimitInclusive', false, ...
                                                                     'Value', obj.safeSpace.nrSteps, ...
                                                                     'ValueChangedFcn',@(txt,event) obj.editNrStepsCallback(txt));
            gui.fieldNrSteps.Layout.Row = 4;
            gui.fieldNrSteps.Layout.Column = 1;
            
            %% Safe space plot
            
            % Axes
            gui.axesSafeSpace = uiaxes(gui.grid, 'XLim', [0 150], ...
                                                 'YLim', [0 25], ...
                                                 'TitleFontSizeMultiplier', 1.4, ...
                                                 'color', '#CC0033'); % kind of dark red
            gui.axesSafeSpace.Title.String = 'Safe space';
            gui.axesSafeSpace.XLabel.String = '\bf \DeltaDistance (m)';
            gui.axesSafeSpace.YLabel.String = '\bf \DeltaSpeed (m/s)';
            gui.axesSafeSpace.Layout.Row = 5;
            gui.axesSafeSpace.Layout.Column = [1,2];
            gui.axesSafeSpace.Interactions = regionZoomInteraction;%rulerPanInteraction ];%zoomInteraction]; % TODO: check if this is the best way
            
            % Areas TODO: get the decelerations and safe distance for area from somewhere, now only placeholder
            gui.areaSafeSet = obj.generateAreaVelocity(gui.axesSafeSpace, -9.15, 1, '#33FF33'); % bright green
            gui.areaSafeTerminalSet = obj.generateAreaVelocity(gui.axesSafeSpace, -6.15, 2, '#009900'); % dark green
           
            % Actual set
            hold(gui.axesSafeSpace, 'on');
            gui.pointActualSet = plot(gui.axesSafeSpace, inf, inf, 'b--o');
            
            % Next sets
            hold(gui.axesSafeSpace, 'on');
            gui.lineNextSets = plot(gui.axesSafeSpace, inf, inf);   
            
            %% Birds eye plot
            
            % Axes
            gui.axesBep = uiaxes(gui.grid);
            gui.axesBep.Layout.Row = [3,5];
            gui.axesBep.Layout.Column = 3;
            
            % Plot
            gui.bep = birdsEyePlot('Parent', gui.axesBep, 'XLim',[-50 100], ...
                                                          'YLim',[-20 20]);
            legend(gui.axesBep,'off');
            
            % Plotters
            gui.outlinePlotter = outlinePlotter(gui.bep); % plots oultline of cars
            gui.lanePlotter = laneBoundaryPlotter(gui.bep); % plots road lanes
                        
            % Arrows
            gui.vecEgoVelocity = Arrow(gui.axesBep, obj.egoVehicle.physics.size(3)/2, 0, ... % start position
                                                    'black'); % color
            
            % Ruler
            gui.rulerDistance = Ruler(gui.axesBep, obj.egoVehicle.physics.size(3)/2, -3, 3, '\Deltad =', false);
            
            % Bars
            gui.barEmBrake = Bar(gui.axesBep, obj.egoVehicle.physics.size(3)/2, 0, 9, 'red', '\bf   Stop!');
            
            % Prediction Box
            colors = ["blue" "green" "green" "blue"];
            labels = ["a=2" "a=0" "a=-1" "a=-3"];
            gui.predictionBox = PredictionBox(gui.axesBep, obj.egoVehicle.physics.size(3)/2, 0, 7, colors, labels, true);
            gui.entryPredictionBox.cb.Value = true; % TODO: remove after testing
            
            %% Special callbacks for gui elements
            % done at the end when all elements are generated that are
            % needed for the callback
            
            % Checkboxes
            gui.entryEmBrakeDistance.cb.ValueChangedFcn = @(cbx,event) obj.checkboxPlotObjectCallback(cbx, gui.barEmBrake);
            gui.entryDeltaDistance.cb.ValueChangedFcn = @(cbx,event) obj.checkboxPlotObjectCallback(cbx, gui.rulerDistance);
            gui.entryVelocity.cb.ValueChangedFcn = @(cbx,event) obj.checkboxPlotObjectCallback(cbx, gui.vecEgoVelocity);
            gui.entryPredictionBox.cb.ValueChangedFcn = @(cbx,event) obj.checkboxPlotObjectCallback(cbx, gui.predictionBox);
            
        end
        
        function variableEntry = generateVariableEntry(~, parent, title, unit, checkboxNeeded)
            % generate a subgridVariabeles entry for showing an variable value
            % returns a structure with following content:
                % .lbl      name of the variable
                % .value    value of the variable
                % .unit     unit of the value
                % .cb       mandatory - checkbox to activate a ploted object
            
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
            if ~checkboxNeeded
                % disable checkbox
                variableEntry.cb.Visible = false;
            end
        end
        
        function interactionElement = generateInteractionElement(obj, parent, variable, variableName, upperLimitSlider)
            % generate a interaction element showing the variable name, an
            % edit field and a slider for changing
            % Name field
            interactionElement.lbl = uilabel(parent, 'Text',variableName);
            
            % Edit field
            interactionElement.editField = uieditfield(parent,'numeric', ...
                                                              'ValueDisplayFormat', '%.2f s', ...
                                                              'Limits', [0 inf], ... % TODO: think about setting this limits
                                                              'LowerLimitInclusive', false, ...
                                                              'Value', variable, ...
                                                              'ValueChangedFcn',@(txt,event) obj.editTimeStepCallback(txt)); % TODO: evtl. input callback or use setter from variable
            % Slider
            interactionElement.slider = uislider(parent, 'Value', variable, ...
                                                         'ValueChangedFcn',@(sld,event) obj.timeStepSliderCallback(sld)); % TODO: evtl. input callback or use setter from variable
            interactionElement.slider.Limits = [0 upperLimitSlider]; % TODO: is only upper limit ok?
            
        end
        
        function areaVelocity = generateAreaVelocity(~, axes, minDeceleration, safeDistance, color)
            % generate an area showing the velocities that still allow the
            % vehicle to slow down to zero with:
                % minDeceleration   minimal Deceleration value allowed
                % safeDistance      safety zone that should not be reached by the vehicle
                % color             color of the area
            
            % Function describing the maximal velocity at minDeceleration
            distance = linspace(axes.XLim(1),axes.XLim(2));
            velocity = sqrt(2*(distance-safeDistance)*-minDeceleration);
            
            % Create colored veloctiy area
            hold(axes, 'on');
            areaVelocity = area(axes, distance, ...
                                      velocity, ...
                                      'FaceColor', color, ...
                                      'EdgeAlpha', 0.2);
            
        end
               
        function roadScenario = setupScenario(~, roadScenario, vehicles)
            % Setup the scenario already containing a road network
            
            % Add vehicles with properties of the simulated vehicles to scenario
            for i = 1 : length(vehicles)
                vehicle(roadScenario, ...
                    'ClassID', 1, ... % group 1 means cars
                    'Name', vehicles(i).id, ...
                    'Length', vehicles(i).physics.size(3), ...
                    'Width', vehicles(i).physics.size(2), ...
                    'Height', vehicles(i).physics.size(1), ...
                    'RearOverhang', vehicles(i).physics.size(3)/2); % This moves the origin from the rear axle to the middle of the car
            end
        end
        
        %% Update functions
        function update(obj, currentSimTime)
            % update gui, this method should be accessible from outside the
            % class
            
            % Optional inputs
            if nargin == 2 % TODO: use arguments
                % update with a new simulation time
                obj.setCurrentSimTime(currentSimTime);
            end
            
            % get information about the vehicle in front of the ego vehicle
            obj.getLeadingVehicleInfo();
           
            %% Set information
                        
            % update ego vehicle infromation
            obj.updateEgoVehicleArea();
            
            % update leading vehicle information
            obj.updateLeadingVehicleArea();
            
            % update safe set plot
            
            obj.updateSafeSpace();

            % update birds eye view
            obj.updateBep();
            
            %% Footer
            
            % check if simulation should pause
            obj.checkTimeToPause();
             
        end
        
        function getLeadingVehicleInfo(obj)
            % Get information about the vehicle in front of ego vehicle
            % like vehicle, velocity and distance
            
            % Leading vehicle
            obj.leadingVehicle = obj.egoVehicle.sensors.leadingVehicle;
            
            % Relative distance
            obj.relativeDistance = obj.egoVehicle.sensors.frontDistance;
            if obj.egoVehicle.sensors.frontSensorRange < obj.egoVehicle.sensors.frontDistance
                obj.relativeDistance = inf;
            else
                obj.relativeDistance = obj.egoVehicle.sensors.frontDistance;
            end
            
            % Relative speed
            if obj.leadingVehicle ~= 0
                obj.relativeSpeed = obj.egoVehicle.dynamics.speed - obj.leadingVehicle.dynamics.speed;            
            else
                obj.relativeSpeed = inf;
            end
        end
        
        function updateEgoVehicleArea(obj)
            % Update all information in the ego vehicle area
            
            % calculate current emergency brake distance
            obj.emergencyBrakeDistance = obj.calculateEmergencyBrakeDistance();
            
            % Update variable entrys
            obj.updateVariableEntry(obj.gui.entrySimTime, obj.currentSimTime); % TODO: should be moved to an other function, is not ego vehicle
            obj.updateVariableEntry(obj.gui.entryVelocity, obj.egoVehicle.dynamics.speed);
            obj.updateVariableEntry(obj.gui.entryEmBrakeDistance, obj.emergencyBrakeDistance);
            obj.updateVariableEntry(obj.gui.entryDrivingMode, obj.egoVehicle.status.drivingMode);
        end
        
        function updateLeadingVehicleArea(obj)
            % Update all information in the leading vehicle area
            
            % Edit fields need no update in the moment, they are the only
            % one writing the values
            
            % Update variable entrys
            obj.updateVariableEntry(obj.gui.entryDeltaDistance, obj.relativeDistance);
            obj.updateVariableEntry(obj.gui.entryDeltaSpeed, obj.relativeSpeed);
        end
        
        function updateSafeSpace(obj)
            % update objects in safe space plot
            
            %% actual set
            obj.gui.pointActualSet.XData = obj.relativeDistance;
            obj.gui.pointActualSet.YData = obj.relativeSpeed;
            
            %% predicted sets TODO: use new ts and k
            nrSteps = obj.safeSpace.nrSteps;
            Ts = obj.safeSpace.ts; % time step
            deltaX = obj.gui.pointActualSet.XData;
            deltaV = obj.gui.pointActualSet.YData;
            deltaAcc = obj.egoVehicle.dynamics.minDeceleration; % TODO: match all values to the right coordinate system
            
            % starting point of line is actual set
            obj.gui.lineNextSets.XData = obj.gui.pointActualSet.XData;
            obj.gui.lineNextSets.YData = obj.gui.pointActualSet.YData;
            
            for i=1:nrSteps
                % calulate speed and distance for next time step
                deltaV = deltaV + deltaAcc*Ts;
                deltaX = deltaX - deltaV*Ts - deltaAcc*Ts^2/2;
                % safe values in plotted line
                obj.gui.lineNextSets.XData(end+1) = deltaX;
                obj.gui.lineNextSets.YData(end+1) = deltaV;
            end
            
        end
           
        function updateBep(obj)
            % update birds eye plot
            
            %% update vehicles
            % get vehicle poses
            positions = cat(1,cat(2,obj.vehicles(1:length(obj.vehicles)).dynamics).position);
            orientations = cat(1,cat(2,obj.vehicles(1:length(obj.vehicles)).dynamics).orientation);
            % update vehicle position and orientation
            obj.updateVehiclePoses(positions(:,1), positions(:,3), orientations(:,4));
            % redraw vehicles
            [position,yaw,Length,width,originOffset,color] = targetOutlines(obj.scenario.Actors(obj.egoVehicle.id));
            plotOutline(obj.gui.outlinePlotter, position, yaw, Length, width, ...
                'OriginOffset',originOffset, 'Color',color);
            %% update roads
            % redraw roads from ego vehicle pose
            rb = roadBoundaries(obj.scenario.Actors(obj.egoVehicle.id)); % Maybe use laneBoundaries and only show a part of the map
            plotLaneBoundary(obj.gui.lanePlotter,rb);           
            
            %% Arrows
            obj.gui.vecEgoVelocity.update(obj.egoVehicle.dynamics.speed);
            
            %% Ruler
            % update length of distance ruler
            obj.gui.rulerDistance.update(obj.relativeDistance);
            
            %% Bar
            obj.gui.barEmBrake.update(obj.emergencyBrakeDistance);
            
            %% Prediction box
            % Test predict motion parameters TODO: move this part!
            i=1:obj.ts;
            futureDistance1 = obj.predictMotion(obj.ts, i*2, 0, obj.egoVehicle.dynamics.speed);
            futureDistance2 = obj.predictMotion(obj.ts, i*0, 0, obj.egoVehicle.dynamics.speed);
            futureDistance3 = obj.predictMotion(obj.ts, i*-1, 0, obj.egoVehicle.dynamics.speed);
            futureDistance4 = obj.predictMotion(obj.ts, i*-3, 0, obj.egoVehicle.dynamics.speed);
            
            obj.gui.predictionBox.update([futureDistance1 futureDistance2 futureDistance3 futureDistance4]);
            
        end      
        
        function updateVehiclePoses(obj, x, y, yaw)
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
            obj.gui.vehicleSelectionDd.Value = obj.egoVehicle.id;
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
                
        function emergencyBrakeDistance = calculateEmergencyBrakeDistance(obj)
            % Calculate the distance a vehicle needs to stop with min deceleration
            %TODO: move this to a safety component, should not be in UI
            
            % get current dynamic parameters
            curSpeed = obj.egoVehicle.dynamics.speed;
            minAcceleration = obj.egoVehicle.dynamics.minDeceleration;
            % calculate distance
            emergencyBrakeDistance = 0.5*-curSpeed^2/minAcceleration;
        end
        
        function [futureDistance, futureSpeed] = predictMotion(~, tStep, accelerations, curDistance, curSpeed)
            % calculate future distance and speed after all accelerations
            % applied
            %
            %   tStep               % time between two acceleration values in s
            %   accelerations       % row vector of actual and future acceleration values
            %   curDistance         % current distance covered
            %   curSpeed            % current speed
            %
            % the math of this formulars can be found in DA Pintscher
            
            nrSteps = length(accelerations); % number of steps to predict
            
            % future speed calculation
            futureSpeed = curSpeed + tStep*sum(accelerations);
            
            % future distance calculation
            n=1:nrSteps; % first value is 1 instead of 0 because of matlab indexing
            futureDistance = curDistance + tStep*nrSteps*curSpeed + ...
                             tStep^2/2*sum(accelerations(n).*(2*nrSteps+1-2*n));
                         
            % restrict motion parameters, vehicle should not move backwards
            if futureSpeed < 0, futureSpeed = 0; end
            if futureDistance < 0, futureDistance = 0; end
        end
        
        %% Callbacks
        function vehicleSelectionCallback(obj, dropdown)
            % Callback of the vehicle selection dropdown menu
            
            % Change the ego vehicle according to the selected name
            obj.setEgoVehicle(dropdown.Value);
            obj.update();        
        end
        
        function pauseSimTimeCallback(obj, btn)
            % pause/resume simulation execution
            
            if btn.Value == true
                if obj.currentSimTime == 0 % start simulation
                    btn.Enable = false; % deactivate to prevent multiple calls
                    waitfor(btn,'Enable','off');
                    % The button change to pause button is executed after!!!
                    % the simulation is started! (UI stuff actualizing
                    % takes some time, but code execution is not stopped.
                    % Simulation preparing takes all resources, so the button
                    % is not actualized until the preparation is finished)
                    % This behavior is intended!
                    obj.gui.pauseBtn.Text = "Pause";
                    btn.Value = false;
                    btn.Enable = true; 
                    % start simulation
                    evalin('base', 'run_Sim');
                else
                    uiwait % pause simulation
                end
            else
                uiresume % unpause simulation
            end
        end
        
        function checkboxPlotObjectCallback(obj, cbx, plotObject)
            % Callback for a checkbox (cbx) triggering a activtable plot
            % object
            
            plotObject.Active = cbx.Value; % change active
            obj.updateBep(); % update the birds eye plot that shows the object
        end
        
        function editTimeStepCallback(obj, txt)
            % Callback for the edit field for changing the time step of
            % safe space
            
            obj.ts = txt.Value;
            obj.updateSafeSpace(); % TODO: not only safe space, now also bep
        end
        
        function editNrStepsCallback(obj, txt)
            % Callback for the edit field for changing the number of steps
            % to predict at safe space
            
            obj.safeSpace.k = txt.Value;
            obj.updateSafeSpace();
        end
        
        function timeStepSliderCallback(obj, sld)
            % Callback of the time step changing slider
            obj.ts=sld.Value;
            obj.updateBep();
        end
        
        function nrStepsSliderCallback(obj, sld)
            % Callback of the number of steps changing slider
            obj.k=sld.Value;
            obj.updateBep();
        end
        %% Setter/Getter 
                
        function setCurrentSimTime(obj, currentSimTime)
            % Set current simulation time
            
            obj.currentSimTime = currentSimTime;
        end
        
        function setTimeToPause(obj, time)
            % set the time to pause simulation
            
            obj.timeToPause = time;
        end
                
        function setEgoVehicle(obj, egoVehicleId)
            % Set the ego vehicle by id
            
            obj.egoVehicle = obj.vehicles(egoVehicleId);
        end

    end
end

