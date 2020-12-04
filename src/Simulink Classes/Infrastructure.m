classdef Infrastructure < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.CustomIcon
    % This block initializes the vehicles on the map, draws and updates their positions on the 2D plot. When a collision happens, the MAP instance detects it and stops the colliding vehicles. Apart from that, it contains an AIM (Autonomous Intersection Manager) to avoid conflictions on intersections. The decisions are sent to the vehicles as stop, wait, and go.
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.
    
    % Public, tunable properties
    properties(Access = private)
        map = evalin('base','Map');
        modelName = evalin('base','modelName');
    end
    
    properties(DiscreteState)
        
    end
    
    % Pre-computed constants
    properties(Access = private)
        
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            %Initializing the Vehicles on the Map
            for vehicle = obj.map.Vehicles
                vehicle.initVehicle();
            end
        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = matlab.system.display.Icon("logo_small.png");
        end
        
        function mergedBrakingFlagArrays = stepImpl(obj, V2Idata)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            for i=1:size(V2Idata,1)
                if ~(V2Idata(i,1) == 0)
                    vehicle = obj.map.Vehicles(i);
                    if V2Idata(i,2) == 1
                        % Car reaches the crossroad
                        obj.map.crossroadUnits(V2Idata(i,1)).carReachesCrossroad(vehicle, vehicle.pathInfo.lastWaypoint);
                    elseif V2Idata(i,2) == 2
                        % Car reaches the braking point
                        obj.map.crossroadUnits(V2Idata(i,1)).carReachesBreakingPoint(vehicle, vehicle.pathInfo.lastWaypoint, obj.getCurrentTime);
                    elseif V2Idata(i,2) == 3
                        
                    elseif V2Idata(i,2) == 4
                        % Car leaves the crossroad
                        obj.map.crossroadUnits(V2Idata(i,1)).carLeavesCrossroad(vehicle, obj.getCurrentTime)
                    end
                end
            end
            
            mergedBrakingFlagArrays = []; % Memory preallocation is not feasible since the assignment size is variable
            for i=1:length(obj.map.crossroadUnits)
                mergedBrakingFlagArrays =  [mergedBrakingFlagArrays; obj.map.crossroadUnits(i).breakingFlagArray]; %#ok<AGROW>
            end
            
            
            %% 2D Traffic Plot
            obj.map.dynamicTrafficPlot();
            %obj.map.dynamicRouteHighlighting(); % enable for dynamic route highlighting - disable for performance

            %% Ped walking
            for j=1:size(obj.map.peds,2)
                info(j,:)=[obj.map.peds(j).positionLocal obj.map.peds(j).direction];    
            end
            
            for m=1:size(obj.map.peds,2)
                desired(m,:)=obj.map.peds(m).findnewpos(obj.map.mapCrosswalkFull,info);
            end
            
                ver=ones(1,size(desired,1));
            for k=1:size(desired,1)-1
                for l=k+1:size(desired,1)           
                        if desired(k,:)==desired(l,:)
                            if randn(1)>0.5
                                ver(k)=0;
                                ver(l)=1;
                            else
                                ver(k)=1;
                                ver(l)=0;
                            end
                        end
                end
            end

             for h=1:size(obj.map.peds,2)
                update(h,:)=obj.map.peds(h).updatepos(ver(h));
             end
            
%             for i=1:length(obj.map.peds)
%                 newpos(i,1:2)=obj.map.peds(i).method1             
%             end
            
            obj.map.plots.Pedestrians.XData = update(:,1);
            obj.map.plots.Pedestrians.YData = update(:,2);
            
            %% Collision detection
            if obj.getCurrentTime>0.1 % If you check collision right away, all vehicles collide at time 0.0
                for vehicle = obj.map.Vehicles
                    vehicle.checkCollision(obj.map.Vehicles);
                end
            end
        end

        
        % These have to be specified because of
        % matlab.system.mixin.Propagates, to have variable size of outputs
        % and to avoid code generation which does not support the digraph
        function mergedBrakingFlagArrays = isOutputFixedSizeImpl(~)
            %Both outputs are always variable-sized
            mergedBrakingFlagArrays = false;
        end
    end



    methods(Static,Access = protected)
        function resetImpl(~)
            % Initialize / reset discrete-state properties
            
        end
        function mergedBrakingFlagArrays = getOutputSizeImpl(~)
            % Maximum length of the output
            mergedBrakingFlagArrays = [10 20];
            
        end
        
        function mergedBrakingFlagArrays = getOutputDataTypeImpl(~)
            mergedBrakingFlagArrays = 'double'; %Linear indices are always double values
        end
        
        function mergedBrakingFlagArrays = isOutputComplexImpl(~)
            mergedBrakingFlagArrays = false; %Linear indices are always real values
        end
    end
end
