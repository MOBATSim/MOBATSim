classdef Map < handle
    %Map Superclass for all maps.
    %   Detailed explanation goes here
    
    properties
        mapName
        waypoints               %all nodes: line = nr, [X,Y]
        connections             %struct, .all = all edges, .circles = all curves, .translations = all straight roads
        directedGraph
        Vehicles                %vector of all vehicles: line = nr
        plots
        crossroadUnits
        crossroads
        Route_LaneNumber % lane number of the route
    end
    
    methods
        function obj = Map(mapName,waypoints, connections_circle,connections_translation, startingNodes, breakingNodes, stoppingNodes, leavingNodes, Route_LaneNumber)
            obj.mapName = mapName;
            obj.waypoints = waypoints;
            obj.Vehicles = [];
            obj.connections.all = [connections_circle(:,1:2);connections_translation(:,1:2)];
            obj.connections.maxSpeeds = [connections_circle(:,end); connections_translation(:,end)];
            obj.connections.circle = connections_circle;
            obj.connections.translation = connections_translation;
            obj.plots.trajectories = [];
            obj.Route_LaneNumber = Route_LaneNumber;
            
            obj.crossroads.startingNodes = startingNodes;
            obj.crossroads.breakingNodes = breakingNodes;
            obj.crossroads.stoppingNodes = stoppingNodes;
            obj.crossroads.leavingNodes = leavingNodes;
            
            for i = 1:  size(startingNodes,1)
                
                obj.crossroadUnits = [obj.crossroadUnits; CrossroadUnit(i,startingNodes(i,:),breakingNodes(i,:),stoppingNodes(i,:),leavingNodes(i,:))];
            end
            %% Calculate curved distances (Lengths of circular routes)
            distancesCircle = ones(1,size(connections_circle,1)); % memory preallocation
            for i = 1:size(connections_circle,1)
                connection = connections_circle(i,:);
                radius = norm(connection(4:6) - waypoints(connection(1),:));
                distancesCircle(i) = radius * abs(connection(3));
            end
            
            %% Calculate translation distances (Lengths of straight routes)
            distancesTranslation = ones(1,size(connections_translation,1)); % memory preallocation
            for i = 1:size(connections_translation,1)
                connection = connections_translation(i,:);
                distancesTranslation(i) = norm(waypoints(connection(1),:)-waypoints(connection(2),:));
            end
            %%
            % Concatenate all distances 
            obj.connections.distances = [distancesCircle';distancesTranslation'];

            %create direct graph with related weights
            obj.directedGraph = digraph( [obj.connections.circle(:,1)' obj.connections.translation(:,1)'],[obj.connections.circle(:,2)' obj.connections.translation(:,2)'],[ obj.connections.distances']);
            
        end %Constructor
        
        function lanenumber = get_lane_number_from_route (obj, routenumber)%to find the lane number of the route
             lanenumber = obj.Route_LaneNumber(routenumber,2);
        end    
        
        function waypoint = get_waypoint_from_coordinates (obj,coordinates)
            
            [~,waypoint] = ismember(coordinates, obj.waypoints, 'rows');
            
        end
        
        function coordinates = get_coordinates_from_waypoint (obj,waypoint)
            
            coordinates = obj.waypoints(waypoint,:);
            
        end
        
        function index = getRouteIDfromPath(obj, path)
            point1 = path(1);
            point2 = path(2);
            index = find(obj.connections.all(:,1) ==point1&obj.connections.all(:,2) ==point2);
            
        end        
        
        function stopCollidingVehicles(~, car)
            if car.status.emergencyCase == 3 % Collision
                % Inform the map and/or the Decision Unit about what happened
                car.dynamics.speed = 0;
                car.setStopStatus(true);
            end
            
        end
        
        function neighbourRoutes = getForwardNeighbourRoutes(obj, route)
            connections = obj.connections.all;
            connection = connections(route,:);
            neighbourRoutes =find(connections(:,1)==connection(2));
            
        end
        
        function neighbourRoutes = getBackwardNeighbourRoutes(obj, route)
            connections = obj.connections.all;
            connection = connections(route,:);
            neighbourRoutes =find(connections(:,2)==connection(1));
            
        end
        
        function closestWaypoint = getClosestWaypoint(obj, waypoint)
            cellWaypoints = num2cell(obj.waypoints,2);
            x = cellfun(@(x) norm(x-obj.waypoints(waypoint,:)), cellWaypoints);
            closestWaypoint = find(x==(min(x(x>0))));
        end
        
        function formattedRoute = getRouteDefinitionfromRouteID(obj, routeID)
            
            
            if routeID>length(obj.connections.circle)
                routeID = routeID - length(obj.connections.circle); % Some transformation to correct the error -> TODO: Check if consistent
                formattedRoute = [obj.waypoints(obj.connections.translation(routeID,1),:);
                    obj.waypoints(obj.connections.translation(routeID,2),:);
                    zeros(1,3);
                    zeros(1,3)];
            elseif routeID<=length(obj.connections.circle)
                formattedRoute = [obj.waypoints(obj.connections.circle(routeID,1),:);
                    obj.waypoints(obj.connections.circle(routeID,2),:);
                    abs(obj.connections.circle(routeID,3)),obj.connections.circle(routeID,4),obj.connections.circle(routeID,6);
                    -sign(obj.connections.circle(routeID,3))*ones(1,3)];
                
            end
            
            
        end
        
        %% Plotting related functions       
        function routeColor = getRouteColorFromSpeed(~, speed)
            if speed > 50
                routeColor = [1-(speed-50)/62.5 0.8 0];
            else
                routeColor = [0.8 speed/62.5  0];
            end
            
        end
        
        function PlotMap(obj)
        % Turn off useless properties for performance optimization
            MapFig = gcf;
            MapFig.WindowState ='maximized';
            %MapFig.WindowState ='fullscreen';
            MapFig.Name = obj.mapName;
            MapFig.NumberTitle = 'off';
            ax = gca;
            ax.Toolbar = [];
            ax.Interactions = [];
            view(2)
            hold on
            obj.plots.Vehicles = scatter([],[],380,'filled'); % Size of the vehicle bubbles            
            hold off
                    
            obj.plots.Vehicles.ZData = 0.01 .* ones(1,10); % Warning: This may sometimes cause a bug and disable the figure zoom for map.

        end
        
        function initCarDescriptionPlot(obj)
            % Prepares the Vehicle tags
            allVehiclePositions = cat(1,cat(1,obj.Vehicles.dynamics).position);

            obj.plots.Vehicles.XData = allVehiclePositions(:,1);
            obj.plots.Vehicles.YData = -allVehiclePositions(:,3);
            obj.plots.carDescription=text(allVehiclePositions(:,1)',-allVehiclePositions(:,3)',{obj.Vehicles.name},'FontWeight','Bold','FontSize',9);
            
            allVehiclePositions = [allVehiclePositions(1:length(obj.Vehicles),1)-5, -allVehiclePositions(1:length(obj.Vehicles),3)+2, 0.011.*ones(1,10)'];
            allTextPositions = mat2cell(allVehiclePositions,ones(1,10),3); % Matrix to Cell for the handle format
            
            % Set the position of the Text handles
            set(obj.plots.carDescription,{'Position'},allTextPositions);
        end
        
        function initialGraphHighlighting(obj)
            obj.plots.graph.EdgeColor = 'g';
        end
        
        function dynamicTrafficPlot(obj)
            
            allVehiclePositions = cat(1,cat(1,obj.Vehicles(1:length(obj.Vehicles)).dynamics).position);
            
            % Vehicles' 2D scatter plot circle positions
            obj.plots.Vehicles.XData = allVehiclePositions(:,1);
            obj.plots.Vehicles.YData = -allVehiclePositions(:,3);
            
            % Vehicles' Annotation String
            speedArray = compose('%4.1f', [cat(1,cat(1,obj.Vehicles(1:length(obj.Vehicles))).dynamics).speed]);
            nameArray={obj.Vehicles(1:length(obj.Vehicles)).name};
            
            %requiredArrayHandle= get(obj.plots.carDescription,{'String'}); % The required format for the handles
            % TODO: If the number of vehicles on the map are not known, use the for loop below, but the performance is
            % 10 times better with the long line of code that comes after. Need to find a solution to the conversion of
            % cell arrays.
            %             textDescArray = cell(length(obj.Vehicles),1);
            %             for k=1:length(obj.Vehicles)
            %                 textDescArray(k) = {{nameshow{k};speedshow{k}}};
            %             end
            
            %This code gives the best performance but needs to be flexible later on.
            textDescArray = {{nameArray{1};speedArray{1}};{nameArray{2};speedArray{2}};{nameArray{3};speedArray{3}};...
                {nameArray{4};speedArray{4}};{nameArray{5};speedArray{5}};{nameArray{6};speedArray{6}};{nameArray{7};speedArray{7}};...
                {nameArray{8};speedArray{8}};{nameArray{9};speedArray{9}};{nameArray{10};speedArray{10}}};
            
            
            
            % Vehicles' Annotation Position
            allVehiclePositions = [allVehiclePositions(1:length(obj.Vehicles),1)-8, -allVehiclePositions(1:length(obj.Vehicles),3)+8, 0.011.*ones(1,10)'];
            allTextPositions = mat2cell(allVehiclePositions,ones(1,10),3); % Matrix to Cell for the handle format
            
            %set the position and string handles
            set(obj.plots.carDescription,{'Position'},allTextPositions);
            set(obj.plots.carDescription,{'String'},textDescArray);
            
        end
        

        
        %% For debugging and analysis - not implemented
        function costs = getCosts(obj, point1, point2)
            index = find(([obj.connections.translation(:,1);obj.connections.circle(:,1)] ==point1)&([obj.connections.translation(:,2);obj.connections.circle(:,2)] ==point2));
            costs_vector = [obj.connections.costs.translation obj.connections.costs.circle] ;
            costs = costs_vector(index);
        end
        
        
        function path = get_shortest_path(obj, starting_point, ending_point)
            
            [path,distance] = shortestpath(obj.directedGraph,starting_point,ending_point);
            
        end
        
        function path_in_waypoints = find_shortest_path_as_waypoints(obj, starting_point, ending_point )
            
            [path,distance] = shortestpath(obj.directedGraph,starting_point,ending_point);
            path_in_waypoints = obj.waypoints(path,:);
            
        end
        
        function speed = get_speed (obj, route, maxSpeed)
            point1 = get_waypoint_from_coordinates (obj,route(1,:));
            point2 = get_waypoint_from_coordinates (obj,route(2,:));
            
            index = find(([obj.connections.translation(:,1);obj.connections.circle(:,1)] == point1)&([obj.connections.translation(:,2);obj.connections.circle(:,2)] ==point2));
            if index > length(obj.connections.translation)
                speed = obj.connections.circle(index -  length(obj.connections.translation),end);
            else
                speed = obj.connections.translation(index,end);
            end
            
            if maxSpeed < speed
                speed = maxSpeed;
            end
            
        end
        

    end
    methods (Static)
        function coordinates2D = transform3DAnimTo2Dcoordinate(coordinates3D)
            
            coordinates2D = [coordinates3D(1) -coordinates3D(3)];
            
        end
        
        function coordinates3D = transform2DcoordinateTo3DAnim(coordinates2D)
            
            coordinates3D = [coordinates2D(1) 0 -coordinates2D(3)];
            
        end
        
        function coordinates3D = transformPoseTo3DAnim(pose)
            
            coordinates3D = [pose(1) 0 -pose(2)];
            
        end
    end
    
    methods (Abstract)
    dynamicRouteHighlighting(obj)
    end
    
  
    
end

