
classdef Map < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mapName
        waypoints
        connections
        direct_graph
        digraph_visualization
        digraph_visualization2
        Vehicles
        plots
        crossroadUnits
        crossroads
    end
    
    methods
        function obj = Map(mapName,waypoints, connections_circle,connections_translation, startingNodes, breakingNodes, stoppingNodes, leavingNodes)
            obj.mapName = mapName;
            obj.waypoints = waypoints;
            obj.Vehicles = [];
            obj.connections.all = [connections_circle(:,1:2);connections_translation(:,1:2)];
            obj.connections.maxSpeeds = [connections_circle(:,end); connections_translation(:,end)];
            obj.connections.circle = connections_circle;
            obj.connections.translation = connections_translation;
            
            
            %calculate distances of connections
            
            for i = 1:size(connections_circle,1)
                connection = connections_circle(i,:);
                radius = norm(connection(4:6) - waypoints(connection(1),:));
                distancesCircle(i) = radius * abs(connection(3));
            end
            
            
            
            for i = 1:size(connections_translation,1)
                connection = connections_translation(i,:);
                distancesTranslation(i) = norm(waypoints(connection(1),:)-waypoints(connection(2),:));
            end
            
            obj.connections.distances = [distancesCircle';distancesTranslation'];
            
            %create direct graph with related weights
            
            obj.direct_graph = sparse(obj.connections.all(:,1),obj.connections.all(:,2),[  obj.connections.distances']);
            
            %h = view(biograph(direct_graph,[],'ShowWeights','on'));
            
            obj.digraph_visualization = digraph( [obj.connections.circle(:,1)' obj.connections.translation(:,1)'],[obj.connections.circle(:,2)' obj.connections.translation(:,2)'],[ obj.connections.distances']);
%             obj.digraph_visualization2 = graph( [obj.connections.circle(:,1)' obj.connections.translation(:,1)'],[obj.connections.circle(:,2)' obj.connections.translation(:,2)'],[ obj.connections.distances']);
            
            
            for i=1:length(obj.connections.all)
                
                graphConnectionsLabel(i) = find(obj.connections.all(:,1) == obj.digraph_visualization.Edges.EndNodes(i,1)&obj.connections.all(:,2) == obj.digraph_visualization.Edges.EndNodes(i,2));
%                 graphConnectionsLabel(i) = find(obj.connections.all(:,1) == obj.digraph_visualization2.Edges.EndNodes(i,1)&obj.connections.all(:,2) == obj.digraph_visualization2.Edges.EndNodes(i,2));

            end
            
            % Plot the map on the figure
            figure('units','normalized','outerposition',[0 0 1 1])
            
            obj.plots.graph2 = plot(obj.digraph_visualization,'XData',obj.waypoints(:,1),'YData',-obj.waypoints(:,3),'LineWidth',4,'NodeFontSize',0.5,'ArrowSize',1,'MarkerSize',2.5);
%             set(gcf, 'Position', get(0, 'Screensize'))
            axis equal
            axis([-600 600 -500 500])
%             axis off tight
            F=getframe(gca);
            f=frame2im(F);
            imshow(f)
            
            g = gcf;
            exportgraphics(g,'graphMS.png','Resolution',349)
%             F=getframe(gca);
%             imwrite(F.cdata,'graphMS.png')
%             saveas(gcf,'graphMS.png');
            image = imread('graphMS.png');
            
            
            grayimage = rgb2gray(image);
            bwimage = ~(grayimage < 200 & grayimage >87);
            grid = binaryOccupancyMap(bwimage,2);
%             redmap=binaryOccupancyMap(grid(100:200,100:200),2)
            figure
            show(grid)
            figure
            obj.plots.graph = plot(obj.digraph_visualization,'XData',obj.waypoints(:,1),'YData',-obj.waypoints(:,3),'EdgeLabel',graphConnectionsLabel');
            

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
            
            
            obj.initialGraphHighlighting();
            obj.plots.graph.LineWidth = 2;
            
            obj.crossroads.startingNodes = startingNodes;
            obj.crossroads.breakingNodes = breakingNodes;
            obj.crossroads.stoppingNodes = stoppingNodes;
            obj.crossroads.leavingNodes = leavingNodes;
            
            for i = 1:  size(startingNodes,1)
                
                obj.crossroadUnits = [obj.crossroadUnits; CrossroadUnit(i,startingNodes(i,:),breakingNodes(i,:),stoppingNodes(i,:),leavingNodes(i,:))];
            end
            
        end %Constructor
        
        function  distance = get_distance_of_shortest_path(obj, starting_point, ending_point)
            
            [distance,path] = graphshortestpath(obj.direct_graph,starting_point,ending_point);
            
        end
        
        function path_in_waypoints = find_shortest_path_as_waypoints(obj, starting_point, ending_point )
            
            [distance,path] = graphshortestpath(obj.direct_graph,starting_point,ending_point);
            path_in_waypoints = obj.waypoints(path,:);
            
        end
        
        function waypoint = get_waypoint_from_coordinates (obj,coordinates)
            
            [~,waypoint] = ismember(coordinates, obj.waypoints, 'rows');
            
        end
        
        function coordinates = get_coordinates_from_waypoint (obj,waypoint)
            
            coordinates = obj.waypoints(waypoint,:);
            
        end
        
        function costs = getCosts(obj, point1, point2)
            index = find(([obj.connections.translation(:,1);obj.connections.circle(:,1)] ==point1)&([obj.connections.translation(:,2);obj.connections.circle(:,2)] ==point2));
            costs_vector = [obj.connections.costs.translation obj.connections.costs.circle] ;
            costs = costs_vector(index);
        end
        
        function index = getRouteIDfromPath(obj, path)
            point1 = path(1);
            point2 = path(2);
            index = find(obj.connections.all(:,1) ==point1&obj.connections.all(:,2) ==point2);
            
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
            
        end %unused function
        
        
        function stopCollidingVehicles(obj, car)
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
        
        function routeColor = getRouteColorFromSpeed(obj, speed)
            if speed > 50
                routeColor = [1-(speed-50)/62.5 0.8 0];
            else
                routeColor = [0.8 speed/62.5  0];
            end
            
        end
        
        function initCarDescriptionPlot(obj)
            % Prepares the Vehicle tags
            obj.plots.carDescription=text(zeros(1,10),zeros(1,10),{obj.Vehicles.name},'FontWeight','Bold','FontSize',9);
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
            allVehiclePositions = [allVehiclePositions(1:length(obj.Vehicles),1)-10, -allVehiclePositions(1:length(obj.Vehicles),3)+12, zeros(1,10)'];
            allTextPositions = mat2cell(allVehiclePositions,ones(1,10),3); % Matrix to Cell for the handle format
            
            %set the position and string handles
            set(obj.plots.carDescription,{'Position'},allTextPositions);
            set(obj.plots.carDescription,{'String'},textDescArray);
            
        end
        
        function dynamicRouteHighlighting(obj)
            
            if(isempty(findobj('type','figure')))
                
            else
                obj.initialGraphHighlighting();
                for vehicle = obj.Vehicles
                    if length(vehicle.pathInfo.path) > 1
                        if vehicle.dynamics.speed < 27.7
                            routeColor = obj.getRouteColorFromSpeed(vehicle.dynamics.speed*3.6);
                            highlight(obj.plots.graph,vehicle.pathInfo.path(1),vehicle.pathInfo.path(2),'EdgeColor', routeColor)
                        end
                    end
                end
            end
            
        end
        
        
        
        
    end
    
end

