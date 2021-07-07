classdef DigraphMap < Map
    % Digraph-based Map object inherited from Map Class
    
    properties
        
    end
    
    methods
        function obj = DigraphMap(mapName,waypoints, connections_circle,connections_translation, startingNodes, breakingNodes, stoppingNodes, leavingNodes, Route_LaneNumber)
            
            obj = obj@Map(mapName,waypoints, connections_circle,connections_translation, startingNodes, breakingNodes, stoppingNodes, leavingNodes, Route_LaneNumber);
            
               
            for i=1:length(obj.connections.all)
                graphConnectionsLabel(i) = find(obj.connections.all(:,1) == obj.directedGraph.Edges.EndNodes(i,1)&obj.connections.all(:,2) == obj.directedGraph.Edges.EndNodes(i,2));
            end
            
            % Plot the map on the figure
            hold off
            obj.plots.graph = plot(obj.directedGraph,'XData',obj.waypoints(:,1),'YData',-obj.waypoints(:,3),'EdgeLabel',graphConnectionsLabel');
            
            obj.PlotMap();
            obj.initialGraphHighlighting();
            obj.plots.graph.LineWidth = 2;
            
        end %Constructor
        
        function initialGraphHighlighting(obj)
            obj.plots.graph.EdgeColor = 'g';
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

