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
        crossroadUnits          % objects controling a crossroad
        Route_LaneNumber % lane number of the route
    end
    
    methods
        function obj = Map(mapName,waypoints, connections_circle,connections_translation, startingNodes, brakingNodes, stoppingNodes, leavingNodes, Route_LaneNumber)
            obj.mapName = mapName;
            obj.waypoints = waypoints;
            obj.Vehicles = []; % Vehicles are added after vehicle generation
            obj.connections.all = [connections_circle(:,1:2);connections_translation(:,1:2)];
            obj.connections.maxSpeeds = [connections_circle(:,end); connections_translation(:,end)];
            obj.connections.circle = connections_circle;
            obj.connections.translation = connections_translation;
            obj.plots.trajectories = [];
            obj.Route_LaneNumber = Route_LaneNumber;
            
            % Crossroad units            
            for i = 1:  size(startingNodes,1)
                
                obj.crossroadUnits = [obj.crossroadUnits; CrossroadUnit(i, startingNodes(i,:),brakingNodes(i,:),stoppingNodes(i,:),leavingNodes(i,:))];
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
            obj.directedGraph = digraph( [obj.connections.circle(:,1)' obj.connections.translation(:,1)'],[obj.connections.circle(:,2)' obj.connections.translation(:,2)'], obj.connections.distances');
            
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
        
        function initialYawAngle = getInitialYawAnglefromWaypoint(obj,waypoint)
            
            possibleForwardPaths = obj.connections.all(obj.connections.all(:,1)==waypoint,:);          
            forwardRouteID = getRouteIDfromPath(obj, possibleForwardPaths(1,:));          
            initialYawAngle = getSpeedVectorAnglefromRouteDefinition(obj,forwardRouteID);

        end
        
        function theta = getSpeedVectorAnglefromRouteDefinition(obj, routeID)
            forwardRouteDefinition = getRouteDefinitionfromRouteID(obj,routeID);
            
            if forwardRouteDefinition(4,1) == 0 % Straight
                speedVector = forwardRouteDefinition(2,:) - forwardRouteDefinition(1,:);
                theta = atan2(-speedVector(3),speedVector(1)); % MOBATSim coordinates -(3) = y, (1) =x
                
                if theta == -pi
                    theta = -theta; %Correction for sometimes -pi. It results an initial circle move for the vehicle.
                end
                
            else%Rotation
                vector_z=[0 0 1];
                rotation_point = [forwardRouteDefinition(3,2) 0 forwardRouteDefinition(3,3)];
                point_to_rotate = forwardRouteDefinition(1,:);         
                                    
                a=point_to_rotate(1)-rotation_point(1);
                b=point_to_rotate(3)-rotation_point(3);

                vectorB = [a b 0];
                
                if forwardRouteDefinition(4,1) == -1 % -1 means turn left
                    vectorSpeedDir = cross(vectorB,vector_z); %left
                    theta=-atan2(vectorSpeedDir(2),vectorSpeedDir(1));       
                    
                elseif forwardRouteDefinition(4,1) == 1 % 1 means turn right
                    vectorSpeedDir = cross(vector_z,vectorB);% right
                    theta=-atan2(vectorSpeedDir(2),vectorSpeedDir(1));
                end
            end
        end
        
        function neighbourRoutes = getForwardNeighbourRoutes(obj, route)
            allConnections = obj.connections.all;
            connection = allConnections(route,:);
            neighbourRoutes =find(allConnections(:,1)==connection(2));
            
        end
        
        function neighbourRoutes = getBackwardNeighbourRoutes(obj, route)
            allConnections = obj.connections.all;
            connection = allConnections(route,:);
            neighbourRoutes =find(allConnections(:,2)==connection(1));
            
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
            obj.plots.Vehicles = scatter([],[],380,'filled', 'MarkerFaceColor',[0.5 0 0.5]); % Size of the vehicle bubbles            
            hold off

        end
        
        function initCarDescriptionPlot(obj)
            % Prepares the Vehicle tags
            allVehiclePositions = cat(1,cat(1,obj.Vehicles.dynamics).position);

            obj.plots.Vehicles.XData = allVehiclePositions(:,1);
            obj.plots.Vehicles.YData = -allVehiclePositions(:,3);
            obj.plots.Vehicles.ZData = 0.01 + allVehiclePositions(:,2); % Warning: This may sometimes cause a bug and disable the figure zoom for map.
            obj.plots.carDescription=text(allVehiclePositions(:,1)',-allVehiclePositions(:,3)',num2cell("V" + cat(1,obj.Vehicles.id))','FontWeight','Bold','FontSize',9);
            
            allVehiclePositions = [allVehiclePositions(:,1)-5, -allVehiclePositions(:,3)+2, allVehiclePositions(:,2)+0.01];
            allTextPositions = mat2cell(allVehiclePositions,ones(1,size(allVehiclePositions,1)),3); % Matrix to Cell for the handle format
            
            % Set the position of the Text handles
            set(obj.plots.carDescription,{'Position'},allTextPositions);
        end
        
        function dynamicTrafficPlot(obj)
            
            allVehiclePositions = cat(1,cat(1,obj.Vehicles.dynamics).position);
            
            % Vehicles' 2D scatter plot circle positions
            obj.plots.Vehicles.XData = allVehiclePositions(:,1);
            obj.plots.Vehicles.YData = -allVehiclePositions(:,3);
            
            % Vehicles' Annotation String
            speedArray = compose("%4.1f", [cat(1,obj.Vehicles.dynamics).speed]);
            nameArray = "V" + [obj.Vehicles.id];
            textDescArray = num2cell([nameArray; speedArray],1)';

            
            % Vehicles' Annotation Position
            allVehiclePositions = [allVehiclePositions(:,1)-8, -allVehiclePositions(:,3)+8, allVehiclePositions(:,2)+0.01];
            allTextPositions = mat2cell(allVehiclePositions,ones(1,size(allVehiclePositions,1)),3); % Matrix to Cell for the handle format
            
            %set the position and string handles
            set(obj.plots.carDescription,{'Position'},allTextPositions);
            set(obj.plots.carDescription,{'String'},textDescArray);
        end
            
        %% For debugging and analysis - not implemented
        function costs = getCosts(obj, point1, point2)
            index = ([obj.connections.translation(:,1);obj.connections.circle(:,1)] ==point1)&([obj.connections.translation(:,2);obj.connections.circle(:,2)] ==point2);
            costs_vector = [obj.connections.costs.translation obj.connections.costs.circle] ;
            costs = costs_vector(index);
        end
           
        function path = get_shortest_path(obj, starting_point, ending_point)
            
            [path,~] = shortestpath(obj.directedGraph,starting_point,ending_point);
            
        end
        
        function path_in_waypoints = find_shortest_path_as_waypoints(obj, starting_point, ending_point )
            
            [path,~] = shortestpath(obj.directedGraph,starting_point,ending_point);
            path_in_waypoints = obj.waypoints(path,:);
            
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

