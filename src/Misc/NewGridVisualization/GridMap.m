classdef GridMap < Map
    %A MOBATSim map object with a XML-graph and a binary occupancy grid object
    %   Detailed explanation goes here
    
    properties
        bogMap;                        % Binary occupancy grid object
        gridResolution = 0.25;         % Number of cells per 1 length unit on the map
        gridLocationMap;               % Container object to store and load all GridLocation objects
        xOffset;                       % Offset to transform visualization into bog coordinates if necessary
        yOffset;
        colourMatrix;
    end
    
    methods
        %% constructor and visualization creation
        function obj = GridMap(mapName,waypoints, connections_circle,connections_translation, startingNodes, breakingNodes, stoppingNodes, leavingNodes)
            obj = obj@Map(mapName,waypoints, connections_circle,connections_translation, startingNodes, breakingNodes, stoppingNodes, leavingNodes);
            
            obj.colourMatrix = [1 0 0;      %1  red
                1 1 0 ;                     %2	yellow
                0 1 1 ;                     %3  light blue
                0 0.2470 0.5410;            %4  dark blue
                0.8500 0.3250 0.0980;       %5  orange
                1 0 1;                      %6  magenta
                0 0.4470 0.7410;            %7  blue
                0.6350 0.0780 0.1840;       %8  dark red
                0.3 0.3 0.3;                %9  grey
                0.4940 0.1840 0.5560;       %10 violet
                ];
            
                        
            % Plot the map on the figure
            generateMapVisual(obj,false);
            % BOG will be created in the prepare_simulator script, to include all vehicle data
            
            %create bog container map object
            obj.gridLocationMap = containers.Map();
            
            obj.PlotMap();
       
        end %Constructor
                
        function generateMapVisual(obj,displayInGridCoordinates)
            %This function plots any XML Map of MOBATSim. Keep in mind that you have to
            %do a coordinate transformation between normal coordinates and grid / mobatsim
            %Input: XML Map object of MOBATSim, boolean wether to plot mobatsim or grid coordinates

            %% prepare everything
            hold on            
            waypoints = obj.waypoints;
            circ = obj.connections.circle; %curves
            trans = obj.connections.translation; %straight roads
            %coordinate transformation
            waypoints(:,3) = -1.*waypoints(:,3);    %MOBATSim stores the negative y, so we have to transform it
            circ(:,6) = -1.*circ(:,6);
            %maybe it is necessary to display everything in the coordinate system of the binary occupancy grid object
            %if so, we have to shift everything here and set the input true
            if displayInGridCoordinates
                xOff = min(waypoints(:,1))-50;
                yOff = min(waypoints(:,3))-50;
                
                waypoints(:,3) = waypoints(:,3)-yOff;
                waypoints(:,1) = waypoints(:,1)-xOff;                
                
                circ(:,4) = circ(:,4)-xOff;
                circ(:,6) = circ(:,6)-yOff;
            end
            %% Generate a usable plot
            %% generate curves
            for c = 1 : length(circ)
                cPart = circ(c,:); %load information on the current curve
                %starting point
                x1 = waypoints(cPart(1),1); 
                y1 = waypoints(cPart(1),3);   
                %goal point
                x2 = waypoints(cPart(2),1); 
                y2 = waypoints(cPart(2),3);
                %central point
                x0W = cPart(4); 
                y0W = cPart(6);
                %radius
                radius = norm( [x2,y2]-[x0W,y0W] );
                %angles of start and goal
                phiStart = angle(complex((x1-x0W) , (y1-y0W)));
                phiGoal = angle(complex((x2-x0W) , (y2-y0W)));
                %direction
                direction = sign(cPart(3));
                %% make angle allways usable
                %we have to make every angle positive and also big eneough,
                %that an angle of a point is allways between start and goal
                if phiStart <0
                    phiStart = phiStart + 2*3.1415;
                end
                if phiGoal <0
                    phiGoal = phiGoal + 2*3.1415;
                end
                %make turns through 0° possible
                if (direction == -1 && phiStart < phiGoal)
                    phiStart = phiStart + 2*3.1415;
                end
                if (direction == 1 && phiStart > phiGoal)
                    phiGoal = phiGoal + 2*3.1415;
                end
                %create an array with all angles between start and goal
                phi1 = phiStart : direction*0.01 : phiGoal;
                %set first and last, to make shure there are no gaps in the
                %plot because of rounding errors
                phi1(1) = phiStart;
                phi1(end) = phiGoal;
                %create the points to plot from angle and radius
                points = [(radius .* cos(phi1)+x0W)',(radius .* sin(phi1))'+y0W];
                %plot it
                plot(points(:,1),points(:,2),'color',[0 1 0],'LineWidth',2);
                %plot number next to edge
                textPos = points(round(length(points)/2,0),:);                
                description = text(textPos(1)-10,textPos(2)-15,num2str(c),'color',[0 0.5 0]);
            end
            %% generate straight lines
            for t = 1 : length(trans)
                position = zeros(2,2); %preallocate start and goal point
                %get both points and plot a line in between
                position(1,:) = [waypoints(trans(t,1),1) ,waypoints(trans(t,1),3)];
                position(2,:) = [waypoints(trans(t,2),1) ,waypoints(trans(t,2),3)];
                plot(position(:,1),position(:,2),'color',[0 1 0],'LineWidth',2);
                %plot number next to edge
                textPos = (position(2,:) + position(1,:))/2;                
                text(textPos(1)+5,textPos(2)-15,num2str(t+c),'color',[0 0.5 0]);
            end
            %% plot nodes with numbers
            for n = 1 : length(waypoints)
                %get position
                pos = [waypoints(n,1),waypoints(n,3)];
                %plot dot and number in a dark blue
                plot(pos(1),pos(2),'Marker','o','MarkerFaceColor',[0 0.2 0.5],'color',[0 0.2 0.5]);
                text(pos(1)-5,pos(2)-15,num2str(n),'color',[0 0.2 0.5]);%TODO make it appear automatically under dot
                %maybe there is a way to let it not collide with other text?
            end
            hold off
        end
                
        function dynamicRouteHighlighting(obj)
            
            delete(obj.plots.trajectories)
            hold on
            for vehicle = obj.Vehicles % Changing sizes of BOGPath makes it hard to vectorize
                obj.plots.trajectories(vehicle.id) = plot(vehicle.pathInfo.BOGPath(:,1),vehicle.pathInfo.BOGPath(:,2),'color',obj.colourMatrix(vehicle.id,:),'LineWidth',2);
            end
            hold off
            
        end

        
    end
    
end

