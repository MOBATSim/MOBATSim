function pathPlot = plotPath(map,path,carID)
    %this function plots the current position and path of all
    %vehicles on the map
    %Input: map is the XML map object,
    %displayInGridCoordinates is a boolean: true = display grid, false = display normal plot
    %% prepare everything
    hold on      
    colourMatrix = [1 0 0;                      %1  red
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
    w = map.waypoints;
    w(:,3) = -1.*w(:,3);
    circ = map.connections.circle;
    circ(:,6) = -1.*circ(:,6);
    trans = map.connections.translation;
    
    %use offset if you need it
    xOff = 0;
    yOff = 0;
    
    %% plot path for every vehicle
    pathPlot = [];
    bogPath = [];%list of points to plot
    %determine if circle or not
    for k = 2 : size(path,2)
        p1 = path(k-1);
        p2 = path(k);
        straight = ~isempty(trans(trans(:,1) == p1 & trans(:,2) == p2,:));
        if straight
            %just draw a line
            if isempty(bogPath)
                bogPath = [[map.Vehicles(carID).dynamics.position(1)-xOff,-map.Vehicles(carID).dynamics.position(3)-yOff]; [w(p2,1)-xOff,w(p2,3)-yOff]];
            else
                bogPath = [bogPath;[w(p1,1)-xOff,w(p1,3)-yOff]; [w(p2,1)-xOff,w(p2,3)-yOff]];
            end
        else
            %gather points on the circle
            rad = circ( circ(:,1)==p1 & circ(:,2) == p2,:);
            if isempty(bogPath)
                x1 = map.Vehicles(carID).dynamics.position(1)-xOff;
                y1 = -map.Vehicles(carID).dynamics.position(3)-yOff;
            else
                x1 = w(rad(1),1)-xOff; %start
                y1 = w(rad(1),3)-yOff;
            end
            x2 = w(rad(2),1)-xOff; %goal
            y2 = w(rad(2),3)-yOff;
            x0W = rad(4)-xOff; %central point
            y0W = rad(6)-yOff;
            radius = norm( [x2,y2]-[x0W,y0W] );
            phiStart = angle(complex((x1-x0W) , (y1-y0W)));
            phiGoal = angle(complex((x2-x0W) , (y2-y0W)));
            direction = sign(rad(3));
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
            phi1 = phiStart : direction*0.1 : phiGoal;
            phi1(1) = phiStart;
            phi1(end) = phiGoal;
            points = [(radius .* cos(phi1)+x0W)',(radius .* sin(phi1))'+y0W];
            bogPath = [bogPath;points];
        end

    end
    if ~isempty(bogPath)
        switch carID
            case 1
                %red
                pathPlot = plot(bogPath(:,1),bogPath(:,2),'color',colourMatrix(1,:),'LineWidth',2);
                %plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[1 0 0],'color',contrastArray);
            case 2
                %yellow
                pathPlot = plot(bogPath(:,1),bogPath(:,2),'color',colourMatrix(2,:),'LineWidth',2);
                %plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[1 1 0],'color',contrastArray);
            case 3
                %light blue
                pathPlot = plot(bogPath(:,1),bogPath(:,2),'color',colourMatrix(3,:),'LineWidth',2);
                %plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[0 1 1],'color',contrastArray);
            case 4
                %green
                pathPlot = plot(bogPath(:,1),bogPath(:,2),'color',colourMatrix(4,:),'LineWidth',2);
                %plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[0.4660 0.6740 0.1880],'color',contrastArray);
            case 5
                %orange
                pathPlot = plot(bogPath(:,1),bogPath(:,2),'color',colourMatrix(5,:),'LineWidth',2);
                %plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[0.8500 0.3250 0.0980],'color',contrastArray);
            case 6
                %magenta
                pathPlot = plot(bogPath(:,1),bogPath(:,2),'color',colourMatrix(6,:),'LineWidth',2);
                %plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[1 0 1],'color',contrastArray);
            case 7
                %blue
                pathPlot = plot(bogPath(:,1),bogPath(:,2),'color',colourMatrix(7,:),'LineWidth',2);
                %plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[0 0.4470 0.7410],'color',contrastArray);
            case 8
                %dark red
                pathPlot = plot(bogPath(:,1),bogPath(:,2),'color',colourMatrix(8,:),'LineWidth',2);
                %plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[0.6350 0.0780 0.1840],'color',contrastArray);
            case 9
                %light green
                pathPlot = plot(bogPath(:,1),bogPath(:,2),'color',colourMatrix(9,:),'LineWidth',2);
                %plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[0.9290 0.6940 0.1250],'color',contrastArray);
            otherwise
                %violet
                pathPlot = plot(bogPath(:,1),bogPath(:,2),'color',colourMatrix(10,:),'LineWidth',2);
                %plot(map.Vehicles(c).dynamics.position(1)-xOff,-map.Vehicles(c).dynamics.position(3)-yOff,'Marker','o','MarkerFaceColor',[0.4940 0.1840 0.5560],'color',contrastArray);
        end
        %make it transparent to see better
        %pathPlot.Color(4) = 0.60;
        %text(map.Vehicles(c).dynamics.position(1)-xOff-15,-map.Vehicles(c).dynamics.position(3)-yOff+20,append("V",num2str(c)),'color',contrastArray);
    end
    hold off
end