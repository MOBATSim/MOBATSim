function [bogMap,xOff,yOff] = generateBOGrid(map)
            %create binary occupancy grid object and grid location objects
            %all cells are drawn like pixel and connect to each other 
            %Input: XML map object
            %Output: binary occupancy map object, x-offset, y-offset
            
            %% prepare for drawing
            %get the number of cars to set up the vectors inside GridLocation
            nrOfCars = length([map.Vehicles.id]);
            %get the gridsize (the number of cells for 1 unit on the map)
            gRes = map.gridResolution;
            %first we need map size
            %we can get it from waypoints with some space for better
            %display
            distances = map.connections.distances;
            waypoints = map.waypoints;  %[x z y]
            waypoints(:,3) = -1.*waypoints(:,3);%transform to mobatsim coordinates
            xSize = max(waypoints(:,1))-min(waypoints(:,1))+100;
            xOff = min(waypoints(:,1))-50;            
            ySize = max(waypoints(:,3))-min(waypoints(:,3))+100;
            yOff = min(waypoints(:,3))-50;
            %calculate speedLimit [carID,edgeNR]
            maxEdgeSpeed = [map.connections.circle(:,end);
                            map.connections.translation(:,end)];
            %set speed limit to equal or less than max vehicle speed
            speedLimit = zeros(nrOfCars,length(maxEdgeSpeed));
            for car = 1 : nrOfCars
                maxSpeed = map.Vehicles(car).dynamics.maxSpeed ;
                maxEdgeSpeed(maxEdgeSpeed>maxSpeed)= maxSpeed;
                speedLimit(car,:) = maxEdgeSpeed;
            end 
            %create binary occupancy map object
            bogMap = binaryOccupancyMap(xSize,ySize,gRes);
            %mark everything as blocked
            occ = ones(ySize*gRes,xSize*gRes);
            setOccupancy(bogMap,[0,0],occ);%lower left corner to set values
            
            %import map details
            trans = map.connections.translation; %[from, to, speed]
            circ = map.connections.circle; %[from, to, angle, x,z,yCenter, speed]
            circ(:,6) = -1.*circ(:,6);     %transform to mobatsim coordinates     
            
            %we iterate backwards in case we want to preallocate something
            
            %% bresenham algorithm
            for j = size(map.connections.all,1) :-1: (size(circ,1)+1)
                %% for each straight edge
                %conncetions.all stores first all circles, then all translations
                %   j = nr of edge globally inside conncetions.all
                %   t = nr inside the vector trans
                t= j - size(circ,1);
                dist = distances(j);
                %start node coordinates in grid
                p1 = bogMap.world2grid([waypoints(trans(t,1),1)-xOff,waypoints(trans(t,1),3)-yOff]);
                %end node coordinates in grid
                p2 = bogMap.world2grid([waypoints(trans(t,2),1)-xOff,waypoints(trans(t,2),3)-yOff]);
                %get difference
                deltaX = p2(1)-p1(1);
                delatY = p2(2)-p1(2);
                %get distance and direction
                absDx = abs(deltaX);
                absDy = abs(delatY); % distance
                signDx = sign(deltaX);
                signDy = sign(delatY); % direction
                %determine which direction to go
                if absDx > absDy
                    % x is shorter
                    pdx = signDx;
                    pdy = 0;        %p is parallel
                    ddx = signDx;
                    ddy = signDy; % d is diagonal
                    deltaLongDirection  = absDy;
                    deltaShortDirection  = absDx;
                else
                    % y is shorter
                    pdx = 0;
                    pdy = signDy; % p is parallel
                    ddx = signDx;
                    ddy = signDy; % d is diagonal
                    deltaLongDirection  = absDx;
                    deltaShortDirection  = absDy;
                end
                %start at node 1
                x = p1(1);
                y = p1(2);
                pixelArray = append(num2str(x), ",", num2str(y));                
                error = deltaShortDirection/2;                
                %% now set the pixel
                for i= 1:deltaShortDirection          
                    %% for each pixel
                    % update error
                    error = error - deltaLongDirection;
                    if error < 0
                        error = error + deltaShortDirection; % error is never < 0
                        % go in long direction
                        x = x + ddx;
                        y = y + ddy; % diagonal                        
                    else
                        % go in short direction
                        x = x + pdx;
                        y = y + pdy; % parallel
                    end                    
                    curKey = append(num2str(x), ",", num2str(y));
                    pixelArray = [pixelArray,curKey];
                end 
                %% now assign properties
                bogMap = setPixelFromArray(map,bogMap,dist,pixelArray,speedLimit(:,j),map.connections.all(j,1),map.connections.all(j,2),nrOfCars,j);
            end
                        
            %% draw a circle pixel by pixel
            for t = size(circ,1):-1:1
                % t = nr of edge
                %% load information
                dist = distances(t);
                %starting point
                pStart = bogMap.world2grid([waypoints(circ(t,1),1)-xOff,waypoints(circ(t,1),3)-yOff]);
                %goal point
                pGoal = bogMap.world2grid([waypoints(circ(t,2),1)-xOff,waypoints(circ(t,2),3)-yOff]);
                %central point
                pCenter = bogMap.world2grid([circ(t,4)-xOff,circ(t,6)-yOff]); 
                
                radius = round(norm( pGoal-pCenter  ),0);
                phiStart = angle(complex((pStart(1)-pCenter(1)) , (pStart(2)-pCenter(2))));
                phiGoal = angle(complex((pGoal(1)-pCenter(1)) , (pGoal(2)-pCenter(2))));
                direction = sign(circ(t,3));                
                % make angle allways usable
                if phiStart <0
                    phiStart = phiStart + 2*3.1415;
                end
                if phiGoal <0
                    phiGoal = phiGoal + 2*3.1415;
                end
                offset = 0;
                %make turns through 0° possible
                if (direction == -1 && phiStart < phiGoal)
                    offset = 2*3.1415;
                    phiStart = phiStart + offset;
                end
                if (direction == 1 && phiStart > phiGoal)
                    offset = 2*3.1415;
                    phiGoal = phiGoal + offset;
                end                
                %% calculate all pixel
                pixelArray = calculateCircle(pStart,phiStart,pGoal,phiGoal,pCenter,direction,radius,offset);
                %% now draw pixel and assign to map
                bogMap = setPixelFromArray(map,bogMap,dist,pixelArray,speedLimit(:,t),map.connections.all(t,1),map.connections.all(t,2),nrOfCars,t);
            end
        end
