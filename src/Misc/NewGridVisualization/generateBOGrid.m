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
occ = ones(round(ySize*gRes,0),round(xSize*gRes,0));
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
        % y is shorter
        pdx = signDx;   %what to add for a parallel step
        pdy = 0;        %p is parallel
        ddx = signDx;   %what to add for a diagonal step
        ddy = signDy;   % d is diagonal
        deltaShortDirection  = absDy;
        deltaLongDirection  = absDx;
    else
        % x is shorter
        pdx = 0;        %what to add for a parallel step
        pdy = signDy;   % p is parallel
        ddx = signDx;   %what to add for a diagonal step
        ddy = signDy;   % d is diagonal
        deltaShortDirection  = absDx;
        deltaLongDirection  = absDy;
    end
    %start at node 1
    x = p1(1);
    y = p1(2);
    pixelArray = append(num2str(x), ",", num2str(y));
    error = deltaLongDirection/2;
    %% now set the pixel
    for i= 1:deltaLongDirection
        %% for each pixel
        % update error
        error = error - deltaShortDirection;
        if error < 0
            error = error + deltaLongDirection; % error is never < 0
            % go in long and short  direction
            x = x + ddx;
            y = y + ddy; % diagonal
        else
            % go in long direction
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

%% Helper functions
function bogMap = setPixelFromArray(map,bogMap,dist,pixelArray,speedLimit,startNodeNR,endNodeNR,nrOfCars,edgeNumber)
%get number of starting node and end node, to set unique
%successors and predecessors per different edge
%% set first GridLocation
curKey = pixelArray(1);
%calculate distance of every cell
dist = dist/size(pixelArray,2);
%load or create GridLocation for starting node
pStart = str2num(pixelArray(1));
if ~map.gridLocationMap.isKey(curKey)
    curGL = GridLocation(pStart,nrOfCars,startNodeNR,0);
else
    curGL = map.gridLocationMap(curKey);
end
%assign property
curGL = curGL.assignDistance(dist);
curGL.speedLimit = speedLimit;
p = str2num(pixelArray(1));
%set pixel
setOccupancy(bogMap,p,0,"grid");
%% set all GridLocation objects on the road and connect them
for s = 2 : (length(pixelArray)-1)
    %% start connecting
    % create new key
    newKey = pixelArray(s);
    % set pixel at location p
    p = str2num(newKey);
    setOccupancy(bogMap,p,0,"grid");
    %create new GL
    if ~map.gridLocationMap.isKey(newKey)
        newGL = GridLocation(p,nrOfCars,0,edgeNumber);
    else
        %or load old one
        newGL = map.gridLocationMap(newKey);
    end
    %% assign properties
    newGL = newGL.assignDistance(dist);
    curGL.speedLimit = speedLimit;
    %% assign successor and predecessor
    curGL = curGL.addTransSucc(newKey,startNodeNR);
    newGL = newGL.addTransPred(curKey,endNodeNR);
    %store in map
    map.gridLocationMap(curKey)=curGL;
    %% move to next GL
    curGL = newGL;
    oldKey = curKey;
    curKey = newKey;
end
%% set the last GL
%get key
newKey = pixelArray(end);
if ~map.gridLocationMap.isKey(newKey)
    %create new
    p = str2num(newKey);
    newGL = GridLocation(p,nrOfCars,endNodeNR,0);
else
    %load from map
    newGL = map.gridLocationMap(newKey);
end
%set inside bog
setOccupancy(bogMap,p,0,"grid");
p = str2num(oldKey);
setOccupancy(bogMap,p,0,"grid");
%assign properties
newGL = newGL.assignDistance(dist);
newGL.speedLimit = speedLimit;
curGL.speedLimit = speedLimit;
%assign succ and pred
curGL = curGL.addTransSucc(newKey,startNodeNR);
newGL = newGL.addTransPred(curKey,endNodeNR);
%store inside map
map.gridLocationMap(curKey)=curGL;
map.gridLocationMap(newKey)=newGL;
end


function pixelArray = calculateCircle(pStart,phiStart,pGoal,phiGoal,pCenter,direction,radius,offset)

%start with the first one
curPix = pStart;
%assign it to an array
pixelArray = [];%store points here
pixelArray = [pixelArray,append(num2str(curPix(1)), ",", num2str(curPix(2)))];
curPhi = phiStart;
while curPix(1) ~= pGoal(1) || curPix(2) ~= pGoal(2)
    %while the goal pixel is not reached, go to the next
    %pixel, that is between the last one and the goal and
    %is closest to the radius
    nextPix = [];   %the next pixel to draw in grid
    nextPhi = 0;
    deltaR = 200000;     %set up distance to the radius point with angle phi to be allways higher first try
    %for every neighbour
    for x = -1 : 1
        for y = -1 : 1
            if x ~= 0 || y ~= 0
                %compare pixel to get the closest one to the original
                %point
                %calculate the distance between the pixel and the
                %reference and use the closest
                neighbourPix = [curPix(1)+x,curPix(2)+y];%new pixel in grid
                phi = angle(complex((neighbourPix(1)-pCenter(1)) , (neighbourPix(2)-pCenter(2))));%angle in world
                if phi < 0
                    phi = phi + 2*3.1415;
                else
                    phi = phi + offset;
                end
                %test, if the angle is relevant (between last and goal)
                if (direction == 1 && curPhi <= phi && phiGoal >= phi)|| (direction ==-1 && curPhi >= phi && phiGoal <= phi)
                    %get point with same angle on the radius
                    referencePoint = [radius * cos(phi)+pCenter(1),radius*sin(phi)+pCenter(2)];
                    %calculate distance between reference and current neighbour pixel
                    refDeltaR = norm(neighbourPix - referencePoint);
                    if refDeltaR < deltaR
                        %if the distance is less then
                        %previously, we found a better next
                        %pixel
                        deltaR = refDeltaR;
                        nextPix = neighbourPix;
                        nextPhi = phi;
                    end
                end
            end
        end
    end
    %move to the best pixel
    curPix = nextPix;
    curPhi = nextPhi;
    %now curPix is the next pixel to draw
    pixelArray = [pixelArray, append(num2str(curPix(1)), ",", num2str(curPix(2)))];
end
end



