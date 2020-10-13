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
    %assign succ and pred
    curGL = curGL.addTransSucc(newKey,startNodeNR);
    newGL = newGL.addTransPred(curKey,endNodeNR);
    %store inside map
    map.gridLocationMap(curKey)=curGL;
    map.gridLocationMap(newKey)=newGL;
end

