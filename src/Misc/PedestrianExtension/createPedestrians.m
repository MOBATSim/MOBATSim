pmap = evalin('base','mapOc');
F=occupancyMatrix(pmap);
% redmap=binaryOccupancyMap(F(1100:1400,1200:1500));
redmap2=occupancyMap(F(1142:1323,1330:1371));
mmap=copy(redmap2);

% pd1 = Pedestrian([66 -159 0],[80,-105,0], 1,redmap);
% pd2 = Pedestrian([70 -75 0],[79,-155,0], 1,redmap);
% pd3 = Pedestrian([80 -150 0],[83,-80,0],1,redmap);
pd1 = Pedestrian([78 -157 0],[76,-81,0], 1,0.5,redmap2);
pd2 = Pedestrian([74 -75.5 0],[74,-155,0], -1,0.2,redmap2);
pd3 = Pedestrian([72 -150 0],[75.5,-80,0],1,0.5,redmap2);

pd4 = Pedestrian([75 -158 0],[78,-81,0], 1,0.5,redmap2);
pd5 = Pedestrian([73 -77.5 0],[72,-156,0], -1,0.2,redmap2);
pd6 = Pedestrian([76 -110 0],[76.5,-79,0],1,0.5,redmap2);

pd7 = Pedestrian([77.5 -115 0],[76.5,-78,0], 1,0.5,redmap2);
pd8 = Pedestrian([74.5 -120 0],[72,-153,0], -1,0.2,redmap2);
pd9 = Pedestrian([75.5 -78.5 0],[76,-150,0], -1,0.2,redmap2);

pds = [pd1 pd2 pd3 pd4 pd5 pd6 pd7 pd8 pd9];

hold on
Map.plots.Pedestrians = scatter(0,0,100,'filled');
allPedestrianPosition = cat(1,pds.position);

Map.plots.Pedestrians.XData = allPedestrianPosition(:,1);
Map.plots.Pedestrians.YData = allPedestrianPosition(:,2);
Map.peds=pds;

for i=1:size(pds,2)    
    setOccupancy(mmap,[pds(i).positionLocal],[pds(i).col],"local")
end
Map.mapCrosswalkEmpty=redmap2
Map.mapCrosswalkFull=mmap

hold off
