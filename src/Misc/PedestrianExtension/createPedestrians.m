pmap = evalin('base','mapOc');
F=occupancyMatrix(pmap);
redmap=binaryOccupancyMap(F(1100:1400,1200:1500));
pd1 = Pedestrian([66 -159 0],[80,-105,0], 1,redmap);
pd2 = Pedestrian([70 -75 0],[79,-155,0], 1,redmap);
pd3 = Pedestrian([80 -150 0],[83,-80,0],1,redmap);

pds = [pd1 pd2 pd3];

hold on
Map.plots.Pedestrians = scatter(0,0,100,'filled');
allPedestrianPosition = cat(1,pds.position);

Map.plots.Pedestrians.XData = allPedestrianPosition(:,1);
Map.plots.Pedestrians.YData = allPedestrianPosition(:,2);
Map.peds=pds;

% newPosPD = allPedestrianPosition(:,1);
% 
% for i = 1:100
%     newPosPD = newPosPD + [0.05; 0.1; 0.15];
%     Map.plots.Pedestrians.YData = newPosPD;
%     pause(0.01);
% end
hold off

% figure(obj.map.fig2)
% show(obj.map.map3)