
pd1 = Pedestrian([73 -155], 1,1);
pd2 = Pedestrian([75 -75], 1,-1);
pd3 = Pedestrian([77 -155],1, 1);

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