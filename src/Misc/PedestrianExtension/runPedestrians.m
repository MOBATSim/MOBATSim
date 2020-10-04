hold on
pd1 = Pedestrian([100 20], 5);
pd2 = Pedestrian([100 60], 5);
pd3 = Pedestrian([100 100], 5);

pds = [pd1 pd2 pd3];

Map.plots.Pedestrians = scatter(0,0,150,'filled');
allPedestrianPosition = cat(1,pds.position);

Map.plots.Pedestrians.XData = allPedestrianPosition(:,1);
Map.plots.Pedestrians.YData = allPedestrianPosition(:,2);


newPosPD = allPedestrianPosition(:,1);

for i = 1:100
    newPosPD = newPosPD + [0.05; 0.1; 0.15];
    Map.plots.Pedestrians.XData = newPosPD;
    pause(0.01);
end
hold off