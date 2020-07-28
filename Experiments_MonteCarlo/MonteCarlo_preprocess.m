%% Create tables of data from vehicles

% if exist('T','var')
%     clear Map % Default map selection
% end
%T  = cell2table(cell(0,8), 'VariableNames', {'maxSpeed', 'startingPoint','currentRoute', 'destinationPoint', 'frontSensorRange', 'AEBdistance', 'minDeceleration', 'collision'});

if ~contains(pwd,'Experiments_MonteCarlo')
    cd('.\Experiments_MonteCarlo')
end


load('test1.mat')

idx = height(T);
for j=1:length(Vehicles)
    
    maxSpeed = Vehicles(j).dynamics.maxSpeed;
    startingPoint = Vehicles(j).pathInfo.startingPoint;
    currentRoute = Vehicles(j).pathInfo.currentRoute;
    destinationPoint = Vehicles(j).pathInfo.destinationPoint;
    frontSensorRange = Vehicles(j).sensors.frontSensorRange;
    AEBdistance = Vehicles(j).sensors.AEBdistance;
    minDeceleration = Vehicles(j).dynamics.minDeceleration;
    collision = Vehicles(j).status.collided;
    
    T(idx + j,:) = {maxSpeed,startingPoint,currentRoute,destinationPoint,frontSensorRange,AEBdistance,minDeceleration,collision};
    
    
end


%name = strcat('test',num2str(ExpNo));
%filename=strcat(name,'.mat');
save('test1.mat','T');

cd ../