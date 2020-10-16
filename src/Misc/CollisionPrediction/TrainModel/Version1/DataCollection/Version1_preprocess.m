%% Create tables of data from vehicles

% Data = cell2table(cell(0,13), 'VariableNames', {'maxSpeed_A', 'Speed_A','DistancetoCrosswaypoint_A',  'frontSensorRange_A', 'AEBdistance_A', 'minDeceleration_A','maxSpeed_B','Speed_B','DistancetoCrosswaypoint_B', 'frontSensorRange_B', 'AEBdistance_B', 'minDeceleration_B','collision'});


load('idx.mat')
load('Data0.mat')
load('Data.mat')

for j = 1:(idx - 1)
    
    i = Data0.A(j);
    k = Data0.B(j);
    
    Data0.collision(j) = Vehicles(i).status.collided && Vehicles(k).status.collided ;
end

Data0 = removevars(Data0,{'A','B'});
Data = [Data;Data0];

save('Data.mat','Data');

