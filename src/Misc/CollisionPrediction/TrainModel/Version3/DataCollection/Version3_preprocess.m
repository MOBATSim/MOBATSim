%% Create tables of data from vehicles
% Data  = cell2table(cell(0,13), 'VariableNames', {'maxSpeed_A', 'Speed_A','DistancetoCrosswaypoint_A','frontSensorRange_A', 'AEBdistance_A', 'minDeceleration_A','maxSpeed_B','Speed_B','DistancetoCrosswaypoint_B','frontSensorRange_B', 'AEBdistance_B', 'minDeceleration_B', 'collision'});

load('Data.mat');
load('Speed');
load('DistancetoCrosswaypoint');

idx = height(Data);

for i=1:2:9
    if Speed(i) ~= 0 && DistancetoCrosswaypoint(i) ~= 0 && Speed(i+1) ~= 0 && DistancetoCrosswaypoint(i+1) ~= 0
        
        maxSpeed_A = Vehicles(i).dynamics.maxSpeed;
        frontSensorRange_A = Vehicles(i).sensors.frontSensorRange;
        AEBdistance_A = Vehicles(i).sensors.AEBdistance;
        minDeceleration_A = Vehicles(i).dynamics.minDeceleration;
        Speed_A = Speed(i);
        DistancetoCrosswaypoint_A = DistancetoCrosswaypoint(i);
        maxSpeed_B = Vehicles(i+1).dynamics.maxSpeed;
        frontSensorRange_B = Vehicles(i+1).sensors.frontSensorRange;
        AEBdistance_B = Vehicles(i+1).sensors.AEBdistance;
        minDeceleration_B = Vehicles(i+1).dynamics.minDeceleration;
        Speed_B = Speed(i+1);
        DistancetoCrosswaypoint_B = DistancetoCrosswaypoint(i+1);
        
        %%%%%%%%%  for scenario 2
%         if i ~= 3
%             collision = Vehicles(i).status.collided && Vehicles(i+1).status.collided ;
%         else
%             collision = Collision13;
%         end
        %%%%%%%%
        
       %%%%%%%% for scenario 1
        collision = Vehicles(i).status.collided && Vehicles(i+1).status.collided ;
       %%%%%%%%%%%% 
        
        Data(idx + 1,:) = {maxSpeed_A,Speed_A,DistancetoCrosswaypoint_A,frontSensorRange_A,AEBdistance_A,minDeceleration_A,...
            maxSpeed_B,Speed_B,DistancetoCrosswaypoint_B,frontSensorRange_B,AEBdistance_B,minDeceleration_B,collision} ;
        idx = idx +1;
    end
end

save('Data.mat','Data');

