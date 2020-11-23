Vehicles =[];
VehicleNames = [{'V1'} {'V2'} {'V3'} {'V4'} {'V5'} {'V6'} {'V7'} {'V8'} {'V9'} {'V10'}];
sizes = [[9 12 16];[9 12 16];[9 12 16];[9 12 16]; [9 12 40]; [9 12 36]; [9 12 16]; [9 12 34]; [9 12 16]; [9 12 16]];
mass = [1800,3000,900,3000,900, 900,1000,1000,1000,1000,];
dataLinksV2V = ones(10,10);
dataLinksV2I = ones(1,length(VehicleNames));


frontSensorRange = 100.* ones(1,length(VehicleNames));
AEBdistance = 25.* ones(1,length(VehicleNames));
minDeceleration = -40.* ones(1,length(VehicleNames)); % -9.15 m/s^2 dec




for j=1:length(VehicleNames)
    
    VehicleVariable = strcat('Vehicle',num2str(j));
    assignin('caller',VehicleVariable,Vehicle(j,VehicleNames{j},startingPoints(j),destinationPoints(j),...
    startingTimes(j),maxSpeeds(j),sizes(j,:),dataLinksV2V(j,:),dataLinksV2I(j),mass(j),...
    simSpeed,frontSensorRange(j),AEBdistance(j),minDeceleration(j)) );
    NewVehicle = evalin('base',strcat('Vehicle',int2str(j)));
    Vehicles =[Vehicles NewVehicle];
end