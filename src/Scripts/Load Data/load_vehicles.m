function Vehicles = load_vehicles(startingPoints, destinationPoints, maxSpeeds, startingTimes, simSpeed)
    
    % To add a vehicle just add a new name to VehicleNames
    VehicleNames = [{'V1 - Sally Carrera'} ...
                    {'V2 - Lightning McQueen'} ...
                    {'V3 - Luigi'} ...
                    {'V4 - Chick Hicks'} ...
                    {'V5 - Doc Hudson'} ...
                    {'V6 - Red'} ...
                    {'V7 - Fillmore'} ...
                    {'V8 - Guido'} ...
                    {'V9 - Mater'} ...
                    {'V10 - Sheriff'}];
    nrVehicles = length(VehicleNames);
    
    % According to the Cuboid world standard vehicle dimensions
    size = [2 1.8 4.7]; % => [height, width, length] in meters
    sizes = repmat(size,nrVehicles,1);
    % To specifically change a vehicle's size:
    % sizes(vehicleId,:) = [2 1.8 10];
    mass = 1800; % kilogram
    masses = repmat(mass,nrVehicles,1);
    % To specifically change a vehicle's mass:
    % masses(vehicleId) = 5000;
    dataLinksV2V = ones(nrVehicles,nrVehicles);
    dataLinksV2I = ones(1,nrVehicles);


    frontSensorRange = 100.* ones(1,nrVehicles);
    AEBdistance = 25.* ones(1,nrVehicles);
    minDeceleration = -9.15.* ones(1,nrVehicles); % -9.15 m/s^2 dec

    Vehicles = [];
    for j=1:nrVehicles
        % Generate vehicle
        NewVehicle = Vehicle(j,VehicleNames{j},startingPoints(j),destinationPoints(j),...
        startingTimes(j),maxSpeeds(j),sizes(j,:),dataLinksV2V(j,:),dataLinksV2I(j),masses(j),...
        simSpeed,frontSensorRange(j),AEBdistance(j),minDeceleration(j));
    
        % Add to vehicle array
        Vehicles = [Vehicles NewVehicle];
    end
end