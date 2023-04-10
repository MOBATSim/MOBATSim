function Vehicles = load_vehicles(startingPoints, destinationPoints, maxSpeeds, map)
    % Generation of vehicles
    
    % Contributors: Mustafa Saraoglu, Qihang Shi, Johannes Pintscher
    
    %% Names
    % To add a vehicle just add a new name to VehicleNames
    VehicleNames = [{'V1'} ...
                    {'V2'} ...
                    {'V3'} ...
                    {'V4'} ...
                    {'V5'} ...
                    {'V6'} ...
                    {'V7'} ...
                    {'V8'} ...
                    {'V9'} ...
                    {'V10'}];
    nrVehicles = length(VehicleNames);
    
    %% Dimensions
    % According to the Cuboid world standard vehicle dimensions
    size = [2 1.8 4.7]; % => [height, width, length] in meters
    sizes = repmat(size,nrVehicles,1);
    % To specifically change a vehicle's size:
    % sizes(vehicleId,:) = [2 1.8 10];
    
    %% Masses
    masses(1:nrVehicles) = 1800; % kilogram
    % To specifically change a vehicle's mass:
    % masses(vehicleId) = 5000;
    
    %% Data links
    dataLinksV2V = ones(nrVehicles,nrVehicles);
    dataLinksV2I = ones(1,nrVehicles);

    %% Starting points
    % add default starting point 2 for every vehicle where no starting point is given
    startingPoints(end+1:nrVehicles) = 2;
    
    %% Destination points
    % add default destination point 1 for every vehicle where no destination point is given
    destinationPoints(end+1:nrVehicles) = 1;
    
    %% Sensors
    frontSensorRange = 100.* ones(1,nrVehicles);
    AEBdistance = 15.* ones(1,nrVehicles);
    
    %% Dynamics
    % add default max speed 10 for every vehicle where no max speed is given
    maxSpeeds(end+1:nrVehicles) = 10;
    
    minDeceleration = -9.15.* ones(1,nrVehicles); % -9.15 m/s^2 dec

   
    %% Generate vehicles
    Vehicles = [];
    for j=1:nrVehicles
        % Generate vehicle
        NewVehicle = Vehicle(j,VehicleNames{j},startingPoints(j),destinationPoints(j),...
        maxSpeeds(j),sizes(j,:),dataLinksV2V(j,:),dataLinksV2I(j),masses(j),...
        frontSensorRange(j),AEBdistance(j),minDeceleration(j),map);
    
        % Add vehicle to array
        Vehicles = [Vehicles NewVehicle];  %#ok<AGROW>
    end
end