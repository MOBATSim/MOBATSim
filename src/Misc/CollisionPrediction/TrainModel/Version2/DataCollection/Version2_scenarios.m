%% Vehicle starting parameters and constants

Vehicles =[];
VehicleNames = [{'V1'} {'V2'} {'V3'} {'V4'} {'V5'} {'V6'} {'V7'} {'V8'} {'V9'} {'V10'}];
sizes = [[9 12 16];[9 12 16];[9 12 16];[9 12 16]; [9 12 40]; [9 12 36]; [9 12 16]; [9 12 34]; [9 12 16]; [9 12 16]];
mass = [1800,3000,900,3000,900, 900,1000,1000,1000,1000,];
dataLinksV2V = ones(10,10);
dataLinksV2I = ones(1,length(VehicleNames));

isMonteCarlo = true; % default: false

%% Create a set of possible starting and destination points sets
startingTimes = [0 0 0 0 0 0 0 0 0 0];
SimTimeOut= '20';

AllSets = [1:78];
OnlyStartingSet = [30 38 6 53 29 28 37 5];
OnlyDestinationSet = [16 1 2 59 60];
crossroadPointsSet = [56 72 73 65 51 44 70 71 74 75 68 62 49 66 77 43 69 50 67 63 42];

RemoveFromStarting = intersect(AllSets,[OnlyDestinationSet crossroadPointsSet]);
StartingSet = setxor(AllSets,RemoveFromStarting);

RemoveFromDestination = intersect(AllSets,[OnlyStartingSet crossroadPointsSet]);
DestinationSet = setxor(AllSets,RemoveFromDestination);

conflict = 1;
%%
while conflict~=0
rng('shuffle')
StartingIdx = randperm(length(StartingSet),10);
startingPoints = StartingSet(StartingIdx);% randomly pick from a set
rng('shuffle')
DestinationIdx = randperm(length(DestinationSet),10);
destinationPoints = DestinationSet(DestinationIdx);% randomly pick from a set

% Check if any vehicle's starting and destination points are the same
[~,ia,ib] = intersect(destinationPoints,startingPoints);

conflict = nnz(ia == ib);
assignin('base','startingPoints',startingPoints);
assignin('base','destinationPoints',destinationPoints);
end

%%
if isMonteCarlo
    rng('shuffle')
    
  maxSpeeds = ((25-2).*rand(10,1) + 5)'; % Between 2 and 25
    frontSensorRange = ((120-20).*rand(10,1) + 40)'; % Between 20 and 120
    AEBdistance = ((40-5).*rand(10,1) + 10)'; % Between 5 and 40
    minDeceleration = -((60-5).*rand(10,1) + 10)'; % Between -5 and -60

else
    frontSensorRange = 100.* ones(1,length(VehicleNames));
    AEBdistance = 25.* ones(1,length(VehicleNames));
    minDeceleration = -40.* ones(1,length(VehicleNames)); % -9.15 m/s^2 dec
end


for j=1:length(VehicleNames)
    
    VehicleVariable = strcat('Vehicle',num2str(j));
    assignin('caller',VehicleVariable,Vehicle(j,VehicleNames{j},startingPoints(j),destinationPoints(j),...
    startingTimes(j),maxSpeeds(j),sizes(j,:),dataLinksV2V(j,:),dataLinksV2I(j),mass(j),...
    simSpeed,frontSensorRange(j),AEBdistance(j),minDeceleration(j)) );
    NewVehicle = evalin('base',strcat('Vehicle',int2str(j)));
    Vehicles =[Vehicles NewVehicle];
end
%%
 t = 0;
 idx = 1;
 Data0 = cell2table(cell(0,15), 'VariableNames', {'maxSpeed_A', 'Speed_A','DistancetoCrosswaypoint_A',  'frontSensorRange_A', 'AEBdistance_A', 'minDeceleration_A','maxSpeed_B','Speed_B','DistancetoCrosswaypoint_B', 'frontSensorRange_B', 'AEBdistance_B', 'minDeceleration_B','A','B','collision'});
 save t t
 save Data0 Data0
 save idx idx
