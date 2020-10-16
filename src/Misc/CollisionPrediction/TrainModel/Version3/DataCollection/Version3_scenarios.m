%% Vehicle starting parameters and constants

Vehicles =[];
VehicleNames = [{'V1'} {'V2'} {'V3'} {'V4'} {'V5'} {'V6'} {'V7'} {'V8'} {'V9'} {'V10'}];
sizes = [[9 12 16];[9 12 16];[9 12 16];[9 12 16]; [9 12 40]; [9 12 36]; [9 12 16]; [9 12 34]; [9 12 16]; [9 12 16]];
mass = [1800,1800,900,3000,900, 900,1000,1000,1000,1000,];
dataLinksV2V = ones(10,10);
dataLinksV2I = ones(1,length(VehicleNames));

isMonteCarlo = true; % default: false

%% Create a set of possible starting and destination points sets
startingTimes = [0 0 0 0 0 0 0 0 0 0];
SimTimeOut= '20';

%%%%%scenario 1: for crossway point 11,4,36,44,25  collect in Data.mat
P1 = [4 35 13 12];    %11A
a = P1(randi([1,length(P1)]));
P2 = [31 32 33];    %11B
b = P2(randi([1,length(P2)]));
P3 = [7 8];    %4A
c = P3(randi([1,length(P3)]));
P4 = [5 6];    %4B
d = P4(randi([1,length(P4)]));
P5 = [37 38];    %36A
e = P5(randi([1,length(P5)]));
f = 39;    %36B
P7 = [47 46 45];    %44A
g = P7(randi([1,length(P7)]));
P8 = [50 79];    %44B
h = P8(randi([1,length(P8)]));
P9 = [27 26];    %25A
i = P9(randi([1,length(P9)]));
j = 42;    %25B

  startingPoints =    [a  b   c   d   e   f   g  h  i  j];    
 destinationPoints =  [74  7  27  1  12  34  43 60 21 41];

%%%%%scenario 2: for crossway point 9,13,52,23      collect in Data1.mat
%for crossway point 9,select one of the groups
% % P1 = [31 32 11 10];    %9A1
% % a = P1(randi([1,length(P1)]));
% % P2 = [14 15 25 67];    %9B1
% % b = P2(randi([1,length(P2)]));
% P1 = [14 15];    %9A2
% a = P1(randi([1,length(P1)]));
% P2 = [55 67];    %9B2
% b = P2(randi([1,length(P2)]));
% 
% P3 = [37 38 39 40 41 36 34];    %13A
% c = P3(randi([1,length(P3)]));
% P4 = [7 8 9 6 5 4 35];    %13B
% d = P4(randi([1,length(P4)]));
% e = 53;    %52A
% P6 = [54 46];    %52B
% f = P6(randi([1,length(P6)]));
% P7 = [28 29 30];    %23A
% g = P7(randi([1,length(P7)]));
% P8 = [24 25 26 27 42];    %23B
% h = P8(randi([1,length(P8)]));
% 
%   startingPoints =    [a  b   c   d   e   f   g  h  47 50];    
%  destinationPoints = [74  7   10  11  51  58  41 21 43 60];

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
 test = [0 0 0 0 0];
 Speed = [0 0 0 0 0 0 0 0 0 0];
 DistancetoCrosswaypoint = [0 0 0 0 0 0 0 0 0 0];
 save test test
 save Speed Speed
 save DistancetoCrosswaypoint DistancetoCrosswaypoint
%  Collision13 = 0;           %enable for scenario 2
%  save Collision13 Collision13