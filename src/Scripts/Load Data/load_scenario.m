function [startingPoints, destinationPoints, maxSpeeds] = load_scenario(scenarioName)
    switch scenarioName
        case 'Urban City Traffic'
%% Vehicle 2 makes lane change on straight road + Vehicle 5 change planned Path
%             startingPoints =    [46 47 84 3  54 64 58 57 4 63];
%             destinationPoints = [54 78 82 26 3  42 27 25 26 47];
%             maxSpeeds =         [3  18  3 7  14 10 0.1 4 10 10];
%% Vehicle 2 makes lane change on curved road + Vehicle 5 change planned Path
%             startingPoints =    [46 29 84 3  54 64 58 57 4 63];
%             destinationPoints = [54 78 82 26 3  42 27 25 26 47];
%             maxSpeeds =         [3  18  3 7  14 10 0.1 4 10 10];
%% Road merging Vehicle Convoy Scenario + V5 stops and blocks the road merge causing a collision
%     startingPoints      = [50  46    15   3    54  52  6    7    47  30];
%     destinationPoints   = [13  17    20   11   25  41  18   31   15  39];
%     maxSpeeds           = [13  12.9  12   9.2  13  13  12.8 13.8  6  13];
%% Platoon control test for vehicle convoy         
%            startingPoints =    [51 47 48 3 55 53 6 7 22 45];
%            destinationPoints = [13 15 18 27 26 40 27 32 16 10];
%            maxSpeeds =         [13 13 11 9 13 15 12 14 4 13];
%% V3 hits V4 with high speed - front collision
%             startingPoints =    [46 47 7 27 85 52 64 57 15 63];
%             destinationPoints = [54 51 26 26  78  27 42 25 49 47];
%             maxSpeeds = [3 15 35 1 6 10 10 12 10 10];
%% V3 merges the road from left while V4 goes straight and collides
%            startingPoints =    [46 47 6 7 85 52 64 57 15 63];
%            destinationPoints = [54 51 27 26  27  78 42 25 49 47];
%            maxSpeeds = [3 15 10 5.55 6 10 10 12 10 10];
%% Complex crossroad scenario + V8 collides with V1
%             startingPoints =    [8  46 29 18 41 52 64 6 15 63];
%             destinationPoints = [26 47 51 1  3  27 42 25 76 46];
%             maxSpeeds = [20 15 10 10 14 20 10 12 10 10];           
%% Extra complex crossroad scenario
            startingPoints =    [8  46 29 18 53 52 65 6 15 64];
            destinationPoints = [59 47 51 1  3  27 42 25 60 46];
            maxSpeeds = [20 20 10 10 14 20 15 12 10 20];
%% Counterexample crossroad safety
%             startingPoints =    [18 72 84 3  21 65 79 19 4 20];
%             destinationPoints = [17 78 82 26 17  42 76 17 26 17];
%             maxSpeeds =         [3  8.7  3 7  14 10 0.1 4 10 10];
        case 'Carmageddon'
            startingPoints =    [46 53 54 51 44 9  65 27 30 6];
            destinationPoints = [25 64 24 16 10 59 42 10 11 24];
            maxSpeeds = [28 25 10 25 38 25 25 30 11 15];

        case 'Convoy Platoons'
            % road merging scenario
            startingPoints =    [51 47 48 3 55 53 6 7 22 45];
            destinationPoints = [13 15 18 11 26 40 17 32 16 10];
            maxSpeeds = [13 13 11 9 13 13 8 14 4 13];


    end
    
end

