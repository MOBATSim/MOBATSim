function [startingPoints, destinationPoints, maxSpeeds] = load_scenario(scenarioName)

% Contributors: Mustafa Saraoglu, Qianwei Yang, Qihang Shi, Johannes Pintscher

switch scenarioName
    case 'Urban City Traffic'
        % Vehicle 2 makes lane change on straight road + Vehicle 5 change planned Path
        startingPoints =    [46 47 84 3  54 64 58 57 4 63];
        destinationPoints = [54 51 82 26 3  24 27 25 26 47];
        maxSpeeds =         [3  18  3 7  14 10 0.1 4 10 10];
    case 'Curved Road Overtaking'
        % Vehicle 2 makes lane change on curved road + Vehicle 5 change planned Path
        startingPoints =    [46 29 84 3  54 64 58 57 4 63];
        destinationPoints = [54 24 82 26 3  24 27 25 26 47];
        maxSpeeds =         [3  18  3 7  14 10 0.1 4 10 10];
    case 'Merge Collision'
        % Road merging Vehicle Convoy Scenario + V5 stops and blocks the road merge causing a collision
        startingPoints      = [50  46    15   3    54  52  6    7    47  30];
        destinationPoints   = [13  17    20   11   25  41  18   31   15  39];
        maxSpeeds           = [13  12.9  12   9.2  13  13  12.8 13.8  6  13];
    case 'Platoon Control'
        % Platoon control test for vehicle convoy
        startingPoints =    [51 47 48 3 55 53 6 7 22 45];
        destinationPoints = [13 15 18 27 26 40 27 32 16 10];
        maxSpeeds =         [13 13 11 9 13 15 12 14 4 13];
    case 'High Speed Collision'
        % V3 hits V4 with high speed - front collision
        startingPoints =    [46 47 7  27 85 52 64 57 15 63];
        destinationPoints = [54 51 26 26 23 27 24 25 59 47];
        maxSpeeds =         [3  15 35 1  6  10 10 12 10 10];
    case 'Collision Merge'
        % V3 merges the road from left while V4 goes straight and collides
        startingPoints =     [46 47 6  7  85 52 64 57 15 63];
        destinationPoints =  [54 51 27 26 27 23 24 25 59 47];
        maxSpeeds =          [3 15 10 5.55 6 10 10 12 10 10];
    case 'Complex Crossroad'
        % Complex crossroad scenario + V8 collides with V1
        startingPoints =    [8  46 29 18 41 52 64 6 15 63];
        destinationPoints = [26 47 51 1  3  27 24 25 7 46];
        maxSpeeds = [20 15 10 10 14 20 10 12 10 10];
    case 'Extra Complex Crossroad'
        % Extra complex crossroad scenario
        startingPoints =    [8  46 29 18 53 52 65 6 15 64];
        destinationPoints = [59 47 51 1  3  27 24 25 60 46];
        maxSpeeds = [20 20 10 10 14 20 15 12 10 20];
    case 'Counterexample Crossroad Safety'
        % Counterexample crossroad safety
        startingPoints =    [18 72 84 3  21 65 79 19 4 20];
        destinationPoints = [17 51 82 26 17  24 60 17 26 17];
        maxSpeeds =         [3  8.7  3 7  14 10 0.1 4 10 10];
    case 'Mayhem'
        startingPoints =    [46 53 54 51 44 9  65 27 30 6];
        destinationPoints = [25 64 24 16 10 59 23 10 11 24];
        maxSpeeds = [28 25 10 25 38 25 25 30 11 15];
    case 'Convoy Platoons'
        % road merging scenario
        startingPoints =    [51 47 48 3 55 53 6 7 22 45];
        destinationPoints = [13 15 18 11 26 40 17 32 16 10];
        maxSpeeds = [13 13 11 9 13 13 8 14 4 13];
    case 'Vehicle Safety Filter Test'
        startingPoints =    [46 85 47 3  54 64 58 57 4 63];
        destinationPoints = [52 51 51 26 3  24 27 25 26 47];
        maxSpeeds =         [5  18 20 7  14 10 0.1 4 10 10];
        
    case 'Scenario 13'
        startingPoints =    [52 20 4 58 48 13 45 35 38 26];
        destinationPoints = [61 25 2 10 12 45 47 54 34 11];
        maxSpeeds =         [16 16 15 7 19 9 7 19 7 13];
        
    case 'Scenario 14'
        startingPoints =    [6  13 48  8 46 55 64 15 25 11];
        destinationPoints = [47 14 32 48 59 26 20 55  2 64];
        maxSpeeds =         [16 16 15 7  19 9  7  19  7 13];
        
    case 'Scenario 15'
        startingPoints =    [39  64  19   8  28  24  32  25  36  22];
        destinationPoints = [55  17  35  24  20  12  59  54  58  31];
        maxSpeeds =         [19  20   6  17   5  18   9   9  14  17];
        
    case 'Scenario 16'
        startingPoints =    [27  20  14  29  37  25  61  11  39  33];
        destinationPoints = [33   9  23  27  25  46   4  47  61  19];
        maxSpeeds =         [11  12   5  10  16   8  10   8  20   8];
        
    case 'Scenario 17'
        startingPoints =    [35  24  27  22   9  53   4  19  30  21];
        destinationPoints = [23  33   9  58  61  46  32  54  59  26];
        maxSpeeds =         [17  16  17   6   5  12   9  12   6  16];
        
    case 'Scenario 18'
        startingPoints =    [13  17   6  55  29  57  18  54  22  53];
        destinationPoints = [19  40  58  60  25  10  52   4  41  17];
        maxSpeeds =         [18  11  13  14  18   9  14  19  10   8];
        
    case 'Scenario 19'
        startingPoints =    [57  25  21  19  11  10   3  54  15   9];
        destinationPoints = [35  32  23  25  54  33  47  26  57  60];
        maxSpeeds =         [20  15   7   8  16  17   5   6  13  18];
        
    case 'Scenario 20'
        startingPoints =    [35  37  14  10  58   4  12  19  38  53];
        destinationPoints = [33  54  24  64  60  25  46  36  22  27];
        maxSpeeds =         [ 7  13  10  20  14   5   7  12  19  19];
        
    case 'Scenario 21'
        startingPoints =    [31  38  18  47  52  64  20   7  61  48];
        destinationPoints = [25   3  34  36  58  18  57  19   8  46];
        maxSpeeds =         [11  14  18  13  14  14  20   8  10   7];
        
    case 'Scenario 22'
        startingPoints =    [23   8  12   4  47  25  61  13  29  27];
        destinationPoints = [7  45  61  39  55   9  59  64  14   1];
        maxSpeeds =         [6   8  10  19   9   5  16   8  12   6];
        
    case 'Scenario 23'
        startingPoints =    [21   3  35  45  36  37   6  40   7  39];
        destinationPoints = [52  33  54  36  31  15  32  48  14  24];
        maxSpeeds =         [7   5  10   6  19  16   5  13  11  19];
        
    case 'Scenario 24'
        startingPoints =    [58  28  39  31  11  10  13  25  61  23];
        destinationPoints = [36  59  16  64  20  55  23  45  31  61];
        maxSpeeds =         [15  11  20  12  15  16  18  20   8  16];
        
        
    case 'Scenario 25'
        startingPoints =    [61  40   7  38  37  48  52  26  14  32];
        destinationPoints = [59  19  47   9  58   2  15  16  36  33];
        maxSpeeds =         [15  14  11  20  19  17  11   5   8  13];
        
    case 'Scenario 26'
        startingPoints =    [41   6  40  32  11  64  28  47  20  53];
        destinationPoints = [14  33  60  59  27   3  19  26  41  25];
        maxSpeeds =         [12  12  16  16  19   5  19  20   8  11];
        
    case 'Scenario 27'
        startingPoints =    [36  39  47  61  35  30  46  28  55  29];
        destinationPoints = [19  41  17  34  15  40  52  58  21  60];
        maxSpeeds =         [13  15  15   5  11  20  12   5  10  20];
        
    case 'Scenario 28'
        startingPoints =    [15  18  23  25  47  45  17  10  37  30];
        destinationPoints = [17  23  33  41  11  19  13  21  47  52];
        maxSpeeds =         [15   6   7   6  11  10  19  17  15   6];
        
    case 'Scenario 29'
        startingPoints =    [5  15  58  26   3   7  47  30  52   9];
        destinationPoints = [79 34  45  59   4  27  57  47   9  40];
        maxSpeeds =         [18  7   6  19  16  11  13   7  17   7];
        
    case 'Scenario 30'
        startingPoints =    [17 41  40  25  39  48  24  20  28  52];
        destinationPoints = [4  15  26  46  23  21  59  36  58  54];
        maxSpeeds =         [10 20  20   5  12  17  13  14  11  12];
        
end

end

