function [Route_LaneNumber, waypoints, connections_circle, connections_translation, ...
          startingNodes, breakingNodes, stoppingNodes, leavingNodes] = load_Crossmap()
    %% Map Data for CrossMap

    waypoints = [[10 0 -50]
        [10 0 -275]
        [10 0 -740]
        [10 0 -750]
        [50 0 -790]
        [550 0 -790]
        [650 0 -790]
        [750 0 -790]
        [790 0 -750]
        [790 0 -250]
        [790 0 -150]
        [790 0 -50]
        [750 0 -10]
        [740 0 -10]
        [275 0 -10]
        [50 0 -10]
        [50 0 -810]
        [150 0 -810]
        [250 0 -810]
        [750 0 -810]
        [810 0 -750]
        [810 0 -650]
        [810 0 -550]
        [810 0 -50]
        [10 0 50]
        [10 0 275]
        [10 0 740]
        [10 0 750]
        [50 0 790]
        [150 0 790]
        [250 0 790]
        [750 0 790]
        [790 0 750]
        [790 0 650]
        [790 0 550]
        [790 0 50]
        [750 0 10]
        [740 0 10]
        [275 0 10]
        [50 0 10]
        [50 0 810]
        [550 0 810]
        [650 0 810]
        [750 0 810]
        [810 0 750]
        [810 0 250]
        [810 0 150]
        [810 0 50]
        [-10 0 -50]
        [-10 0 -275]
        [-10 0 -740]
        [-10 0 -750]
        [-50 0 -790]
        [-150 0 -790]
        [-250 0 -790]
        [-750 0 -790]
        [-790 0 -750]
        [-790 0 -650]
        [-790 0 -550]
        [-790 0 -50]
        [-750 0 -10]
        [-740 0 -10]
        [-275 0 -10]
        [-50 0 -10]
        [-50 0 -810]
        [-550 0 -810]
        [-650 0 -810]
        [-750 0 -810]
        [-810 0 -750]
        [-810 0 -250]
        [-810 0 -150]
        [-810 0 -50]
        [-10 0 50]
        [-10 0 275]
        [-10 0 740]
        [-10 0 750]
        [-50 0 790]
        [-550 0 790]
        [-650 0 790]
        [-750 0 790]
        [-790 0 750]
        [-790 0 250]
        [-790 0 150]
        [-790 0 50]
        [-750 0 10]
        [-740 0 10]
        [-275 0 10]
        [-50 0 10]
        [-50 0 810]
        [-150 0 810]
        [-250 0 810]
        [-750 0 810]
        [-810 0 750]
        [-810 0 650]
        [-810 0 550]
        [-810 0 50]


        ];

    connections_circle = [
        [16 1 -pi/2 50 0 -50 100]
        [49 40 pi/2 50 0 -50 100]
        [4 5 -pi/2 50 0 -750 100]
        [17 52 pi/2 50 0 -750 100]
        [8 9 -pi/2 750 0 -750 100]
        [12 13 -pi/2 750 0 -50 100]
        [37 24 pi/2 750 0 -50 100]
        [21 20 pi/2 750 0 -750 100]

        [25 40 -pi/2 50 0 50 100]
        [16 73 pi/2 50 0 50 100]
        [29 28 -pi/2 50 0 750 100]
        [76 41 pi/2 50 0 750 100]
        [33 32 -pi/2 750 0 750 100]
        [37 36 -pi/2 750 0 50 100]
        [48 13 pi/2 750 0 50 100]
        [44 45 pi/2 750 0 750 100]

        [49 64 -pi/2 -50 0 -50 100]
        [88 1 pi/2 -50 0 -50 100]
        [53 52 -pi/2 -50 0 -750 100]
        [4 65 pi/2 -50 0 -750 100]
        [57 56 -pi/2 -750 0 -750 100]
        [61 60 -pi/2 -750 0 -50 100]
        [72 85 pi/2 -750 0 -50 100]
        [68 69 pi/2 -750 0 -750 100]

        [88 73 -pi/2 -50 0 50 100]
        [25 64 pi/2 -50 0 50 100]
        [76 77 -pi/2 -50 0 750 100]
        [89 28 pi/2 -50 0 750 100]
        [80 81 -pi/2 -750 0 750 100]
        [84 85 -pi/2 -750 0 50 100]
        [61 96 pi/2 -750 0 50 100]
        [93 92 pi/2 -750 0 750 100]
        ];

    connections_translation = [
        [1 2 100]
        [2 3 100]
        [3 4 100]
        [5 6 100]
        [6 7 100]
        [7 8 100]
        [9 10 100]
        [10 11 100]
        [11 12 100]
        [13 14 100]
        [14 15 100]
        [15 16 100]
        [18 17 100]
        [19 18 100]
        [20 19 100]
        [22 21 100]
        [23 22 100]
        [24 23 100]

        [26 25 100]
        [27 26 100]
        [28 27 100]
        [30 29 100]
        [31 30 100]
        [32 31 100]
        [34 33 100]
        [35 34 100]
        [36 35 100]
        [38 37 100]
        [39 38 100]
        [40 39 100]
        [41 42 100]
        [42 43 100]
        [43 44 100]
        [45 46 100]
        [46 47 100]
        [47 48 100]

        [50 49 100]
        [51 50 100]
        [52 51 100]
        [54 53 100]
        [55 54 100]
        [56 55 100]
        [58 57 100]
        [59 58 100]
        [60 59 100]
        [62 61 100]
        [63 62 100]
        [64 63 100]
        [65 66 100]
        [66 67 100]
        [67 68 100]
        [69 70 100]
        [70 71 100]
        [71 72 100]

        [73 74 100]
        [74 75 100]
        [75 76 100]
        [77 78 100]
        [78 79 100]
        [79 80 100]
        [81 82 100]
        [82 83 100]
        [83 84 100]
        [85 86 100]
        [86 87 100]
        [87 88 100]
        [90 89 100]
        [91 90 100]
        [92 91 100]
        [94 93 100]
        [95 94 100]
        [96 95 100]


        [48 24 100]
        [12 36 100]
        [89 41 100]
        [29 77 100]
        [84 60 100]
        [72 96 100]
        [53 5 100]
        [17 65 100]
        [16 64 100]
        [88 40 100]
        [25 1 100]
        [49 73 100]

        ];
    
    allRoutes = 1:(length(connections_circle)+length(connections_translation));
    Route_LaneNumber = [allRoutes' ones(length(allRoutes),1)];

    % crossroads waypoint defintions
    startingNodes = [51    14    27    86;
        %                  10    0     46    39;
        %                  74    31    0     91;
        %                  70    63    82    0;
        %                  0     19    2     55
        ];

    breakingNodes = [ 50    15    26    87;
        %                   11    0     47    38;
        %                   75    30    0     90;
        %                   71    62    83    0;
        %                   0     18     3    54
        ];

    stoppingNodes = [49    16    25    88;
        %                  12    0     48    37;
        %                  76    29    0     89;
        %                  72    61    84    0;
        %                  0     17     4    53
        ];

    leavingNodes = [1     40    73    64;
        %                 24    0     36    13;
        %                 28    41    0     77;
        %                 60    85    96    0;
        %                 0     5     52    65
        ];



    %  1 vehicle - 1 platoon (acceleration fail)
    % startingPoints = [59 46 62];
    % destinationPoints = [9 45 45];

    %   3 traffic states sequentielly (breaking fail)
    % startingPoints = [62 8 65];
    % destinationPoints = [8 44 45];

    %   TODO: sort priority traffic states and check for leaving queue match
    % startingPoints = [47 8 65];
    % destinationPoints = [65 44 45];

    %  1 vehicle - 1 platoon (acceleration delayed)
    % startingPoints = [47 9 8];
    % destinationPoints = [65 48 60];

    %   no conflict situation
    % startingPoints = [59 9 8];
    % destinationPoints = [65 45 60];

    %  complex situation ( some acceleration fails)
    % startingPoints = [47 59 65];
    % destinationPoints = [45 48 47];



    % Fabian AIM Scenario
    %startingPoints =    [52 11 32 47 33 92 22 6  45 82];
    %destinationPoints = [35 79 95 79 95 95 21 8  47 83];
    %maxSpeeds =         [7  9  12 9 12.5 19 8  16 16 17];

end