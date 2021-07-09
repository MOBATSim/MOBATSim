function [startingPoints, destinationPoints, maxSpeeds] = load_scenario(scenarioName)
    switch scenarioName
        case 'Urban City Traffic'
            %% Test scenarios           
%            startingPoints =    [51 47 48 3 55 53 6 7 22 45];
%            destinationPoints = [13 15 18 27 26 40 27 32 16 10];
%            maxSpeeds =         [13 13 11 9 13 13 12 14 4 13];
% 
            startingPoints =    [46 47 85 3  53 64 58 57 4 63];
            destinationPoints = [54 78 81 26 3  42 27 25 3 47];
            maxSpeeds =         [3  15  6 7  14 10 0.1 4 10 10];
         %% test scenario Qianwei
             %V4 and V3 on the same road testing front collision
            %startingPoints =    [46 47 7 27 85 52 64 57 15 63];
            %destinationPoints = [54 51 26 26  78  27 42 25 49 47];
            %maxSpeeds = [3 15 50 1 6 10 10 12 10 10];

           %V4 go straight, v3 tries to merge， collision
%            startingPoints =    [46 47 6 7 85 52 64 57 15 63];
%            destinationPoints = [54 51 27 26  27  78 42 25 49 47];
%            maxSpeeds = [3 15 6.5 5.55 6 10 10 12 10 10]; 
          % the boundary of the collision and no collision of the v3 is
           %8.9,here set 6.5 for collision situation

            %% Complex crossroad scenario
%             startingPoints =    [8  46 29 18 53 52 64 6 15 63];
%             destinationPoints = [26 47 51 1  3  27 42 25 76 46];
%             maxSpeeds = [20 15 10 10 14 20 10 12 10 10];

            %% temp goal test
    %         startingPoints =    [23 28 30 58 19 18 17 53 51 78];
    %         destinationPoints = [36 36 29 57 18 17 16 52 78 77];
    %         maxSpeeds =         [20 20 10 10 20 20 10 12 10 10];

            %% Collision scenario    
    %         startingPoints =    [51 47 22 4 55 53 6 7 15 30]; 
    %         destinationPoints = [10 11 10 12 34 11 18 20 25 11];
    %         maxSpeeds = [13 12.4 11 8.4 13 13 9.2 13.2 9 13];  

            %% SC1
            %short driving scenario
    %         startingPoints =    [30  41 18  7 48 72 56 57 78 51];
    %         destinationPoints = [28  34 16 27 47 71 70 56 77 78];
    %         maxSpeeds =         [20  10 15 20 20 20 20 20 20 20];

            %% SC2
            %V2 should be detected by V1, even if it only drives a short time
    %         startingPoints =    [28  41 19  7 48 72 56 57 78 51];
    %         destinationPoints = [11  40 15 27 47 71 70 56 77 78];
    %         maxSpeeds =         [18  20 20 20 20 20 20 20 20 20];

            %% SC2a edge cost test
            %V1 and V2 should take the left route, because the slow V3 takes
            %the shorter right one
    %         startingPoints =    [21  19 18  7 48 72 56 57 78 51]; % Test futureData
    %         destinationPoints = [ 2  35 74 27 47 71 70 56 77 78];% Test futureData
    %         maxSpeeds =         [20  19  8 20 20 20 20 20 20 20];% Test futureData

            %% SC2b edge cost test
            %Long driving to see if other cars change the chosen route
    %         startingPoints =    [28  41 20  7 48 72 56 57 78 51];
    %         destinationPoints = [ 2  35 74 27 47 71 70 56 77 78];
    %         maxSpeeds =         [18   8 19 20 20 20 20 20 20 20];

            %% SC3 test temporary goal
            %V1 should redirect the path before the crashed vehicles V2 and V3
    %         startingPoints =    [14   7  6 41 48 72 56 57 78 51];
    %         destinationPoints = [ 2  27 10 34 47 71 70 56 77 78];
    %         maxSpeeds =         [20   5 10 20 20 20 20 20 20 20];

            %% SC3a dont block before you are there
            %V2 should reach node 10, even if node 11 is blocked
    %                 startingPoints =    [23  22 19  7 48 72 56 57 78 51];
    %                 destinationPoints = [10  11 15 27 47 71 70 56 77 78];
    %                 maxSpeeds =         [20  11 20 20 20 20 20 20 20 20];

            %% SC3a use temporary goal
            %V2 should drive to node 11
            %TODO: Error with D*EL
            %Output argument "currentRoute" (and maybe others) not assigned during call to "VehicleKinematics/setCurrentRoute"
    %                 startingPoints =    [23  22 19  7 48 72 56 57 78 51];
    %                 destinationPoints = [11  11 15 27 47 71 70 56 77 78];
    %                 maxSpeeds =         [20  20 20 20 20 20 20 20 20 20];

        case 'Carmageddon'
            startingPoints =    [46 53 54 51 44 9  65 27 30 6];
            destinationPoints = [25 64 24 16 10 59 42 10 11 24];
            maxSpeeds = [28 25 10 25 38 25 25 30 11 15];

        case 'Convoy Platoons'
            % road merging scenario
            startingPoints =    [51 47 48 3 55 53 6 7 22 45];
            destinationPoints = [13 15 18 11 26 40 17 32 16 10];
            maxSpeeds = [13 13 11 9 13 13 8 14 4 13];

        case 'Intersection'
            startingPoints =    [52 11 32 47 33 92 22 6  45 82];
            destinationPoints = [35 79 95 79 95 95 63 59  47 27];
            maxSpeeds =         [17  19  22 19 19 19 28  26 26 27];

        case 'Hard Left'
            startingPoints =    [20 23 24 92 93 96 84 83 82 30];
            destinationPoints = [80 79 78 9 8 7 68 67 66 60];
            maxSpeeds =         [17  19  12 19 13 19 14  16 16 17];

    end
    % single car vs. 2-platoon vs. 3-platoon (Crossroad Map)
    % startingPoints =    [52 11  32 10  33 34 22 6  45 82];
    % destinationPoints = [35 79 95 79 95 95 21 8  47 83];
    % maxSpeeds =         [8 10 19 19 22 24 8  16 16 17];



    % road merging scenario
    % startingPoints      = [50   46      15      3       54      52      6       7       47      30];
    % destinationPoints   = [13   17      20      11      25      41      18      31      15      39];
    % maxSpeeds           = [13   12.9    12      9.2     13      13      12.8    13.8    6       13];

    %Current Scenario
    % startingPoints =    [47 48 72 73 46 55 52 45 26 31];
    % destinationPoints = [16 75 1 14 10 11 15 17 18 19];
    % maxSpeeds = [20 15 10 20 15 10 20 15 10 20];

    % road merging scenario (Used for the Experiment AAC2019)
    % startingPoints =    [51 47 48 3 55 53 6 7 31 49];
    % destinationPoints = [13 15 18 11 26 40 17 32 16 37];
    % maxSpeeds = [13 12.8 11 9 13 13 8.6 13.8 6 13];

    %% Errors to be Solved (TODO)
    % Error in this Scenario Should be solved
    % startingPoints =    [47 48 72 73 46 55 52 45 26 31];
    % destinationPoints = [16 75 1 14 10 11 15 17 18 19];
    % maxSpeeds = [20 15 10 20 15 10 20 15 10 20];

    % Error in this Scenario Should be solved (crossroadunit error)
    % startingPoints =    [51 47 48 4 55 53 6 7 15 49];
    % destinationPoints = [37 15 18 33 34 15 65 55 16 19];
    % maxSpeeds = [13 12.4 12 8 13 13 11 9 9 13];

    % road merging scenario Collisions should be solved
    % startingPoints =    [51 47 48 4 55 53 6 7 15 49];
    % destinationPoints = [37 15 18 33 34 15 18 20 16 19];
    % maxSpeeds = [13 12.4 11 8.4 13 13 9.2 13.2 9 13];

    % Fabian AIM Scenario
    %startingPoints =    [52 11 32 47 33 92 22 6  45 82];
    %destinationPoints = [35 79 95 79 95 95 21 8  47 83];
    %maxSpeeds =         [7  9  12 9 12.5 19 8  16 16 17];
    
end

