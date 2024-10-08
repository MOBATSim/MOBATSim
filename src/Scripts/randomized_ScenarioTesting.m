function randomized_ScenarioTesting(options)
%Random testing Script: Simulation-based testing with RANDOM driving scenarios.
%   Failed scenarios will be shown in the command window with the scenario parameters for reproducibility

% Contributors: Johannes Pintscher, Mustafa Saraoglu

    arguments
        options.simStopTime             (1,1) double     = 120    % Time to test the scenarios in seconds
        options.simNumber               (1,1) double     = 4      % How many simulations to run
        options.MaxSpeedLowerBound      (1,1) double     = 5      % Default value for lower max speed value
        options.MaxSpeedUpperBound      (1,1) double     = 20     % Default value for upper max speed value
    end
    
    %% Set starting and destination sets
    AllSets = 1:78;
    OnlyStartingSet = [30 38 6 53 29 28 37 5];
    OnlyDestinationSet = [16 1 2 59 60];
    crossroadPointsSet = [56 72 73 65 51 44 70 71 74 75 78 68 62 49 66 77 43 69 50 67 63 42 76];
    
    RemoveFromStarting = intersect(AllSets,[OnlyDestinationSet crossroadPointsSet]);
    startingSet = setxor(AllSets,RemoveFromStarting);
    
    RemoveFromDestination = intersect(AllSets,[OnlyStartingSet crossroadPointsSet]);
    destinationSet = setxor(AllSets,RemoveFromDestination);
    
    %% Start the timer to measure the wall-clock time for the entire simulations
    tic;
    % Test each scenario
    for i=1:options.simNumber
        
        %% Randomize the initial points for the next loop
        conflict = 1;
        
        while conflict~=0
            rng('shuffle')
            StartingIdx = randperm(length(startingSet),10);
            rng('shuffle')
            DestinationIdx = randperm(length(destinationSet),10);
            [~,ia,ib] = intersect(startingSet(StartingIdx), destinationSet(DestinationIdx));
            conflict = nnz(ia == ib);
        end
        
        maxSpeedSet = randi([options.MaxSpeedLowerBound,options.MaxSpeedUpperBound],10,1); % set random max speeds
        
        try
            
            disp("Simulation number:"+ num2str(i));
            % prepare and run simulation
            prepare_simulator("startingPoints",startingSet(StartingIdx)',"destinationPoints",destinationSet(DestinationIdx)',"maxSpeeds",maxSpeedSet,"simStopTime",options.simStopTime);
            mpcverbosity off; % Make sure that the MPC doesn't fill the command window
            run_Sim();
            disp("Successful simulation:"+ num2str(i));
            
        catch ME
           
            disp(ME);
            disp("Discard simulation:"+ num2str(i));
            disp(["Starting Points:" num2str(startingSet(StartingIdx))]);
            disp(["Destination Points:" num2str(destinationSet(DestinationIdx))]);
            disp(["Max Speeds:" num2str(maxSpeedSet')]);
            disp(["Simulation Time:" num2str(options.simStopTime)])

        end

    end
    %% Simulations done - Output: Real-time in seconds
    toc;
end

