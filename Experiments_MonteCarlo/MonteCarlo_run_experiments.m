prepare_simulator()
    
    tic;
    for k = 1:1
        try
            rng('shuffle');
            evalin('base', 'run_Sim');
            MonteCarlo_preprocess();
        catch ME
            disp('Error in Simulation');
            disp(startingPoints);
            disp(destinationPoints);            
        end
        prepare_simulator()    
    end
    toc;

