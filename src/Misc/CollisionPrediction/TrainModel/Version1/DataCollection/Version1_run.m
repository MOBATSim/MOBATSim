prepare_simulator()
    
    tic;
    for k = 1
        try
            rng('shuffle');
            evalin('base', 'run_Sim');
            Version1_preprocess();
        catch ME
            disp('Deney gg');
            disp(startingPoints);
            disp(destinationPoints);            
        end
        prepare_simulator()    
    end
    toc;

