prepare_simulator()
    
    tic;
    
    nr_Simulations = 40;
    
    for k = 1:nr_Simulations
        try
            rng('shuffle');
            evalin('base', 'run_Sim');
            MonteCarlo_preprocess();
        catch ME
            disp('Error in Simulation');
            disp(startingPoints);
            disp(destinationPoints);            
        end
        prepare_simulator("Analysing",0,"FI_id",3,"FI_value",(k*0.1)-2)
        % evalin('base', 'run_Sim');
        
        % Create/Load Table
        % Log Data + Add Data to Table
        % Save the Table (because prepare simulator will clear data)
    end
    toc;

