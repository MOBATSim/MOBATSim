function MOBATSimAutoTesting(options)
%MOBATSim TEST Script: Simulation-based testing with different driving scenarios.
%   All scenarios have to produce the [PASSED] as the output without errors
    arguments
        options.simStopTime         (1,1) double    = 50                    % Time to test the scenarios in seconds
        options.scenarioNames       (1,:) string    = [ "Urban City Traffic", ...
                                                        "Curved Road Overtaking", ...
                                                        "Merge Collision", ...
                                                        "Platoon Control", ...
                                                        "High Speed Collision", ...
                                                        "Collision Merge", ...
                                                        "Complex Crossroad", ...
                                                        "Extra Complex Crossroad", ...
                                                        "Counterexample Crossroad Safety", ...
                                                        "Mayhem", ...
                                                        "Convoy Platoons", ...
                                                        "Vehicle Safety Filter Test"];     % Scenarios to test
    end

    % List of results for every scenario
    passed = strings(size(options.scenarioNames));
    % Make or open a text file for the report
    fileID = fopen('AutoTestReport.txt','w');
    % Header
    fprintf(fileID,'Automatic Test Report\n\r');
    tic;
    % Test each scenario
    for i=1:length(options.scenarioNames)
        try
            % prepare and run simulation
            prepare_simulator("scenarioName",options.scenarioNames(i),"simStopTime",options.simStopTime);
            mpcverbosity off; % Make sure that the MPC doesn't fill the command window
            run_Sim();
            passed(i) = "PASSED";
        catch ME
            passed(i) = "ERROR";
        end

        % Display scenario name and if passed    
        disp("<strong>" + options.scenarioNames(i) + ":</strong>" + " [" + passed(i) +"]");
        % Add this information also to the report
        fprintf(fileID,'%s: [%s]\n',options.scenarioNames(i),passed(i));

        % Display only the exception message, but dont interrupt the execution
        if  passed(i) ~= "PASSED"
            fprintf(2,ME.cause{1}.message + "\n");
        end
    end
    % Finish and close the report
    fclose(fileID);
    toc;
end

