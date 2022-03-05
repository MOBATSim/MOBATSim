function MOBATSimAutoTesting(options)
%MOBATSIMTESTER Test the simulation with different scenarios.
%   All scenarios have to produce the right output and no errors to pass
%   the test.
    arguments
        options.simStopTime         (1,1) double    = 80                    % Time to test the scenarios in seconds
        options.scenarioNames       (1,:) string    = [ "Urban City Traffic", ...
                                                        "Curved Road Overtaking", ...
                                                        "Road Merge Collision", ...
                                                        "Platoon Control", ...
                                                        "High Speed Collision", ...
                                                        "Road Merge Collision V3,V4", ...
                                                        "Complex Crossroad", ...
                                                        "Extra Complex Crossroad", ...
                                                        "Counterexample Crossroad Safety", ...
                                                        "Carmageddon", ...
                                                        "Convoy Platoons"];     % Scenarios to test
    end

    % a list of results for every scenario
    passed = strings(size(options.scenarioNames));
    % make or open a file for the report
    fileID = fopen('AutoTestReport.txt','w');
    % header
    fprintf(fileID,'Automatic Test Report\n\r');
    tic;
    % test every scenario
    for i=1:length(options.scenarioNames)
        try
            % prepare and run simulation
            prepare_simulator("scenarioName",options.scenarioNames(i),"simStopTime",options.simStopTime);
            run_Sim();
            passed(i) = "PASSED";
        catch ME
            passed(i) = "ERROR";
        end

        % display scenario name and if passed    
        disp("<strong>" + options.scenarioNames(i) + ":</strong>" + " [" + passed(i) +"]");
        % add this information also to the report
        fprintf(fileID,'%s: [%s]\n',options.scenarioNames(i),passed(i));

        % display only the exception message, but dont interrupt the execution
        if  passed(i) ~= "PASSED"
            fprintf(2,ME.cause{1}.message + "\n");
        end
    end
    % finish report
    fclose(fileID);
    toc;
end

