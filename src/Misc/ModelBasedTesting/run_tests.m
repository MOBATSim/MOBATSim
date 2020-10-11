%% Run this script to automatically test your model and generate reports.

% Load the existing requirements set
rs1 = slreq.load('export_Requirements_MOBATSim');

% Open the requirements set in the Requirements Editor
slreq.open(rs1);

% Load the tested model 
modelName = 'MOBATSim_AutoTesting';
open_system(modelName)


% Load the example test file
testFile = sltest.testmanager.load('UnitTest_MOBATSim');

% Get the test suite object by test suite name
testSuite = getTestSuiteByName(testFile,'Test Scenarios');

% Get the test case object
% testCase = getTestCases(testSuite);

% Run the test case and return an object with results data
resultObj = run(testSuite);

% Get the coverage results
coverageResult = getCoverageResults(resultObj);

% Open the Test Manager to view results
sltest.testmanager.view;

% Generate a report after the simulation
sltest.testmanager.report(resultObj,'.\src\Misc\ModelBasedTesting\TestReport_UnitTest_MOBATSim.pdf',...
    'Author','Test Engineer',...
    'Title','Test Results of Vehicle Longitudinal Control Part',...
    'IncludeMATLABFigures',true,...     
    'IncludeErrorMessages',true,...     
    'IncludeTestRequirement',true,...   
    'IncludeTestResults',0,...
    'IncludeCoverageResult',true,...
    'LaunchReport',true);            

% 'IncludeMATLABFigures' 
        %Option to include the figures opened from a callback script, 
        %custom criteria, or by the model in the report
% 'IncludeErrorMessages'
        %Choose to include error messages from the test case simulations
% 'IncludeTestRequirement'
        %Choose to include the test requirement link defined under the tab Requirements in the test case
% 'IncludeTestResults'
        %Option to include all or a subset of test results in the report:
        %select passed and failed results, specified as the integer value 0; 
        %select only passed results, specified as the value 1; 
        %or select only failed results, specified as the value 2
% 'IncludeCoverageResult'        
        %Choose to include coverage metrics that are collected at test execution
% 'LaunchReport'
        %Open the report when it is finished generating


%Open model testing dashboard
%modelTestingDashboard