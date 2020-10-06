%% Run this script to automatically test your model and generate reports.

%Load the test 
modelName = 'MOBATSim_AutoTesting';
open_system(modelName)

% Load the example test file
testFile = sltest.testmanager.load('Test_MOBATSim');

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

%generate a report after the simulation
sltest.testmanager.report(resultObj,'CoverageReport_MOBATSim.pdf',...,
    'Title','Vehicle Longitudinal Control Process',...
    'IncludeMATLABFigures',true,...
    'IncludeErrorMessages',true,...
    'IncludeTestResults',0,'LaunchReport',true);