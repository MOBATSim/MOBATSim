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
sltest.testmanager.report(resultObj,'C:\Users\62783\Documents\GitHub\MOBATSim\src\Misc\ModelBasedTesting\TestReport_UnitTest_MOBATSim.pdf',...
    'Title','Test Result of Vehicle Longitudinal Control Part',...
    'IncludeMATLABFigures',true,...
    'IncludeErrorMessages',true,...
    'IncludeTestResults',0,'LaunchReport',true);

% Open model testing dashboard
% modelTestingDashboard