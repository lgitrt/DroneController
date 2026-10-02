% Author: Luca Obwegs
function tests = testResultPublication
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.originalPath = path;
addpath(fullfile(fileparts(mfilename('fullpath')),'..','matlab'));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testSixPublishedFigures(testCase)
verifyEmpty(testCase,comparisonPlotSelection('ideal'));
verifyEqual(testCase,comparisonPlotSelection('sensors'), ...
    {'rmse_summary','imu_measurements'});
verifyEqual(testCase,comparisonPlotSelection('robustness'), ...
    {'xy_tracking_comparison','rmse_summary','tracking_error_comparison','estimation_errors'});
verifyError(testCase,@() comparisonPlotSelection('unknown'),'comparisonPlotSelection:Scenario');
verifyError(testCase,@() plotComparison({},[],tempdir,5,{'unknown'}),'plotComparison:Selection');
end

function testMetricsWithoutFigures(testCase)
cfg = comparisonConfiguration();
outputs = cell(1,3);
for j = 1:3
    outputs{j} = simulateClosedLoop(cfg.p,cfg.traj,cfg.controllers{j}, ...
        cfg.Ts,1,cfg.x0,cfg.scenarios(1));
end
directory = tempname;
figuresBefore = findall(groot,'Type','figure');
metrics = plotComparison(outputs,cfg.p,directory,0,{});
verifyFalse(testCase,exist(directory,'dir') == 7);
verifyEqual(testCase,findall(groot,'Type','figure'),figuresBefore);
for j = 1:3
    expected = trackingMetrics(outputs{j},cfg.p,0);
    verifyEqual(testCase,metrics.(cfg.names{j}),expected);
end
end
