% Author: Luca Obwegs
function tests = testSensorsAndPID
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.originalPath = path;
addpath(fullfile(fileparts(mfilename('fullpath')),'..','matlab'));
testCase.TestData.p = parameters();
testCase.TestData.traj = struct('radius',2,'omega',2*pi/30,'altitude',2.5,'rampTime',5);
testCase.TestData.cases = comparisonScenarios(true);
testCase.TestData.controllers = { ...
    struct('type','LQR','K',designLQR(testCase.TestData.p,'Tuning','Balanced','SampleTime',0.05)), ...
    struct('type','MPC','mpcObj',designMPC_Linear(testCase.TestData.p,0.05)),designPID()};
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testBodySensorFramesAndGravity(testCase)
p = testCase.TestData.p;
cfg = sensorParameters();
cfg.accelSigma = 0; cfg.gyroSigma = 0; cfg.magSigma = 0;
cfg.accelBiasWalk = 0; cfg.gyroBiasWalk = 0;
x = zeros(12,1);
x(4:6) = [0.1;-0.15;0.3]; x(10:12) = [0.2;-0.1;0.4];
acceleration = [0.3;-0.2;0.1];
stream = RandStream('mt19937ar','Seed',cfg.seed);
[sample,~] = sampleSensors(x,acceleration,0,zeros(6,1),stream,cfg,p);
[R,E] = attitudeGeometry(x(4:6));
verifyEqual(testCase,R*sample.accel-[0;0;p.g],acceleration,'AbsTol',1e-12);
verifyEqual(testCase,E\sample.gyro,x(10:12),'AbsTol',1e-12);
verifyEqual(testCase,R*sample.mag,cfg.magneticField,'AbsTol',1e-12);
verifyEqual(testCase,R'*R,eye(3),'AbsTol',1e-12);
end

function testNoiseStatisticsAndSampleRates(testCase)
p = testCase.TestData.p;
cfg = sensorParameters();
cfg.accelBiasWalk = 0; cfg.gyroBiasWalk = 0;
stream = RandStream('mt19937ar','Seed',cfg.seed);
bias = [cfg.accelBias;cfg.gyroBias];
n = 6000;
noise = zeros(6,n);
gnss = false(1,n); mag = gnss;
for k = 1:n
    [s,bias] = sampleSensors(zeros(12,1),zeros(3,1),k-1,bias,stream,cfg,p);
    noise(:,k) = [s.accel-s.accelTruth;s.gyro-s.gyroTruth]-bias;
    gnss(k) = s.hasGNSS; mag(k) = s.hasMag;
end
expectedSigma = [cfg.accelSigma*ones(3,1);cfg.gyroSigma*ones(3,1)];
verifyLessThan(testCase,max(abs(std(noise,0,2)./expectedSigma-1)),0.04);
verifyLessThan(testCase,max(abs(mean(noise,2)./expectedSigma)),0.05);
verifyEqual(testCase,nnz(gnss),n/cfg.gnssPeriod);
verifyEqual(testCase,nnz(mag),n/cfg.magPeriod);
end

function testEKFCovarianceAndBiasCorrection(testCase)
p = testCase.TestData.p;
cfg = sensorParameters();
stream = RandStream('mt19937ar','Seed',cfg.seed);
bias = [cfg.accelBias;cfg.gyroBias];
[s,bias] = sampleSensors(zeros(12,1),zeros(3,1),0,bias,stream,cfg,p);
filter = initializeNavigationFilter(s,cfg);
initialRateError = norm(s.gyro);
for k = 1:4000
    [s,bias] = sampleSensors(zeros(12,1),zeros(3,1),k,bias,stream,cfg,p);
    [filter,state] = navigationFilterStep(filter,s,0.005,cfg,p);
    if mod(k,200) == 0
        verifyEqual(testCase,filter.P,filter.P','AbsTol',1e-12);
        verifyGreaterThanOrEqual(testCase,min(eig(filter.P)),-1e-12);
    end
end
verifyLessThan(testCase,norm(state(1:3)),0.08);
verifyLessThan(testCase,norm(state(4:6)),deg2rad(2));
verifyLessThan(testCase,norm(filter.x(13:15)-bias(4:6)),initialRateError);
end

function testPIDHoverAndMixing(testCase)
p = testCase.TestData.p;
traj = testCase.TestData.traj;
traj.radius = 0; traj.altitude = 0;
ref = circleTrajectory(0,p,traj);
[u,memory] = pidControl(designPID(),[],zeros(12,1),ref,p,0.05);
verifyEqual(testCase,u,p.uHover*ones(4,1),'AbsTol',1e-8);
verifyEqual(testCase,memory.positionIntegral,zeros(3,1));
ref.state(3) = 0.2;
[u,~] = pidControl(designPID(),[],zeros(12,1),ref,p,0.05);
verifyGreaterThan(testCase,min(u),p.uHover);
end

function testEstimatorNeverUsesTruthFields(testCase)
p = testCase.TestData.p; cfg = sensorParameters();
stream = RandStream('mt19937ar','Seed',cfg.seed);
bias = [cfg.accelBias;cfg.gyroBias];
[sample,bias] = sampleSensors(zeros(12,1),zeros(3,1),0,bias,stream,cfg,p);
filter = initializeNavigationFilter(sample,cfg);
[sample,~] = sampleSensors(zeros(12,1),zeros(3,1),1,bias,stream,cfg,p);
poisoned = sample;
poisoned.accelTruth = nan(3,1); poisoned.gyroTruth = nan(3,1);
[a,xa] = navigationFilterStep(filter,sample,0.005,cfg,p);
[b,xb] = navigationFilterStep(filter,poisoned,0.005,cfg,p);
verifyEqual(testCase,xa,xb,'AbsTol',1e-12);
verifyEqual(testCase,a.P,b.P,'AbsTol',1e-12);
end

function testPIDAntiWindupWithActuatorLimits(testCase)
p = testCase.TestData.p;
traj = testCase.TestData.traj;
traj.radius = 0; traj.altitude = 20; traj.rampTime = 1;
cfg = testCase.TestData.cases(1);
cfg.maxInputStep = 0.001;
out = simulateClosedLoop(p,traj,designPID(),0.05,3,zeros(12,1),cfg);
verifyTrue(testCase,any(out.rateLimited));
limited = find(out.rateLimited|out.saturated);
verifyEqual(testCase,out.pidState(1:6,limited+1),out.pidState(1:6,limited),'AbsTol',1e-12);
steps = abs(diff([p.uHover*ones(4,1),out.U],1,2))/p.uHover;
verifyLessThanOrEqual(testCase,max(steps,[],'all'),cfg.maxInputStep+1e-10);
verifyTrue(testCase,all(isfinite(out.X),'all'));
end

function testNoisyControllerRepeatabilityAndCommonRandomNumbers(testCase)
p = testCase.TestData.p; traj = testCase.TestData.traj;
cfg = testCase.TestData.cases(2);
a = simulateClosedLoop(p,traj,testCase.TestData.controllers{1},0.05,5,zeros(12,1),cfg);
b = simulateClosedLoop(p,traj,testCase.TestData.controllers{1},0.05,5,zeros(12,1),cfg);
c = simulateClosedLoop(p,traj,testCase.TestData.controllers{3},0.05,5,zeros(12,1),cfg);
verifyEqual(testCase,a.X,b.X,'AbsTol',1e-12);
verifyEqual(testCase,a.XEstimated,b.XEstimated,'AbsTol',1e-12);
verifyEqual(testCase,a.biasTrue,c.biasTrue,'AbsTol',1e-12);
indices = 1:20:size(a.sensors.position,2);
stateIndices = 1:2:size(a.X,2);
verifyEqual(testCase,a.sensors.position(:,indices)-a.X(1:3,stateIndices), ...
    c.sensors.position(:,indices)-c.X(1:3,stateIndices),'AbsTol',1e-12);
verifyGreaterThan(testCase,norm(a.XEstimated-a.X,'fro'),0.1);
verifyGreaterThanOrEqual(testCase,min(eig(a.finalEstimatorCovariance)),-1e-12);
end

function testFullNoisyAndDisturbedComparisons(testCase)
p = testCase.TestData.p; traj = testCase.TestData.traj;
for i = 2:3
    for j = 1:3
        out = simulateClosedLoop(p,traj,testCase.TestData.controllers{j},0.05,45,zeros(12,1),testCase.TestData.cases(i));
        m = trackingMetrics(out,p,5);
        verifyTrue(testCase,all(isfinite(out.XEstimated),'all'));
        verifySize(testCase,out.sensors.accel,[3 9001]);
        verifyLessThan(testCase,m.rmse3D,0.15);
        verifyLessThan(testCase,m.maxError3D,0.4);
        verifyLessThan(testCase,m.estimationPositionRMSE,0.04);
        verifyLessThan(testCase,m.estimationAttitudeRMSEDeg,1);
        verifyGreaterThanOrEqual(testCase,min(eig(out.finalEstimatorCovariance)),-1e-12);
        verifyLessThanOrEqual(testCase,m.peakCommandStep,0.2+1e-10);
        if j == 2
            verifyGreaterThan(testCase,min(out.solverIterations),0);
        end
    end
end
end
