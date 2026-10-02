% Author: Luca Obwegs
function tests = testControllers
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
directory = fullfile(fileparts(mfilename('fullpath')),'..','matlab');
testCase.TestData.originalPath = path;
addpath(directory);
testCase.TestData.p = parameters();
testCase.TestData.Ts = 0.05;
testCase.TestData.traj = struct('radius',2,'omega',2*pi/30,'altitude',2.5,'rampTime',5);
testCase.TestData.scenarios = comparisonScenarios();
testCase.TestData.lqr = struct('type','LQR','K', ...
    designLQR(testCase.TestData.p,'Tuning','Balanced','SampleTime',testCase.TestData.Ts));
testCase.TestData.mpc = struct('type','MPC','mpcObj', ...
    designMPC_Linear(testCase.TestData.p,testCase.TestData.Ts));
end

function teardownOnce(testCase)
path(testCase.TestData.originalPath);
end

function testHoverAndParameterConsistency(testCase)
p = testCase.TestData.p;
verifyEqual(testCase,QuadrotorStateFcn(zeros(12,1),p.uHover*ones(4,1),p), ...
    zeros(12,1),'AbsTol',1e-12);
p.m = 1.3*p.m;
p.uHover = p.m*p.g/(4*p.kt);
verifyEqual(testCase,QuadrotorStateFcn(zeros(12,1),p.uHover*ones(4,1),p), ...
    zeros(12,1),'AbsTol',1e-12);
end

function testLinearizationAndIndependentYaw(testCase)
p = testCase.TestData.p;
[A,B] = quadrotorLinearModel(p);
[numericA,numericB] = QuadrotorStateJacobianFcn(zeros(12,1),p.uHover*ones(4,1),p);
verifyEqual(testCase,A,numericA,'AbsTol',1e-8);
verifyEqual(testCase,B,numericB,'AbsTol',1e-11);
[~,M] = mixingMatrix(p);
verifyEqual(testCase,rank(M),4);
verifyEqual(testCase,rank(ctrb(A,B*p.uHover)),12);
u = M\[p.m*p.g;0;0;0.01];
dx = QuadrotorStateFcn(zeros(12,1),u,p);
verifyEqual(testCase,dx(7:11),zeros(5,1),'AbsTol',1e-10);
verifyEqual(testCase,dx(12),0.01/p.Iz,'AbsTol',1e-10);
end

function testRigidBodyKinematics(testCase)
p = testCase.TestData.p;
x = zeros(12,1);
x(4:6) = [0.12;-0.2;0.6];
x(10:12) = [0.4;-0.3;0.2];
dx = QuadrotorStateFcn(x,p.uHover*ones(4,1),p);
phi = x(4); theta = x(5);
E = [1 0 -sin(theta);0 cos(phi) sin(phi)*cos(theta);0 -sin(phi) cos(phi)*cos(theta)];
Edot = [0 0 -cos(theta)*x(11); ...
    0 -sin(phi)*x(10) cos(phi)*cos(theta)*x(10)-sin(phi)*sin(theta)*x(11); ...
    0 -cos(phi)*x(10) -sin(phi)*cos(theta)*x(10)-cos(phi)*sin(theta)*x(11)];
omega = E*x(10:12);
I = diag([p.Ix,p.Iy,p.Iz]);
verifyEqual(testCase,I*(E*dx(10:12)+Edot*x(10:12))+cross(omega,I*omega), ...
    zeros(3,1),'AbsTol',1e-12);
end

function testDiscreteDesignAndMPCScaling(testCase)
p = testCase.TestData.p;
Ts = testCase.TestData.Ts;
[K,A,B,Q,R] = designLQR(p,'Tuning','Balanced','SampleTime',Ts);
discrete = c2d(ss(A,B,eye(12),zeros(12,4)),Ts,'zoh');
verifyLessThan(testCase,max(abs(eig(discrete.A-discrete.B*K))),1);
obj = testCase.TestData.mpc.mpcObj;
verifyEqual(testCase,obj.Model.Plant.A,discrete.A,'AbsTol',1e-12);
verifyEqual(testCase,obj.Model.Plant.B,discrete.B*p.uHover,'AbsTol',1e-10);
verifyEqual(testCase,obj.Weights.OutputVariables.^2,diag(Q)','AbsTol',1e-12);
verifyEqual(testCase,obj.Weights.ManipulatedVariables.^2,diag(R)'*p.uHover^2,'AbsTol',1e-10);
end

function testMPCAtHoverAndReset(testCase)
p = testCase.TestData.p;
traj = testCase.TestData.traj;
traj.radius = 0; traj.altitude = 0;
a = simulateClosedLoop(p,traj,testCase.TestData.mpc,0.05,1,zeros(12,1));
b = simulateClosedLoop(p,traj,testCase.TestData.mpc,0.05,1,zeros(12,1));
verifyEqual(testCase,a.X,zeros(12,21),'AbsTol',1e-9);
verifyEqual(testCase,a.U,p.uHover*ones(4,20),'AbsTol',1e-7);
verifyEqual(testCase,a.X,b.X,'AbsTol',1e-12);
verifyGreaterThan(testCase,min(a.solverIterations),0);
end

function testActiveCommandConstraints(testCase)
p = testCase.TestData.p;
scenario = testCase.TestData.scenarios(1);
scenario.maxInputStep = 0.005;
controller = struct('type','MPC','mpcObj', ...
    designMPC_Linear(p,0.05,'MaxInputStep',scenario.maxInputStep));
traj = testCase.TestData.traj;
traj.radius = 0; traj.altitude = 2; traj.rampTime = 1;
out = simulateClosedLoop(p,traj,controller,0.05,3,zeros(12,1),scenario);
verifyOutputs(testCase,out,p,scenario);
steps = abs(diff([p.uHover*ones(4,1),out.U],1,2))/p.uHover;
verifyGreaterThan(testCase,max(steps,[],'all'),0.0049);
verifyGreaterThan(testCase,max(out.solverIterations),1);
end

function testFullCircleAndRobustness(testCase)
p = testCase.TestData.p;
traj = testCase.TestData.traj;
cases = testCase.TestData.scenarios;
for i = 1:numel(cases)
    a = simulateClosedLoop(p,traj,testCase.TestData.lqr,0.05,45,zeros(12,1),cases(i));
    b = simulateClosedLoop(p,traj,testCase.TestData.mpc,0.05,45,zeros(12,1),cases(i));
    verifyOutputs(testCase,a,p,cases(i));
    verifyOutputs(testCase,b,p,cases(i));
    verifyEqual(testCase,a.Ref,b.Ref);
    verifyEqual(testCase,a.windForce,b.windForce);
    verifySize(testCase,b.X,[12 901]);
    metricsA = trackingMetrics(a,p,traj.rampTime);
    metricsB = trackingMetrics(b,p,traj.rampTime);
    verifyLessThan(testCase,metricsA.rmse3D,0.08);
    verifyLessThan(testCase,metricsB.rmse3D,0.08);
    verifyLessThan(testCase,metricsA.maxError3D,0.2);
    verifyLessThan(testCase,metricsB.maxError3D,0.2);
    verifyEqual(testCase,metricsB.rmse3D,sqrt(3)*metricsB.rmseCoordinate,'AbsTol',1e-12);
    if i == 1
        idealA = metricsA.rmse3D;
        idealB = metricsB.rmse3D;
        verifyEqual(testCase,a.UApplied(:,1:end-1),a.U,'AbsTol',1e-12);
        verifyEqual(testCase,b.UApplied(:,1:end-1),b.U,'AbsTol',1e-12);
    else
        verifyGreaterThan(testCase,metricsA.rmse3D,idealA);
        verifyGreaterThan(testCase,metricsB.rmse3D,idealB);
        verifyGreaterThan(testCase,max(abs(b.UApplied(:,2:end)-b.U),[],'all'),1);
    end
end
end

function testInvalidInputsFailExplicitly(testCase)
p = testCase.TestData.p;
traj = testCase.TestData.traj;
verifyError(testCase,@() simulateClosedLoop(p,traj,struct('type','invalid'),0.05,1,zeros(12,1)), ...
    'simulateClosedLoop:Controller');
verifyError(testCase,@() simulateClosedLoop(p,traj,testCase.TestData.mpc,0.1,1,zeros(12,1)), ...
    'simulateClosedLoop:SampleTime');
verifyError(testCase,@() simulateClosedLoop(p,traj,testCase.TestData.lqr,0.05,1.01,zeros(12,1)), ...
    'simulateClosedLoop:Duration');
end

function testInfeasibleMPCDoesNotSilentlyFallback(testCase)
p = testCase.TestData.p;
obj = designMPC_Linear(p,0.05);
obj.OV(3).Min = 100;
obj.OV(3).Max = 100;
obj.OV(3).MinECR = 0;
obj.OV(3).MaxECR = 0;
controller = struct('type','MPC','mpcObj',obj);
verifyError(testCase,@() simulateClosedLoop(p,testCase.TestData.traj,controller, ...
    0.05,0.1,zeros(12,1)),'simulateClosedLoop:MPCSolver');
end

function verifyOutputs(testCase,out,p,scenario)
verifyTrue(testCase,all(isfinite(out.X),'all'));
verifyGreaterThanOrEqual(testCase,min(out.U,[],'all'),p.uMin-1e-6);
verifyLessThanOrEqual(testCase,max(out.U,[],'all'),p.uMax+1e-6);
verifyGreaterThanOrEqual(testCase,min(out.UApplied,[],'all'),p.uMin-1e-6);
verifyLessThanOrEqual(testCase,max(out.UApplied,[],'all'),p.uMax+1e-6);
steps = abs(diff([p.uHover*ones(4,1),out.U],1,2))/p.uHover;
verifyLessThanOrEqual(testCase,max(steps,[],'all'),scenario.maxInputStep+1e-10);
if all(isfinite(out.solverIterations))
    verifyGreaterThan(testCase,min(out.solverIterations),0);
    verifyFalse(testCase,any(out.rateLimited|out.saturated));
end
end
