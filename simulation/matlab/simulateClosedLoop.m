% Author: Luca Obwegs
function out = simulateClosedLoop(p, traj, controller, Ts, tEnd, x0, scenario)
%SIMULATECLOSEDLOOP Shared plant and limits with ideal or aided-IMU feedback.
%   U contains requested (bounded) inputs; UApplied contains motor inputs
%   at each sample. Wind is an inertial-frame force, not known to controllers.

if nargin < 7
    cases = comparisonScenarios();
    scenario = cases(1);
end
validateattributes(Ts, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(tEnd, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(x0, {'double'}, {'size', [12 1], 'finite'});
validateattributes(scenario.motorTimeConstant, {'double'}, {'scalar', 'nonnegative', 'finite'});
validateattributes(scenario.maxInputStep, {'double'}, {'scalar', 'positive', 'finite'});
N = round(tEnd/Ts);
if abs(N*Ts-tEnd) > 1e-9
    error('simulateClosedLoop:Duration', 'Duration must be an integer number of control intervals.');
end
isMPC = strcmpi(controller.type, 'MPC');
isNMPC = strcmpi(controller.type, 'NMPC');
isPID = strcmpi(controller.type, 'PID');
if ~isMPC && ~isNMPC && ~isPID && ~strcmpi(controller.type, 'LQR')
    error('simulateClosedLoop:Controller', 'Unknown controller type: %s', controller.type);
end
nSub = max(10, ceil(Ts/0.005));
dtSim = Ts/nSub;
if scenario.sensorsEnabled && abs(dtSim-scenario.sensorConfig.imuSampleTime) > 1e-12
    error('simulateClosedLoop:IMURate', 'Plant substeps must match the configured IMU sample time.');
end
if isMPC
    if abs(controller.mpcObj.Ts-Ts) > 1e-12
        error('simulateClosedLoop:SampleTime', 'MPC and simulation sample times must match.');
    end
    state = mpcstate(controller.mpcObj);
    state.Plant = x0;
    state.LastMove = zeros(4,1);
    Hp = controller.mpcObj.PredictionHorizon;
elseif isNMPC
    Hp = controller.nlobj.PredictionHorizon;
end

out.t = (0:N)*Ts;
out.X = zeros(12,N+1);
out.X(:,1) = x0;
reference = circleTrajectory(out.t, p, traj);
out.Ref = reference.state;
out.U = zeros(4,N);
out.UApplied = zeros(4,N+1);
out.UApplied(:,1) = p.uHover*ones(4,1);
out.windForce = zeros(3,N+1);
out.solveTime = zeros(1,N);
out.solverIterations = nan(1,N);
out.rateLimited = false(1,N);
out.saturated = false(1,N);
out.scenario = scenario.name;
out.maxInputStep = scenario.maxInputStep;
out.motorTimeConstant = scenario.motorTimeConstant;
lastMV = p.uHover*ones(4,1);
motorInput = lastMV;
pidMemory = [];
if isPID
    out.pidState = zeros(9,N+1);
    out.pidState(7:9,1) = out.Ref(4:6,1);
end
out.XEstimated = out.X;
out.controllerType = controller.type;
out.estimationEnabled = scenario.sensorsEnabled;
if scenario.sensorsEnabled
    cfg = scenario.sensorConfig;
    stream = RandStream('mt19937ar','Seed',cfg.seed);
    sensorBias = [cfg.accelBias;cfg.gyroBias];
    dx = plant(0,x0,motorInput,p,scenario);
    [sample,sensorBias] = sampleSensors(x0,dx(7:9),0,sensorBias,stream,cfg,p);
    filter = initializeNavigationFilter(sample,cfg);
    estimatedState = navigationControllerState(filter,sample);
    out.XEstimated(:,1) = estimatedState;
    out.sensors.t = (0:N*nSub)*dtSim;
    fields = {'accel','gyro','accelTruth','gyroTruth','position','velocity','mag'};
    for f = 1:numel(fields)
        out.sensors.(fields{f}) = nan(3,N*nSub+1);
        out.sensors.(fields{f})(:,1) = sample.(fields{f});
    end
    out.biasTrue = zeros(6,N+1);
    out.biasEstimated = zeros(6,N+1);
    out.biasTrue(:,1) = sensorBias;
    out.biasEstimated(:,1) = filter.x(10:15);
    out.covarianceDiagonal = zeros(15,N+1);
    out.covarianceDiagonal(:,1) = diag(filter.P);
end

for k = 1:N
    tk = out.t(k);
    xk = out.X(:,k);
    if scenario.sensorsEnabled
        feedback = estimatedState;
    else
        feedback = xk;
        out.XEstimated(:,k) = xk;
    end
    clock = tic;
    if isMPC
        preview = circleTrajectory(tk + Ts*(1:Hp), p, traj);
        state.Plant = feedback;
        state.LastMove = (lastMV-p.uHover)/p.uHover;
        [du, info] = mpcmove(controller.mpcObj, state, [], preview.state', []);
        out.solverIterations(k) = info.Iterations;
        if info.Iterations <= 0 || ~all(isfinite(du))
            error('simulateClosedLoop:MPCSolver', ...
                'MPC did not solve optimally at t=%.3f s (iterations=%d).', tk, info.Iterations);
        end
        requested = p.uHover*(1+du);
    elseif isNMPC
        preview = circleTrajectory(tk + Ts*(1:Hp), p, traj);
        [requested, ~, info] = nlmpcmove(controller.nlobj, feedback, lastMV, preview.state(1:6,:)');
        out.solverIterations(k) = info.ExitFlag;
        if info.ExitFlag <= 0
            error('simulateClosedLoop:NMPCSolver', ...
                'NMPC failed at t=%.3f s (ExitFlag=%d).', tk, info.ExitFlag);
        end
    elseif isPID
        previousPID = pidMemory;
        rk = circleTrajectory(tk,p,traj);
        [requested,pidMemory] = pidControl(controller,pidMemory,feedback,rk,p,Ts);
    else
        requested = p.uHover*ones(4,1) - controller.K*(feedback-out.Ref(:,k));
    end
    out.solveTime(k) = toc(clock);
    if ~all(isfinite(requested))
        error('simulateClosedLoop:Input', 'Nonfinite control input at t=%.3f s.', tk);
    end
    step = scenario.maxInputStep*p.uHover;
    bounded = min(max(requested, p.uMin), p.uMax);
    uk = min(max(bounded, lastMV-step), lastMV+step);
    out.saturated(k) = any(abs(bounded-requested) > 1e-6);
    out.rateLimited(k) = any(abs(uk-bounded) > 1e-6);
    if isMPC && (out.saturated(k) || out.rateLimited(k))
        error('simulateClosedLoop:MPCConstraints', 'MPC violated shared input limits at t=%.3f s.', tk);
    end
    % Conditional integration prevents windup when motor/rate limits bind.
    if isPID && (out.saturated(k) || out.rateLimited(k))
        if isempty(previousPID)
            pidMemory.positionIntegral = zeros(3,1);
            pidMemory.attitudeIntegral = zeros(3,1);
        else
            pidMemory.positionIntegral = previousPID.positionIntegral;
            pidMemory.attitudeIntegral = previousPID.attitudeIntegral;
        end
    end
    if isPID
        out.pidState(:,k+1) = [pidMemory.positionIntegral; ...
            pidMemory.attitudeIntegral;pidMemory.previousDesired];
    end
    out.U(:,k) = uk;
    lastMV = uk;
    out.windForce(:,k) = scenario.windForce(tk);

    if scenario.motorTimeConstant == 0
        out.UApplied(:,k) = uk;
        motorInput = uk;
    end
    z = [xk;motorInput];
    for s = 1:nSub
        subTime = tk+(s-1)*dtSim;
        if scenario.motorTimeConstant > 0
            z = rk4Step(@(t, z) laggedPlant(t, z, uk, p, scenario), ...
                subTime, z, dtSim);
        else
            z(1:12) = rk4Step(@(t, x) plant(t, x, uk, p, scenario), ...
                subTime, z(1:12), dtSim);
        end
        if scenario.sensorsEnabled
            index = (k-1)*nSub+s;
            dx = plant(subTime+dtSim,z(1:12),z(13:16),p,scenario);
            [sample,sensorBias] = sampleSensors(z(1:12),dx(7:9),index,sensorBias,stream,cfg,p);
            [filter,estimatedState] = navigationFilterStep(filter,sample,dtSim,cfg,p);
            for f = 1:numel(fields)
                out.sensors.(fields{f})(:,index+1) = sample.(fields{f});
            end
        end
    end
    out.X(:,k+1) = z(1:12);
    motorInput = z(13:16);
    out.UApplied(:,k+1) = motorInput;
    if scenario.sensorsEnabled
        out.XEstimated(:,k+1) = estimatedState;
        out.biasTrue(:,k+1) = sensorBias;
        out.biasEstimated(:,k+1) = filter.x(10:15);
        out.covarianceDiagonal(:,k+1) = diag(filter.P);
    else
        out.XEstimated(:,k+1) = out.X(:,k+1);
    end
    if ~all(isfinite(out.X(:,k+1))) || any(abs(out.X(4:5,k+1)) >= pi/2)
        error('simulateClosedLoop:Plant', 'Nonfinite state or Euler singularity at t=%.3f s.', out.t(k+1));
    end
end
out.windForce(:,end) = scenario.windForce(out.t(end));
if scenario.sensorsEnabled
    out.finalEstimatorCovariance = filter.P;
end
end

function dx = plant(t, x, u, p, scenario)
force = scenario.windForce(t);
validateattributes(force, {'double'}, {'size', [3 1], 'finite'});
dx = QuadrotorStateFcn(x, u, p);
dx(7:9) = dx(7:9) + force/p.m;
end

function dz = laggedPlant(t, z, command, p, scenario)
dz = [plant(t, z(1:12), z(13:16), p, scenario); ...
    (command-z(13:16))/scenario.motorTimeConstant];
end

function next = rk4Step(f, t, x, dt)
k1 = f(t, x);
k2 = f(t+dt/2, x+dt*k1/2);
k3 = f(t+dt/2, x+dt*k2/2);
k4 = f(t+dt, x+dt*k3);
next = x + dt*(k1+2*k2+2*k3+k4)/6;
end
