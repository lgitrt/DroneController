function out = simulateClosedLoop(p, traj, controller, Ts, tEnd, x0, scenario)
%SIMULATECLOSEDLOOP Shared nonlinear plant, limits and state feedback.
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
if ~isMPC && ~isNMPC && ~strcmpi(controller.type, 'LQR')
    error('simulateClosedLoop:Controller', 'Unknown controller type: %s', controller.type);
end
nSub = max(10, ceil(Ts/0.005));
dtSim = Ts/nSub;
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

for k = 1:N
    tk = out.t(k);
    xk = out.X(:,k);
    clock = tic;
    if isMPC
        preview = circleTrajectory(tk + Ts*(1:Hp), p, traj);
        state.Plant = xk;
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
        [requested, ~, info] = nlmpcmove(controller.nlobj, xk, lastMV, preview.state(1:6,:)');
        out.solverIterations(k) = info.ExitFlag;
        if info.ExitFlag <= 0
            error('simulateClosedLoop:NMPCSolver', ...
                'NMPC failed at t=%.3f s (ExitFlag=%d).', tk, info.ExitFlag);
        end
    else
        requested = p.uHover*ones(4,1) - controller.K*(xk-out.Ref(:,k));
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
    out.U(:,k) = uk;
    lastMV = uk;
    out.windForce(:,k) = scenario.windForce(tk);

    if scenario.motorTimeConstant > 0
        z = [xk; motorInput];
        for s = 1:nSub
            z = rk4Step(@(t, z) laggedPlant(t, z, uk, p, scenario), ...
                tk+(s-1)*dtSim, z, dtSim);
        end
        out.X(:,k+1) = z(1:12);
        motorInput = z(13:16);
    else
        out.UApplied(:,k) = uk;
        for s = 1:nSub
            xk = rk4Step(@(t, x) plant(t, x, uk, p, scenario), ...
                tk+(s-1)*dtSim, xk, dtSim);
        end
        out.X(:,k+1) = xk;
        motorInput = uk;
    end
    out.UApplied(:,k+1) = motorInput;
    if ~all(isfinite(out.X(:,k+1))) || any(abs(out.X(4:5,k+1)) >= pi/2)
        error('simulateClosedLoop:Plant', 'Nonfinite state or Euler singularity at t=%.3f s.', out.t(k+1));
    end
end
out.windForce(:,end) = scenario.windForce(out.t(end));
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
