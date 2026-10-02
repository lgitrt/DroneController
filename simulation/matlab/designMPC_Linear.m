function mpcobj = designMPC_Linear(p, Ts, varargin)
%DESIGNMPC_LINEAR Linear MPC controller using mpc toolbox.
%   MPCOBJ = DESIGNMPC_LINEAR(P, TS) designs a linear MPC controller based
%   on the linearized quadrotor dynamics at hover. This avoids the
%   numerical difficulties of nonlinear MPC on an underactuated system
%   (4 inputs, 6 trackable outputs).
%
%   The controller is slower and softer than LQR, prioritizing robust
%   tracking with constraint handling. Useful for comparison and
%   understanding the trade-offs between linear feedback (LQR) and
%   online optimization (MPC).

p_ = inputParser;
p_.addParameter('PredictionHorizon', 15);
p_.addParameter('ControlHorizon', 3);
p_.parse(varargin{:});
opt = p_.Results;

% Linearized continuous-time model
[A, B] = quadrotorLinearModel(p);

% Convert to discrete-time
Ad = A * Ts + eye(12);   % First-order Euler approx for simplicity
Bd = B * Ts;

% Full 12-state output (identity)
Cd = eye(12);
Dd = zeros(12, 4);

sys = ss(Ad, Bd, Cd, Dd, Ts);

% Create MPC
mpcobj = mpc(sys, Ts, opt.PredictionHorizon, opt.ControlHorizon);

% Input (rotor command) bounds and rates
for i = 1:4
    mpcobj.MV(i).Min = p.uMin;
    mpcobj.MV(i).Max = p.uMax;
    mpcobj.MV(i).RateMin = -0.3 * p.uMax;
    mpcobj.MV(i).RateMax =  0.3 * p.uMax;
end

% Output: track position and attitude (first 6 states), ignore rates
mpcobj.Weights.OutputVariables = [20 20 30 4 4 2 0 0 0 0 0 0];

% Input weight (smoothness)
mpcobj.Weights.ManipulatedVariables = 1e-8 * ones(1, 4);

end
