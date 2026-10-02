function nlobj = designNMPC(p, Ts, varargin)
%DESIGNNMPC Nonlinear MPC controller for the quadrotor.
%   NLOBJ = DESIGNNMPC(P, TS) builds an nlmpc object that stabilizes
%   the quadrotor at a commanded setpoint using the nonlinear prediction
%   model QUADROTORSTATEFCN/QUADROTORSTATEJACOBIANFCN.
%
%   The design is conservative: it uses a longer prediction horizon,
%   a shorter control horizon, and soft output tracking to ensure the
%   solver converges reliably. Reference tracking is approximately
%   achieved via feedback rather than aggressive trajectory optimization.
%
%   NOTE ON ROTOR COMMAND BOUNDS: the manipulated variables are squared
%   rotor speeds ui = omega_i^2 (rad^2/s^2, see PARAMETERS.M), so their
%   natural range is [0, wMax^2] (~[0, 3.9e6]) with a hover value of
%   ~4.6e5 per rotor.

p_ = inputParser;
p_.addParameter('PredictionHorizon', 20);
p_.addParameter('ControlHorizon', 2);
p_.parse(varargin{:});
opt = p_.Results;

nx = 12;
ny = 6;  % Output: [x y z phi theta psi]
nu = 4;

nlobj = nlmpc(nx, ny, nu);
nlobj.Model.StateFcn = "QuadrotorStateFcn";
nlobj.Jacobian.StateFcn = @QuadrotorStateJacobianFcn;
nlobj.Model.OutputFcn = @(x, u) x(1:6);  % Extract [x y z phi theta psi]

nlobj.Ts = Ts;
nlobj.PredictionHorizon = opt.PredictionHorizon;
nlobj.ControlHorizon = opt.ControlHorizon;

nlobj.MV = struct( ...
    Min=    {p.uMin; p.uMin; p.uMin; p.uMin}, ...
    Max=    {p.uMax; p.uMax; p.uMax; p.uMax}, ...
    RateMin={-0.3*p.uMax; -0.3*p.uMax; -0.3*p.uMax; -0.3*p.uMax}, ...
    RateMax={ 0.3*p.uMax;  0.3*p.uMax;  0.3*p.uMax;  0.3*p.uMax});

% Soft output tracking with emphasis on position, then attitude
nlobj.Weights.OutputVariables        = [20 20 30 4 4 2];
nlobj.Weights.ManipulatedVariables      = [1e-8 1e-8 1e-8 1e-8];
nlobj.Weights.ManipulatedVariablesRate  = [1e-9 1e-9 1e-9 1e-9];

end
