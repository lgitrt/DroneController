function mpcobj = designMPC_Linear(p, Ts, varargin)
%DESIGNMPC_LINEAR Constrained MPC with normalized hover-deviation inputs.
%   The manipulated variable is (u-uHover)/uHover, not absolute rotor speed.
%   Q and R match the discrete LQR cost in physical state/input units.

parser = inputParser;
parser.addParameter('PredictionHorizon', 40, @(v) isscalar(v) && isfinite(v) && v >= 1 && v == floor(v));
parser.addParameter('ControlHorizon', 20, @(v) isscalar(v) && isfinite(v) && v >= 1 && v == floor(v));
parser.addParameter('Tuning', 'Balanced');
parser.addParameter('MaxInputStep', 0.2, @(v) isscalar(v) && isfinite(v) && v > 0);
parser.parse(varargin{:});
opt = parser.Results;
validateattributes(Ts, {'double'}, {'scalar', 'positive', 'finite'});
if opt.ControlHorizon > opt.PredictionHorizon
    error('designMPC_Linear:Horizon', 'Control horizon must not exceed prediction horizon.');
end

[~, A, B, Q, R] = designLQR(p, 'Tuning', opt.Tuning, 'SampleTime', Ts);
plant = c2d(ss(A, B*p.uHover, eye(12), zeros(12,4)), Ts, 'zoh');
mpcobj = mpc(plant, Ts, opt.PredictionHorizon, opt.ControlHorizon);
mpcobj.Model.Nominal.X = zeros(12,1);
mpcobj.Model.Nominal.U = zeros(4,1);
mpcobj.Model.Nominal.Y = zeros(12,1);
mpcobj.Model.Noise = ss(eye(12));

% Both controllers receive the true plant state, without a hidden estimator.
setoutdist(mpcobj, 'model', ss(zeros(12,1)));
setEstimator(mpcobj, 'custom');
for i = 1:4
    mpcobj.MV(i).Min = (p.uMin-p.uHover)/p.uHover;
    mpcobj.MV(i).Max = (p.uMax-p.uHover)/p.uHover;
    mpcobj.MV(i).RateMin = -opt.MaxInputStep;
    mpcobj.MV(i).RateMax = opt.MaxInputStep;
    mpcobj.MV(i).MinECR = 0;
    mpcobj.MV(i).MaxECR = 0;
    mpcobj.MV(i).RateMinECR = 0;
    mpcobj.MV(i).RateMaxECR = 0;
    mpcobj.MV(i).ScaleFactor = 1;
end
for i = 1:12
    mpcobj.OV(i).ScaleFactor = 1;
end
mpcobj.Weights.OutputVariables = sqrt(diag(Q))';
mpcobj.Weights.ManipulatedVariables = p.uHover*sqrt(diag(R))';
mpcobj.Weights.ManipulatedVariablesRate = zeros(1,4);
end
