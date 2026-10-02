function [K, A, B] = designLQR(p, varargin)
%DESIGNLQR Linear-Quadratic-Regulator gain for hover-linearized quadrotor.
%   [K, A, B] = DESIGNLQR(P) linearizes the quadrotor dynamics about
%   hover (see QUADROTORLINEARMODEL) and solves the continuous-time LQR
%   problem to obtain the state-feedback gain K such that
%       u(t) = uHover - K*(x(t) - xRef(t))
%   tracks a (slowly varying) reference state xRef.
%
%   Optional name-value pair: 'Tuning' can be 'Aggressive' (default),
%   'Balanced', or 'Conservative' to adjust the position/rate weight
%   tradeoff.

p_ = inputParser;
p_.addParameter('Tuning', 'Aggressive');
p_.parse(varargin{:});
tuning = p_.Results.Tuning;

[A, B] = quadrotorLinearModel(p);

% State order: [x y z phi theta psi xdot ydot zdot phidot thetadot psidot]
switch tuning
    case 'Aggressive'
        qPos   = [60 60 100];
        qAtt   = [8 8 4];
        qVel   = [4 4 6];
        qRate  = [0.8 0.8 0.5];
        R_scale = 5e-11;
    case 'Balanced'
        qPos   = [40 40 60];
        qAtt   = [6 6 3];
        qVel   = [2 2 3];
        qRate  = [0.3 0.3 0.2];
        R_scale = 1e-10;
    case 'Conservative'
        qPos   = [20 20 30];
        qAtt   = [3 3 1.5];
        qVel   = [1 1 1.5];
        qRate  = [0.1 0.1 0.05];
        R_scale = 5e-10;
    otherwise
        error('Unknown tuning: %s', tuning);
end

Q = diag([qPos, qAtt, qVel, qRate]);
R = R_scale * eye(4);

K = lqr(A, B, Q, R);

end
