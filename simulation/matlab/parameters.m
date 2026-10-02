% Author: Luca Obwegs
function p = parameters()
%PARAMETERS Physical and controller-design parameters of the quadrotor.
%   P = PARAMETERS() returns a struct with the mechanical parameters of
%   the quadrotor (mass, inertia, arm length, rotor thrust/drag
%   coefficients) together with the state/input layout convention used
%   throughout this project.
%
%   State vector (12x1):
%       [x y z phi theta psi xdot ydot zdot phidot thetadot psidot]
%   where phi/theta/psi are roll/pitch/yaw (rad) and the remaining
%   entries are the corresponding linear/angular rates.
%
%   Input vector (4x1):
%       u = [u1 u2 u3 u4], ui = omega_i^2 (squared rotor speed, rad^2/s^2)
%
%   These values are shared by controller design and nonlinear dynamics.

p.m  = 1;              % [kg]      total mass
p.g  = 9.81;            % [m/s^2]   gravitational acceleration
p.Ix = 6700e-6;          % [kg m^2]  roll axis inertia
p.Iy = 6700e-6;          % [kg m^2]  pitch axis inertia
p.Iz = 13270e-6;         % [kg m^2]  yaw axis inertia
p.l  = 0.186;            % [m]       arm length (center to rotor)

p.wMax = (18870 / 60) * 2 * pi;      % [rad/s] max rotor speed
p.kt   = 82.58 / (4 * p.wMax^2);     % [N s^2 / rad^2] rotor thrust coefficient
p.d    = 0.15 * p.kt;                % [N m s^2 / rad^2] rotor drag/yaw coefficient

p.uMax = p.wMax^2;       % [rad^2/s^2] max squared rotor speed (per MV bound)
p.uMin = 0;

% Hover equilibrium input (equal thrust on all four rotors balances gravity)
p.uHover = p.m * p.g / (4 * p.kt);

% Attitude limits used to clip the feed-forward reference angles
p.rollMax  = pi/4;
p.pitchMax = pi/4;

end
