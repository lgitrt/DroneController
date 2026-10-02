% Author: Luca Obwegs
function ref = circleTrajectory(t, p, traj)
%CIRCLETRAJECTORY Reference trajectory generator: smooth-start circle.
%   REF = CIRCLETRAJECTORY(T, P, TRAJ) evaluates the reference trajectory
%   at time(s) T (row vector) and returns a struct of 1xN arrays:
%       REF.state  - 12xN full reference state (position/attitude/rates)
%       REF.pos    - 3xN reference position [x;y;z]
%       REF.vel    - 3xN reference velocity
%
%   The horizontal path is a circle of radius TRAJ.radius centered at the
%   origin, reached at constant altitude TRAJ.altitude. Both the radius
%   and the altitude are smoothly ramped in over TRAJ.rampTime using a
%   quintic (minimum-jerk) time-scaling s(t), so the drone starts from
%   rest at the origin instead of teleporting onto the circle.
%
%   Feed-forward attitude: the reference roll/pitch angles are derived
%   from the required horizontal acceleration via the standard
%   differential-flatness relation for a quadrotor
%       xddot_ref =  g*theta_ff   =>  theta_ff =  xddot_ref / g
%       yddot_ref = -g*phi_ff     =>  phi_ff   = -yddot_ref / g
%   which lets a linear controller track a curved path without relying
%   purely on feedback error.
%
%   TRAJ fields: radius, omega, altitude, rampTime.

t = t(:)';
N = numel(t);
h = 1e-3; % finite-difference step for numeric differentiation

[pos, vel, acc] = pathDerivatives(t, p, traj, h);

phi_ff   = max(min(-acc(2,:) / p.g, p.rollMax),  -p.rollMax);
theta_ff = max(min( acc(1,:) / p.g, p.pitchMax), -p.pitchMax);

% Rates of the feed-forward angles, via finite differences as well.
phi_ff_p   = feedforwardAngles(t + h, p, traj);
phi_ff_m   = feedforwardAngles(t - h, p, traj);
phidot_ff   = (phi_ff_p(1,:) - phi_ff_m(1,:)) / (2*h);
thetadot_ff = (phi_ff_p(2,:) - phi_ff_m(2,:)) / (2*h);

ref.state = [pos; phi_ff; theta_ff; zeros(1,N); vel; phidot_ff; thetadot_ff; zeros(1,N)];
ref.pos = pos;
ref.vel = vel;
ref.acc = acc;

end

% ------------------------------------------------------------------
function [pos, vel, acc] = pathDerivatives(t, p, traj, h)
pos = pathPosition(t, traj);
pos_p = pathPosition(t + h, traj);
pos_m = pathPosition(t - h, traj);
vel = (pos_p - pos_m) / (2*h);
acc = (pos_p - 2*pos + pos_m) / (h^2);
end

% ------------------------------------------------------------------
function pos = pathPosition(t, traj)
s = rampProfile(t, traj.rampTime);
r = s .* traj.radius;
z = s .* traj.altitude;
theta = traj.omega .* t;
pos = [r .* cos(theta); r .* sin(theta); z];
end

% ------------------------------------------------------------------
function s = rampProfile(t, rampTime)
%RAMPPROFILE Quintic smoothstep from 0 to 1 over [0, rampTime].
tau = max(min(t ./ rampTime, 1), 0);
s = 6*tau.^5 - 15*tau.^4 + 10*tau.^3;
end

% ------------------------------------------------------------------
function phiTheta = feedforwardAngles(t, p, traj)
h = 1e-3;
pos_p = pathPosition(t + h, traj);
pos   = pathPosition(t, traj);
pos_m = pathPosition(t - h, traj);
acc = (pos_p - 2*pos + pos_m) / (h^2);
phi_ff   = max(min(-acc(2,:) / p.g, p.rollMax),  -p.rollMax);
theta_ff = max(min( acc(1,:) / p.g, p.pitchMax), -p.pitchMax);
phiTheta = [phi_ff; theta_ff];
end
