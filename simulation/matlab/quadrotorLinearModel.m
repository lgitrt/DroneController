function [A, B] = quadrotorLinearModel(p)
%QUADROTORLINEARMODEL Linearized quadrotor model about hover.
%   [A, B] = QUADROTORLINEARMODEL(P) returns the continuous-time state
%   space matrices of the quadrotor dynamics linearized about the hover
%   equilibrium (zero attitude, zero velocity, equal rotor speeds).
%
%   State:  x = [x y z phi theta psi xdot ydot zdot phidot thetadot psidot]'
%   Input:  u = [u1 u2 u3 u4]'  (squared rotor speeds, rad^2/s^2)
%
%   The small-angle hover linearization decouples into four
%   independently controllable chains (x-theta, y-phi, z, psi), which is
%   the standard textbook quadrotor result used for LQR design, see e.g.
%   Gibiansky, "Quadcopter Dynamics, Simulation, and Control".
%
%   This closed-form model is checked against finite differences of
%   QuadrotorStateFcn at hover by the simulation tests.

A = zeros(12);
A(1,7)  = 1;   % xdot
A(2,8)  = 1;   % ydot
A(3,9)  = 1;   % zdot
A(4,10) = 1;   % phidot
A(5,11) = 1;   % thetadot
A(6,12) = 1;   % psidot
A(7,5)  =  p.g;   % xddot  =  g*theta
A(8,4)  = -p.g;   % yddot  = -g*phi

B = zeros(12, 4);
B(9,:)  = (p.kt / p.m)        * [1  1  1  1];
B(10,:) = (p.l * p.kt / p.Ix) * [1  0 -1  0];
B(11,:) = (p.l * p.kt / p.Iy) * [0  1  0 -1];
B(12,:) = (p.d / p.Iz)        * [1 -1  1 -1];

end
