% Author: Luca Obwegs
function dx = QuadrotorStateFcn(x, u, p)
%QUADROTORSTATEFCN Rigid-body quadrotor dynamics with ZYX Euler angles.
%   State rates 10:12 are Euler angle rates, not body angular velocity.
%   Inertial z is positive upwards; rotor inputs are squared speeds.
if nargin < 3
    p = parameters();
end
phi = x(4);
theta = x(5);
psi = x(6);
rates = x(10:12);
cp = cos(phi); sp = sin(phi);
ct = cos(theta); st = sin(theta);
cy = cos(psi); sy = sin(psi);

thrustAxis = [cy*st*cp+sy*sp; sy*st*cp-cy*sp; ct*cp];
[~, mixer] = mixingMatrix(p);
forceTorque = mixer*u;
acceleration = thrustAxis*forceTorque(1)/p.m - [0;0;p.g];

% omegaBody = E * eulerRates; differentiate E as well as the rates.
E = [1 0 -st; 0 cp sp*ct; 0 -sp cp*ct];
Edot = [0 0 -ct*rates(2); ...
    0 -sp*rates(1) cp*ct*rates(1)-sp*st*rates(2); ...
    0 -cp*rates(1) -sp*ct*rates(1)-cp*st*rates(2)];
omega = E*rates;
inertia = diag([p.Ix p.Iy p.Iz]);
omegaDot = inertia\(forceTorque(2:4)-cross(omega,inertia*omega));
eulerAcceleration = E\(omegaDot-Edot*rates);
dx = [x(7:9); rates; acceleration; eulerAcceleration];
end
