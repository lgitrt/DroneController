% Author: Luca Obwegs
function [R, E] = attitudeGeometry(angles)
%ATTITUDEGEOMETRY Body-to-inertial ZYX rotation and Euler-rate mapping.
phi = angles(1); theta = angles(2); psi = angles(3);
cp = cos(phi); sp = sin(phi);
ct = cos(theta); st = sin(theta);
cy = cos(psi); sy = sin(psi);
R = [cy*ct, cy*st*sp-sy*cp, cy*st*cp+sy*sp; ...
     sy*ct, sy*st*sp+cy*cp, sy*st*cp-cy*sp; ...
     -st,   ct*sp,          ct*cp];
E = [1 0 -st; 0 cp sp*ct; 0 -sp cp*ct];
end
