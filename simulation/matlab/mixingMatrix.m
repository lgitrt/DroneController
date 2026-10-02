function [T, M] = mixingMatrix(p)
%MIXINGMATRIX Rotor-command <-> virtual-input mixing matrix.
%   [T, M] = MIXINGMATRIX(P) returns the 4x4 matrix M such that
%       v = M * u,    u = M \ v
%   where u = [u1;u2;u3;u4] are the squared rotor speeds and
%   v = [Fz; tauPhi; tauTheta; tauPsi] is the "virtual" control input
%   (collective thrust and body-axis torques). T is an alias of M kept
%   for readability at call sites (T == M).
%
%   Rotor layout (standard "+" configuration, rotors 1 and 3 on the
%   roll axis, rotors 2 and 4 on the pitch axis, alternating spin
%   direction for yaw control):
%       Fz      = kt*(u1 + u2 + u3 + u4)
%       tauPhi  = l*kt*(u1 - u3)
%       tauTheta= l*kt*(u2 - u4)
%       tauPsi  = d*(u1 - u2 + u3 - u4)

M = [ p.kt,       p.kt,       p.kt,       p.kt;
      p.l*p.kt,   0,         -p.l*p.kt,   0;
      0,          p.l*p.kt,   0,         -p.l*p.kt;
      p.d,       -p.d,        p.d,       -p.d];

T = M;

end
