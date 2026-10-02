% Author: Luca Obwegs
function [A, B] = QuadrotorStateJacobianFcn(x, u, p)
%QUADROTORSTATEJACOBIANFCN Central-difference Jacobian of the nonlinear plant.
if nargin < 3
    p = parameters();
end
A = zeros(12);
B = zeros(12,4);
for i = 1:12
    h = eps^(1/3)*max(1,abs(x(i)));
    offset = zeros(12,1);
    offset(i) = h;
    A(:,i) = (QuadrotorStateFcn(x+offset,u,p)-QuadrotorStateFcn(x-offset,u,p))/(2*h);
end
for i = 1:4
    h = eps^(1/3)*max(1,abs(u(i)));
    offset = zeros(4,1);
    offset(i) = h;
    B(:,i) = (QuadrotorStateFcn(x,u+offset,p)-QuadrotorStateFcn(x,u-offset,p))/(2*h);
end
end
