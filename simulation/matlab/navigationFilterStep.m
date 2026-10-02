% Author: Luca Obwegs
function [filter, state] = navigationFilterStep(filter, sample, dt, cfg, p)
%NAVIGATIONFILTERSTEP 15-state inertial navigation EKF with bias estimation.
%   Only measurements and noise parameters enter this estimator.
imu = 0.5*(filter.previousIMU+[sample.accel;sample.gyro]);
filter.previousIMU = [sample.accel;sample.gyro];
x = filter.x;
angles = x(7:9);
[R,E] = attitudeGeometry(angles);
specificForce = imu(1:3)-x(10:12);
omega = imu(4:6)-x(13:15);
acceleration = R*specificForce-[0;0;p.g];
angleRate = E\omega;
F = eye(15);
F(1:3,4:6) = dt*eye(3);
F(1:3,10:12) = -0.5*dt^2*R;
F(4:6,10:12) = -dt*R;
F(7:9,13:15) = -dt*(E\eye(3));
for i = 1:3
    delta = zeros(3,1); delta(i) = 1e-5;
    [Rp,Ep] = attitudeGeometry(angles+delta);
    [Rm,Em] = attitudeGeometry(angles-delta);
    derivativeAcceleration = (Rp-Rm)*specificForce/(2e-5);
    F(1:3,6+i) = 0.5*dt^2*derivativeAcceleration;
    F(4:6,6+i) = dt*derivativeAcceleration;
    F(7:9,6+i) = F(7:9,6+i)+dt*((Ep\omega)-(Em\omega))/(2e-5);
end
G = zeros(15,12);
G(1:3,1:3) = 0.5*dt^2*R;
G(4:6,1:3) = dt*R;
G(7:9,4:6) = dt*(E\eye(3));
G(10:15,7:12) = sqrt(dt)*eye(6);
noise = [cfg.accelSigma*ones(3,1);cfg.gyroSigma*ones(3,1); ...
    cfg.accelBiasWalk*ones(3,1);cfg.gyroBiasWalk*ones(3,1)];
filter.x(1:3) = x(1:3)+dt*x(4:6)+0.5*dt^2*acceleration;
filter.x(4:6) = x(4:6)+dt*acceleration;
filter.x(7:9) = x(7:9)+dt*angleRate;
filter.P = F*filter.P*F'+G*diag(noise.^2)*G';
if sample.hasGNSS
    H = [eye(6),zeros(6,9)];
    variance = [cfg.gnssPositionSigma;cfg.gnssVelocitySigma].^2;
    filter = correct(filter,[sample.position;sample.velocity],filter.x(1:6),H,diag(variance));
end
if sample.hasMag
    [R,~] = attitudeGeometry(filter.x(7:9));
    predicted = R'*cfg.magneticField;
    H = zeros(3,15);
    for i = 1:3
        delta = zeros(3,1); delta(i) = 1e-5;
        [Rp,~] = attitudeGeometry(filter.x(7:9)+delta);
        [Rm,~] = attitudeGeometry(filter.x(7:9)-delta);
        H(:,6+i) = (Rp'-Rm')*cfg.magneticField/(2e-5);
    end
    filter = correct(filter,sample.mag,predicted,H,cfg.magSigma^2*eye(3));
end
filter.x(7:9) = atan2(sin(filter.x(7:9)),cos(filter.x(7:9)));
filter.P = (filter.P+filter.P')/2;
if ~all(isfinite(filter.x)) || ~all(isfinite(filter.P),'all') || abs(filter.x(8)) >= pi/2
    error('navigationFilterStep:State', 'Invalid EKF state or covariance.');
end
state = navigationControllerState(filter,sample);
end

function filter = correct(filter, measurement, prediction, H, variance)
innovationVariance = H*filter.P*H'+variance;
gain = (filter.P*H')/innovationVariance;
filter.x = filter.x+gain*(measurement-prediction);
residual = eye(15)-gain*H;
% Joseph form avoids loss of covariance symmetry/positive semidefiniteness.
filter.P = residual*filter.P*residual'+gain*variance*gain';
end
