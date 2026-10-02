function filter = initializeNavigationFilter(sample, cfg)
%INITIALIZENAVIGATIONFILTER Initialize using sensors, not the plant state.
if ~sample.hasGNSS || ~sample.hasMag
    error('initializeNavigationFilter:Aiding', 'Initial GNSS and magnetic measurements are required.');
end
a = sample.accel;
phi = atan2(a(2),a(3));
theta = atan2(-a(1),hypot(a(2),a(3)));
[R,~] = attitudeGeometry([phi;theta;0]);
leveledField = R*sample.mag;
psi = atan2(cfg.magneticField(2),cfg.magneticField(1)) ...
    - atan2(leveledField(2),leveledField(1));
% Navigation state: [position; velocity; Euler angles; accel bias; gyro bias].
filter.x = [sample.position; sample.velocity; phi;theta;psi; zeros(6,1)];
sigma = [cfg.gnssPositionSigma;cfg.gnssVelocitySigma; ...
    deg2rad([2;2;3]);0.05*ones(3,1);deg2rad(0.3)*ones(3,1)];
filter.P = diag(sigma.^2);
filter.previousIMU = [sample.accel;sample.gyro];
end
