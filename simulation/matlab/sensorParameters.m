% Author: Luca Obwegs
function cfg = sensorParameters()
%SENSORPARAMETERS Representative calibrated MEMS and fixed-solution RTK.
%   Standard deviations are per sample, not noise spectral densities.
cfg.imuSampleTime = 0.005;
cfg.gnssPeriod = 20;
cfg.magPeriod = 10;
cfg.accelSigma = 0.04;                % m/s^2
cfg.gyroSigma = deg2rad(0.08);        % rad/s
cfg.accelBias = [0.02;-0.015;0.025];  % m/s^2 residual after calibration
cfg.gyroBias = deg2rad([0.12;-0.10;0.08]);
cfg.accelBiasWalk = 0.001;           % m/s^2 / sqrt(s)
cfg.gyroBiasWalk = deg2rad(0.005);    % rad/s / sqrt(s)
cfg.gnssPositionSigma = [0.02;0.02;0.04]; % m (RTK fixed solution)
cfg.gnssVelocitySigma = [0.03;0.03;0.05]; % m/s
cfg.magneticField = [20;0;45];        % microtesla, known local field
cfg.magSigma = 0.35;                 % microtesla
cfg.seed = 20261002;
end
