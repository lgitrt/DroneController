% Author: Luca Obwegs
function [sample, bias] = sampleSensors(x, acceleration, index, bias, stream, cfg, p)
%SAMPLESENSORS Noisy body-frame specific force, angular rate, RTK and field.
%   Truth is used only here to synthesize measurements, never by the EKF.
if index > 0
    bias = bias + sqrt(cfg.imuSampleTime)*[cfg.accelBiasWalk*randn(stream,3,1); ...
        cfg.gyroBiasWalk*randn(stream,3,1)];
end
[R,E] = attitudeGeometry(x(4:6));
sample.accelTruth = R'*(acceleration+[0;0;p.g]);
sample.gyroTruth = E*x(10:12);
sample.accel = sample.accelTruth + bias(1:3) + cfg.accelSigma*randn(stream,3,1);
sample.gyro = sample.gyroTruth + bias(4:6) + cfg.gyroSigma*randn(stream,3,1);
% Draw on every IMU tick so random sequences do not depend on aiding rate.
positionNoise = cfg.gnssPositionSigma.*randn(stream,3,1);
velocityNoise = cfg.gnssVelocitySigma.*randn(stream,3,1);
magNoise = cfg.magSigma*randn(stream,3,1);
sample.hasGNSS = mod(index,cfg.gnssPeriod) == 0;
sample.hasMag = mod(index,cfg.magPeriod) == 0;
sample.position = nan(3,1);
sample.velocity = nan(3,1);
sample.mag = nan(3,1);
if sample.hasGNSS
    sample.position = x(1:3) + positionNoise;
    sample.velocity = x(7:9) + velocityNoise;
end
if sample.hasMag
    sample.mag = R'*cfg.magneticField + magNoise;
end
end
