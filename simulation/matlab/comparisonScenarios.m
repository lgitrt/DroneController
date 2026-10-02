function scenarios = comparisonScenarios(includeSensors)
%COMPARISONSCENARIOS Reproducible ideal and unmodelled wind/motor-lag cases.
ideal.name = 'Ideal baseline';
ideal.slug = 'ideal';
ideal.motorTimeConstant = 0;
ideal.maxInputStep = 0.2;
ideal.windForce = @(t) zeros(3,1);
ideal.sensorsEnabled = false;
ideal.sensorConfig = sensorParameters();

robust = ideal;
robust.name = 'Wind and motor lag';
robust.slug = 'robustness';
robust.motorTimeConstant = 0.04;
robust.windForce = @windForce;
scenarios = [ideal, robust];
if nargin > 0 && includeSensors
    sensed = ideal;
    sensed.name = 'Noisy sensors and EKF';
    sensed.slug = 'sensors';
    sensed.sensorsEnabled = true;
    robust.name = 'Wind, motor lag and sensors';
    robust.sensorsEnabled = true;
    scenarios = [ideal, sensed, robust];
end
end

function force = windForce(t)
envelope = sin(pi*min(max((t-12)/16, 0), 1))^2;
force = envelope * [0.6 + 0.2*sin(0.8*t); -0.4 + 0.15*cos(0.6*t); 0.2];
end
