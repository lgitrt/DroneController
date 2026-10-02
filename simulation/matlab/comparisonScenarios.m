function scenarios = comparisonScenarios()
%COMPARISONSCENARIOS Reproducible ideal and unmodelled wind/motor-lag cases.
ideal.name = 'Ideal baseline';
ideal.slug = 'ideal';
ideal.motorTimeConstant = 0;
ideal.maxInputStep = 0.2;
ideal.windForce = @(t) zeros(3,1);

robust = ideal;
robust.name = 'Wind and motor lag';
robust.slug = 'robustness';
robust.motorTimeConstant = 0.04;
robust.windForce = @windForce;
scenarios = [ideal, robust];
end

function force = windForce(t)
envelope = sin(pi*min(max((t-12)/16, 0), 1))^2;
force = envelope * [0.6 + 0.2*sin(0.8*t); -0.4 + 0.15*cos(0.6*t); 0.2];
end
