% Author: Luca Obwegs
function controller = designPID()
%DESIGNPID Cascaded position PID and attitude PID with physical mixing.
controller.type = 'PID';
controller.positionKp = [2;2;3];
controller.positionKi = [0.3;0.3;0.8];
controller.positionKd = [2.5;2.5;2.8];
controller.attitudeKp = [22;22;10];
controller.attitudeKi = [2;2;1];
controller.attitudeKd = [7;7;5];
controller.positionIntegralLimit = [2;2;1];
controller.attitudeIntegralLimit = deg2rad([10;10;10]);
controller.accelerationLimit = [3;3;4];
controller.attitudeLimit = deg2rad(25);
end
