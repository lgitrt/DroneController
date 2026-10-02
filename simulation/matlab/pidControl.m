% Author: Luca Obwegs
function [u, memory] = pidControl(controller, memory, state, ref, p, Ts)
%PIDCONTROL Feedforward position PID -> attitude PID -> standard rotor mixer.
if isempty(memory)
    memory.positionIntegral = zeros(3,1);
    memory.attitudeIntegral = zeros(3,1);
    memory.previousDesired = ref.state(4:6);
end
positionError = ref.state(1:3)-state(1:3);
velocityError = ref.state(7:9)-state(7:9);
memory.positionIntegral = clip(memory.positionIntegral+Ts*positionError,controller.positionIntegralLimit);
acceleration = ref.acc+controller.positionKp.*positionError ...
    +controller.positionKd.*velocityError+controller.positionKi.*memory.positionIntegral;
acceleration = clip(acceleration,controller.accelerationLimit);
yaw = ref.state(6);
desired = [ ...
    (sin(yaw)*acceleration(1)-cos(yaw)*acceleration(2))/p.g; ...
    (cos(yaw)*acceleration(1)+sin(yaw)*acceleration(2))/p.g; yaw];
desired(1:2) = clip(desired(1:2),controller.attitudeLimit);
attitudeError = atan2(sin(desired-state(4:6)),cos(desired-state(4:6)));
memory.attitudeIntegral = clip(memory.attitudeIntegral+Ts*attitudeError,controller.attitudeIntegralLimit);
desiredRates = (desired-memory.previousDesired)/Ts;
memory.previousDesired = desired;
angleAcceleration = controller.attitudeKp.*attitudeError ...
    +controller.attitudeKd.*(desiredRates-state(10:12)) ...
    +controller.attitudeKi.*memory.attitudeIntegral;
torque = [p.Ix;p.Iy;p.Iz].*angleAcceleration;
thrust = p.m*(p.g+acceleration(3))/max(0.5,cos(state(4))*cos(state(5)));
[~,M] = mixingMatrix(p);
u = M\[thrust;torque];
end

function value = clip(value, limit)
value = min(max(value,-limit),limit);
end
