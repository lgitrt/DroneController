function state = navigationControllerState(filter, sample)
%NAVIGATIONCONTROLLERSTATE Convert EKF navigation state to plant ordering.
[~,E] = attitudeGeometry(filter.x(7:9));
rates = E\(sample.gyro-filter.x(13:15));
state = [filter.x(1:3);filter.x(7:9);filter.x(4:6);rates];
end
