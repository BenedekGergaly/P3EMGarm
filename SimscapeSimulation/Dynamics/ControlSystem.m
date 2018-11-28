function y = ControlSystem(inputs)
%ControlSystem Summary of this function goes here
%   Detailed explanation goes here


thetaDesired = [inputs(1); inputs(2); inputs(3)];
dThetaDesired = [inputs(4); inputs(5); inputs(6)];
ddThetaDesired = [inputs(7); inputs(8); inputs(9)];
theta = [inputs(10); inputs(11); inputs(12)];
dTheta = [inputs(13); inputs(14); inputs(15)];
time = inputs(16);

omega_n = 4.5;
%kp = omega_n^2;
%kv = 2 * omega_n;
kp = 40;
kv = 15;
ki = 3;

persistent lastTime;
persistent integralValue1;
persistent integralValue2;
persistent integralValue3;
if isempty(lastTime)
    lastTime = 0;
    integralValue1 = 0;
    integralValue2 = 0;
    integralValue3 = 0;
end

if (lastTime == 0)
    lastTime = time;
end

accelerations = [0 0 0];

% for i = 1:3
%    accelerations(i) = (thetaDesired(i) - theta(i)) * kp + (dThetaDesired(i) - dTheta(i)) * kv + ddThetaDesired(i);
% end
positionError = (thetaDesired(1) - theta(1));
integralValue1 = integralValue1 + positionError * (time - lastTime);
accelerations(1) = (thetaDesired(1) - theta(1)) * kp + (dThetaDesired(1) - dTheta(1)) * kv + integralValue1 * ki + ddThetaDesired(1);

positionError = (thetaDesired(2) - theta(2));
integralValue2 = integralValue2 + positionError * (time - lastTime);
accelerations(2) = (thetaDesired(2) - theta(2)) * kp + (dThetaDesired(2) - dTheta(2)) * kv + integralValue2 * ki + ddThetaDesired(2);

positionError = (thetaDesired(3) - theta(3));
integralValue3 = integralValue3 + positionError * (time - lastTime);
accelerations(3) = (thetaDesired(3) - theta(3)) * kp + (dThetaDesired(3) - dTheta(3)) * kv + integralValue3 * ki + ddThetaDesired(3);
lastTime = time;

y = accelerations; 
end

