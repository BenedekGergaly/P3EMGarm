function y = ControlSystem(inputs)
%TORQUEFORWARDDYNAMICS Summary of this function goes here
%   Detailed explanation goes here


thetaDesired = [inputs(1); inputs(2); inputs(3)];
dThetaDesired = [inputs(4); inputs(5); inputs(6)];
ddThetaDesired = [inputs(7); inputs(8); inputs(9)];
theta = [inputs(10); inputs(11); inputs(12)];
dTheta = [inputs(13); inputs(14); inputs(15)];

kp = 484;
kv = 40;

accelerations = [0 0 0];

for i = 1:3
    accelerations(i) = (thetaDesired(i) - theta(i)) * kp + (dThetaDesired(i) - dTheta(i)) * kv + ddThetaDesired(i);
end


y = accelerations; 
end

