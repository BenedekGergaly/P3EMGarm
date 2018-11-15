function outputTorques = TorqueForwardDynamics(inputs)
%TORQUEFORWARDDYNAMICS Summary of this function goes here
%   Detailed explanation goes here

ddTheta = [inputs(1) inputs(2) inputs(3)];
dThetaFeedback = [inputs(4) inputs(5) inputs(6)];
thetaFeedback = [inputs(7) inputs(8) inputs(9)];

torques = [0 0 0];
g = 9.80665;
l = [0.067 0.224 0.149];
lc = [0.04 0.17670 0.13468];
m = [0.1956 0.227 0.285];

%gravity
torques(2) = g * (-sin(thetaFeedback(2)) * l(2) * m(3) - sin(thetaFeedback(2)) * lc(2) * m(2) - sin(thetaFeedback(2) + thetaFeedback(3)) * lc(3) * m(3));
torques(3) = g * lc(3) * m(3) * -sin(thetaFeedback(2) + thetaFeedback(3));

outputTorques = [torques(1) torques(2) torques(3)]
end
