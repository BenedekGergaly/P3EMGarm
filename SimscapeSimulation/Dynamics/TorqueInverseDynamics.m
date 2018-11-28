function outputTorques = TorqueInverseDynamics(inputs)
%TORQUEINVERSEDYNAMICS Summary of this function goes here
%   Detailed explanation goes here

ddTheta = [inputs(1); inputs(2); inputs(3)];
dTheta = [inputs(4); inputs(5); inputs(6)];
theta = [inputs(7); inputs(8); inputs(9)];

torques = [0; 0; 0];
g = 9.815;
l = [0.067 0.224 0.149];
lc = [0.04 0.17670 0.13468];
m = [0.1956 0.227 0.285];

I1 = [0.00039968 0.00000007 -0.00000547;
    0.00000007 0.00037926 -0.00000609;
    -0.00000547 -0.00000609 0.00005511];
I2 = [0.00005997 -0.00014512 0.00002275;
    -0.00014512 0.00761485 -0.00000052;
    0.00002275 -0.00000052 0.00760132];
I3 = [0.00019476 -0.00014581 0.00003861;
    -0.00014581 0.00612172 0.00000075;
    0.00003861, -0.00000075, 0.00601887];

%gravity
torques(2) = -g * (sin(theta(2)) * l(2) * m(3) + sin(theta(2)) * lc(2) * m(2) + sin(theta(2) + theta(3)) * lc(3) * m(3));
torques(3) = -g * lc(3) * m(3) * sin(theta(2) + theta(3));

%inertia
m11 = ((-2*lc(3)^2*m(3)+2*I3(1, 1)-2*I3(2, 2))*cos(theta(3))^2+((-2*I3(1, 2)-2*I3(2, 1))*sin(theta(3))-2*m(3)*lc(3)*l(2))*cos(theta(3))+(-l(2)^2+lc(3)^2)*m(3)-m(2)*lc(2)^2-I2(2, 2)-I3(1, 1)+I3(2, 2)+I2(1, 1))*cos(theta(2))^2-((2*I3(1, 2)+2*I3(2, 1))*cos(theta(3))^2+2*sin(theta(3))*(-lc(3)^2*m(3)+I3(1, 1)-I3(2, 2))*cos(theta(3))-2*m(3)*lc(3)*sin(theta(3))*l(2)+I2(2, 1)-I3(1, 2)-I3(2, 1)+I2(1, 2))*sin(theta(2))*cos(theta(2))+(lc(3)^2*m(3)-I3(1, 1)+I3(2, 2))*cos(theta(3))^2+(sin(theta(3))*(I3(1, 2)+I3(2, 1))+2*m(3)*lc(3)*l(2))*cos(theta(3))+m(3)*l(2)^2+m(2)*lc(2)^2+I2(2, 2)+I3(1, 1)+I1(3, 3);
m12 = (1/2)*(cos(theta(3))*(I3(1, 3)+I3(3, 1))+(-I3(2, 3)-I3(3, 2))*sin(theta(3))+I2(1, 3)+I2(3, 1))*cos(theta(2))-(1/2)*sin(theta(2))*((I3(2, 3)+I3(3, 2))*cos(theta(3))+sin(theta(3))*(I3(1, 3)+I3(3, 1))+I2(2, 3)+I2(3, 2));
m13 = (1/2)*cos(theta(2))*(cos(theta(3))*(I3(1, 3)+I3(3, 1))-sin(theta(3))*(I3(2, 3)+I3(3, 2)))-(1/2)*((I3(2, 3)+I3(3, 2))*cos(theta(3))+sin(theta(3))*(I3(1, 3)+I3(3, 1)))*sin(theta(2));
m21 = (1/2)*(cos(theta(3))*(I3(1, 3)+I3(3, 1))+(-I3(2, 3)-I3(3, 2))*sin(theta(3))+I2(1, 3)+I2(3, 1))*cos(theta(2))-(1/2)*sin(theta(2))*((I3(2, 3)+I3(3, 2))*cos(theta(3))+sin(theta(3))*(I3(1, 3)+I3(3, 1))+I2(2, 3)+I2(3, 2));
m22 = 2*m(3)*l(2)*lc(3)*cos(theta(3))+(l(2)^2+lc(3)^2)*m(3)+m(2)*lc(2)^2+I2(3, 3)+I3(3, 3);
m23 = m(3)*l(2)*lc(3)*cos(theta(3))+lc(3)^2*m(3)+I3(3, 3);
m31 = (1/2)*cos(theta(2))*(cos(theta(3))*(I3(1, 3)+I3(3, 1))-sin(theta(3))*(I3(2, 3)+I3(3, 2)))-(1/2)*((I3(2, 3)+I3(3, 2))*cos(theta(3))+sin(theta(3))*(I3(1, 3)+I3(3, 1)))*sin(theta(2));
m32 = m(3)*l(2)*lc(3)*cos(theta(3))+lc(3)^2*m(3)+I3(3, 3);
m33 = lc(3)^2*m(3)+I3(3, 3);

M = [m11 m12 m13;
    m21 m22 m23;
    m31 m32 m33]

torques = torques + (M * ddTheta);

b11 = ((-4*I3(1, 2)-4*I3(2, 1))*cos(theta(3))^2-4*sin(theta(3))*(-lc(3)^2*m(3)+I3(1, 1)-I3(2, 2))*cos(theta(3))+4*m(3)*lc(3)*sin(theta(3))*l(2)+2*I3(1, 2)+2*I3(2, 1)-2*I2(1, 2)-2*I2(2, 1))*cos(theta(2))^2+(2*(2*lc(3)^2*m(3)-2*I3(1, 1)+2*I3(2, 2))*cos(theta(3))^2+2*((2*I3(1, 2)+2*I3(2, 1))*sin(theta(3))+2*m(3)*lc(3)*l(2))*cos(theta(3))+2*(l(2)^2-lc(3)^2)*m(3)+2*m(2)*lc(2)^2+2*I2(2, 2)+2*I3(1, 1)-2*I3(2, 2)-2*I2(1, 1))*sin(theta(2))*cos(theta(2))+(2*I3(1, 2)+2*I3(2, 1))*cos(theta(3))^2+2*sin(theta(3))*(-lc(3)^2*m(3)+I3(1, 1)-I3(2, 2))*cos(theta(3))-2*m(3)*lc(3)*sin(theta(3))*l(2)-I3(1, 2)-I3(2, 1)+I2(1, 2)+I2(2, 1);
b12 = ((-4*I3(1, 2)-4*I3(2, 1))*cos(theta(3))^2+4*sin(theta(3))*(lc(3)^2*m(3)-I3(1, 1)+I3(2, 2))*cos(theta(3))+2*m(3)*lc(3)*sin(theta(3))*l(2)+2*I3(1, 2)+2*I3(2, 1))*cos(theta(2))^2+4*sin(theta(2))*((lc(3)^2*m(3)-I3(1, 1)+I3(2, 2))*cos(theta(3))^2+(sin(theta(3))*(I3(1, 2)+I3(2, 1))+(1/2)*m(3)*lc(3)*l(2))*cos(theta(3))-(1/2)*lc(3)^2*m(3)+(1/2)*I3(1, 1)-(1/2)*I3(2, 2))*cos(theta(2))+(2*I3(1, 2)+2*I3(2, 1))*cos(theta(3))^2-2*sin(theta(3))*(lc(3)^2*m(3)-I3(1, 1)+I3(2, 2))*cos(theta(3))-2*m(3)*lc(3)*sin(theta(3))*l(2)-I3(1, 2)-I3(2, 1);
b13 = ((-I3(2, 3)-I3(3, 2))*cos(theta(3))-sin(theta(3))*(I3(1, 3)+I3(3, 1)))*cos(theta(2))+sin(theta(2))*((-I3(1, 3)-I3(3, 1))*cos(theta(3))+sin(theta(3))*(I3(2, 3)+I3(3, 2)));
b21 = 0;
b22 = 0;
b23 = -2*m(3)*lc(3)*sin(theta(3))*l(2);
b31 = 0;
b32 = 0;
b33 = 0;

B = [b11 b12 b13;
    b21 b22 b23;
    b31 b32 b33]

torques = torques + B * [dTheta(1) * dTheta(2); dTheta(1) * dTheta(3); dTheta(2) * dTheta(3)];

c11 = 0;
c12 = (1/2)*((-I3(2, 3)-I3(3, 2))*cos(theta(3))+(-I3(1, 3)-I3(3, 1))*sin(theta(3))-I2(2, 3)-I2(3, 2))*cos(theta(2))-(1/2)*sin(theta(2))*(cos(theta(3))*(I3(1, 3)+I3(3, 1))+(-I3(2, 3)-I3(3, 2))*sin(theta(3))+I2(1, 3)+I2(3, 1));
c13 = (1/2)*((-I3(2, 3)-I3(3, 2))*cos(theta(3))-sin(theta(3))*(I3(1, 3)+I3(3, 1)))*cos(theta(2))+(1/2)*sin(theta(2))*((-I3(1, 3)-I3(3, 1))*cos(theta(3))+sin(theta(3))*(I3(2, 3)+I3(3, 2)));
c21 = (1/2)*((4*I3(1, 2)+4*I3(2, 1))*cos(theta(3))^2+4*sin(theta(3))*(-lc(3)^2*m(3)+I3(1, 1)-I3(2, 2))*cos(theta(3))-4*m(3)*lc(3)*sin(theta(3))*l(2)-2*I3(1, 2)-2*I3(2, 1)+2*I2(1, 2)+2*I2(2, 1))*cos(theta(2))^2-((2*lc(3)^2*m(3)-2*I3(1, 1)+2*I3(2, 2))*cos(theta(3))^2+((2*I3(1, 2)+2*I3(2, 1))*sin(theta(3))+2*m(3)*lc(3)*l(2))*cos(theta(3))+(l(2)^2-lc(3)^2)*m(3)+m(2)*lc(2)^2+I2(2, 2)+I3(1, 1)-I3(2, 2)-I2(1, 1))*sin(theta(2))*cos(theta(2))+(1/2)*(-2*I3(1, 2)-2*I3(2, 1))*cos(theta(3))^2-sin(theta(3))*(-lc(3)^2*m(3)+I3(1, 1)-I3(2, 2))*cos(theta(3))+m(3)*lc(3)*sin(theta(3))*l(2)+(1/2)*I3(1, 2)+(1/2)*I3(2, 1)-(1/2)*I2(1, 2)-(1/2)*I2(2, 1);
c22 = 0;
c23 = -m(3)*lc(3)*sin(theta(3))*l(2);
c31 = (1/2)*((4*I3(1, 2)+4*I3(2, 1))*cos(theta(3))^2-4*sin(theta(3))*(lc(3)^2*m(3)-I3(1, 1)+I3(2, 2))*cos(theta(3))-2*m(3)*lc(3)*sin(theta(3))*l(2)-2*I3(1, 2)-2*I3(2, 1))*cos(theta(2))^2-2*sin(theta(2))*((lc(3)^2*m(3)-I3(1, 1)+I3(2, 2))*cos(theta(3))^2+(sin(theta(3))*(I3(1, 2)+I3(2, 1))+(1/2)*m(3)*lc(3)*l(2))*cos(theta(3))-(1/2)*lc(3)^2*m(3)+(1/2)*I3(1, 1)-(1/2)*I3(2, 2))*cos(theta(2))+(1/2)*(-2*I3(1, 2)-2*I3(2, 1))*cos(theta(3))^2+sin(theta(3))*(lc(3)^2*m(3)-I3(1, 1)+I3(2, 2))*cos(theta(3))+m(3)*lc(3)*sin(theta(3))*l(2)+(1/2)*I3(1, 2)+(1/2)*I3(2, 1);
c32 = m(3)*lc(3)*sin(theta(3))*l(2);
c33 = 0;

C = [c11 c12 c13;
    c21 c22 c23;
    c31 c32 c33]

torques = torques + C * [dTheta(1)^2; dTheta(2)^2; dTheta(3)^2]

%torques(1) = torques(1) + 0.08 * sign(dTheta(1));
%torques(2) = torques(2) + 0.08 * sign(dTheta(2));
%torques(3) = torques(3) + 0.08 * sign(dTheta(3));

outputTorques = [torques(1); torques(2); torques(3)]
end

