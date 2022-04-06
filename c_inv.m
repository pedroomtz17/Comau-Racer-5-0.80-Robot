function F=c_inv(q)
q1=q(1); q2=q(2); q3=q(3); q4=q(4); q5=q(5); q6=q(6);
global xe ye ze alfa betha gamma
global l1 l2 l3 l4

%Hay 6 ecuaciones debido a que hay 
%6 variables articulares (qn)
F=[
%ecu1 =
- sin(q6)*(cos(q4)*sin(q1) - sin(q4)*(cos(q1)*sin(q3)*sin(q2 + pi/2) - cos(q1)*cos(q3)*cos(q2 + pi/2))) - cos(q6)*(cos(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*sin(q3)*sin(q2 + pi/2) - cos(q1)*cos(q3)*cos(q2 + pi/2))) + sin(q5)*(cos(q1)*cos(q3)*sin(q2 + pi/2) + cos(q1)*cos(q2 + pi/2)*sin(q3))) - cos(betha)*cos(gamma);
%ecu4 = 
l4*(sin(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*sin(q3)*sin(q2 + pi/2) - cos(q1)*cos(q3)*cos(q2 + pi/2))) - cos(q5)*(cos(q1)*cos(q3)*sin(q2 + pi/2) + cos(q1)*cos(q2 + pi/2)*sin(q3))) - xe - l3*(cos(q1)*cos(q3)*sin(q2 + pi/2) + cos(q1)*cos(q2 + pi/2)*sin(q3)) + l2*cos(q1)*cos(q2 + pi/2);
%ecu6 =
cos(q6)*(cos(q1)*cos(q4) + sin(q4)*(sin(q1)*sin(q3)*sin(q2 + pi/2) - cos(q3)*cos(q2 + pi/2)*sin(q1))) - sin(q6)*(cos(q5)*(cos(q1)*sin(q4) - cos(q4)*(sin(q1)*sin(q3)*sin(q2 + pi/2) - cos(q3)*cos(q2 + pi/2)*sin(q1))) - sin(q5)*(cos(q3)*sin(q1)*sin(q2 + pi/2) + cos(q2 + pi/2)*sin(q1)*sin(q3))) - cos(alfa)*cos(gamma) - sin(alfa)*sin(betha)*sin(gamma);
%ecu8 =
l2*cos(q2 + pi/2)*sin(q1) - l4*(sin(q5)*(cos(q1)*sin(q4) - cos(q4)*(sin(q1)*sin(q3)*sin(q2 + pi/2) - cos(q3)*cos(q2 + pi/2)*sin(q1))) + cos(q5)*(cos(q3)*sin(q1)*sin(q2 + pi/2) + cos(q2 + pi/2)*sin(q1)*sin(q3))) - l3*(cos(q3)*sin(q1)*sin(q2 + pi/2) + cos(q2 + pi/2)*sin(q1)*sin(q3)) - ye;
%ecu11 =
cos(q5)*(cos(q3)*cos(q2 + pi/2) - sin(q3)*sin(q2 + pi/2)) - cos(alfa)*cos(betha) - cos(q4)*sin(q5)*(cos(q3)*sin(q2 + pi/2) + cos(q2 + pi/2)*sin(q3));
%ecu12 =
l1 - ze + l3*(cos(q3)*cos(q2 + pi/2) - sin(q3)*sin(q2 + pi/2)) + l2*sin(q2 + pi/2) + l4*(cos(q5)*(cos(q3)*cos(q2 + pi/2) - sin(q3)*sin(q2 + pi/2)) - cos(q4)*sin(q5)*(cos(q3)*sin(q2 + pi/2) + cos(q2 + pi/2)*sin(q3)));
];
end