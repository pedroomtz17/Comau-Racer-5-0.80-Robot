%Modelado de Comau-Racer 5-0.80 Robot
syms q1 q2 q3 q4 q4 q5 q6
syms l1 l2 l3 l4 pi
S01=DHC(0,0,q1,l1);
S12=DHC(pi/2,0,q2+pi/2,0);
S23=DHC(0,l2,q3,0);
S34=DHC(-pi/2,0,q4,l3);
S45=DHC(pi/2,0,q5,0);
S56=DHC(-pi/2,0,q6,0);
S67=DHC(0,0,0,l4);
%CINEMÁTICA DIRECTA
CD=S01*S12*S23*S34*S45*S56*S67;
simplify(CD);