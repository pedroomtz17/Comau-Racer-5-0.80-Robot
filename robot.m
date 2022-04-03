%Función robot
function []=robot(q1,q2,q3,q4,q5,q6)
global l1 l2 l3 l4
S01=DHC(0,0,q1,l1);
S12=DHC(-pi/2,0,q2-pi/2,0);
S23=DHC(0,l2,q3,0);
S34=DHC(-pi/2,0,q4,l3);
S45=DHC(-pi/2,0,q5,0);
S56=DHC(-pi/2,0,q6,0);
S67=DHC(0,0,0,l4);
%CINEMÁTICA DIRECTA
CD=S01*S12*S23*S34*S45*S56*S67;

%Vectores de posición
S02=S01*S12;
S03=S01*S12*S23;
S04=S01*S12*S23*S34;
S05=S01*S12*S23*S34*S45;
S06=S01*S12*S23*S34*S45*S56;
S07=S01*S12*S23*S34*S45*S56*S67;


x=[0 S01(1,4) S02(1,4) S03(1,4) S04(1,4) S05(1,4) S06(1,4) S07(1,4)];
y=[0 S01(2,4) S02(2,4) S03(2,4) S04(2,4) S05(2,4) S06(2,4) S07(2,4)];
z=[0 S01(3,4) S02(3,4) S03(3,4) S04(3,4) S05(3,4) S06(3,4) S07(3,4)];
plot3(x,y,z)
hold on
%Sistemas de referencia
frame(eye(4),'r',0.2)
frame(S01,'r',0.2)
frame(S02,'g',0.2)
frame(S03,'b',0.2)
frame(S04,'r',0.2)
frame(S05,'g',0.2)
frame(S06,'b',0.2)
frame(S07,'c',0.2)

axis([-1 1 -1 1 -1 1])
grid on
end