clc; close all; clear all;
parametros

global xe ye ze alfa betha gamma

c=1;
q0=[0,0,1,0,0,1];%condición inicial
for i=0:10:360
  clf
  %Trayectoria de efector
  xe=0.35+sin(deg2rad(i))*0.085;
  ye=0.35+cos(deg2rad(i))*0.085;
  ze=0.65+cos(deg2rad(i))*0.085;
  
  alfa=0;
  betha=-pi/6;
  gamma=0;
  
  x(c)=xe; y(c)=ye; z(c)=ze;
  q=fsolve(@c_inv,q0);
  q1(c)=q(1); q2(c)=q(2); q3(c)=q(3); 
  q4(c)=q(4); q5(c)=q(5); q6(c)=q(6);
  DET(c)=-l2*l3*cos(q(3))*sin(q(5))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)));
  robot(q(1),q(2),q(3),q(4),q(5),q(6))
  hold on
  plot3(x,y,z,'b*')
  campos([1 1 1])
  pause(0.001)
  q0=q;
  c=c+1;
end
%Graficas de posición de las juntas
figure(2)
t=1:1:37;
plot(t,rad2deg(q1),'rd');
title('Espacio de juntas');
xlabel('Pasos');
ylabel('Grados');
grid on;
hold on
plot(t,rad2deg(q2),'gd');
plot(t,rad2deg(q3),'bd');
plot(t,rad2deg(q4),'r*');
plot(t,rad2deg(q5),'g*');
plot(t,rad2deg(q6),'b*');

figure(3)
plot(DET)
title('Determinante');
grid on

