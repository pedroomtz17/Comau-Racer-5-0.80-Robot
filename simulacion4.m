clc; close all; clear all;
parametros

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Perfil quintico
% Univerdidad Nacional Autónoma de México
% Robótica
% Noé Alfredo Martínez Sánchez
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf=10; % tiempo final de la trayetoria
c=1;

% Posiciones iniciales y finales
pi=[0.6;0.4;0.3];
pf=[0.4;0.6;0.7];

%Orientaciones iniciales y finales
ai=[deg2rad(-5);deg2rad(0);deg2rad(5)];
af=[deg2rad(5);deg2rad(10);deg2rad(-5)];

%% Posición
for t=0:1/10:tf
   time(c)=t;
   Rt=pi+(10*(t/tf)^3-15*(t/tf)^4+6*(t/tf)^5)*(pf-pi);
   x(c)=Rt(1);
   y(c)=Rt(2);
   z(c)=Rt(3);
   c=c+1;
end

c=1;
for t=0:1/10:tf
   Rt=ai+(10*(t/tf)^3-15*(t/tf)^4+6*(t/tf)^5)*(af-ai);
   a1(c)=Rt(1);
   a2(c)=Rt(2);
   a3(c)=Rt(3);
   c=c+1;
end

figure(1)
plot(time,x,'r');
hold on
plot(time,y,'g');
hold on
plot(time,z,'b');
hold on
plot(time,a1,'r');
hold on
plot(time,a2,'g');
hold on
plot(time,a3,'b');
hold on
title('Posición espacio cartesiano')
xlabel('Tiempo[s]')
ylabel('Posición[m] Orientacion[rad]')
legend('x','y','z','\alpha','\beta','\gamma','orientation','horizontal','location','SouthEast')
grid on

%% Velocidad cartesiana
c=1;
for t=0:1/10:tf
   time(c)=t;
   Vt=(30*(t^2/tf^3)-60*(t^3/tf^4)+30*(t^4/tf^5))*(pf-pi);
   xv(c)=Vt(1);
   yv(c)=Vt(2);
   zv(c)=Vt(3);
   c=c+1;
end

c=1;
for t=0:1/10:tf
   Vt=(30*(t^2/tf^3)-60*(t^3/tf^4)+30*(t^4/tf^5))*(af-ai);
   a1p(c)=Vt(1);
   a2p(c)=Vt(2);
   a3p(c)=Vt(3);
   c=c+1;
end

figure(2)
plot(time,xv,'r');
hold on
plot(time,yv,'g');
hold on
plot(time,zv,'b');
hold on
plot(time,a1p,'r');
hold on
plot(time,a2p,'g');
hold on
plot(time,a3p,'b');
hold on
title('Velocidad espacio cartesiano')
xlabel('Tiempo [s]')
ylabel('Velocidad [m/s]-[rad/s]')
legend('x\prime','y\prime','z\prime','\alpha\prime','\beta\prime','\gamma\prime',...
    'orientation','horizontal','location','SouthEast')
grid on

%% Aceleración cartesiana
c=1;
for t=0:1/10:tf
   time(c)=t;
   At=(60*(t/tf^3)-180*(t^2/tf^4)+120*(t^3/tf^5))*(pf-pi);
   xa(c)=At(1);
   ya(c)=At(2);
   za(c)=At(3);
   c=c+1;
end

c=1;
for t=0:1/10:tf
   At=(60*(t/tf^3)-180*(t^2/tf^4)+120*(t^3/tf^5))*(af-ai);
   a1pp(c)=At(1);
   a2pp(c)=At(2);
   a3pp(c)=At(3);
   c=c+1;
end

figure(3)
plot(time,xa,'r');
hold on
plot(time,ya,'g');
hold on
plot(time,za,'b');
hold on
plot(time,a1pp,'r');
hold on
plot(time,a2pp,'g');
hold on
plot(time,a3pp,'b');
hold on
title('Aceleración espacio cartesiano')
xlabel('Tiempo [s]')
ylabel('Aceleración [m/s^2]-[rad/s^2]')
legend('x\prime\prime','y\prime\prime','z\prime\prime',...
    '\alpha\prime\prime','\beta\prime\prime','\gamma\prime\prime',...
    'orientation','horizontal','location','SouthEast')
grid on


figure(4)
global xe ye ze alfa betha gamma

c=1;
q0=[0,0,1,0,0,1];%condición inicial
for i=1:1:length(x)
  clf
  %Trayectoria de efector
  xe=x(i);
  ye=y(i);
  ze=z(i);
  
%   alfa=0;
%   betha=-pi/6;
%   gamma=0;
  
  q=fsolve(@c_inv,q0);
  q1(c)=q(1); q2(c)=q(2); q3(c)=q(3); 
  q4(c)=q(4); q5(c)=q(5); q6(c)=q(6);
  DET(c)=-l2*l3*cos(q(3))*sin(q(5))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)));
  %% Calculo de la velocidad articular
  xp=[xv(i);yv(i);zv(i)];%velocidad cartesiana
  %Jacobiano 77 inverso
  J77i=[                                                                                                                                                                                                                                                                                                                                                                                                               -(cos(q(4))*sin(q(6)) + cos(q(5))*cos(q(6))*sin(q(4)))/(l3*cos(q(2) + q(3)) + l2*sin(q(2))),                                                                                                                                                                                                                                                                                                                                                                                                               -(cos(q(4))*cos(q(6)) - cos(q(5))*sin(q(4))*sin(q(6)))/(l3*cos(q(2) + q(3)) + l2*sin(q(2))),                                                                                                                                                                                                                                                              (sin(q(4))*sin(q(5)))/(l3*cos(q(2) + q(3)) + l2*sin(q(2))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            -(l4*(cos(q(4))*cos(q(6)) - cos(q(5))*sin(q(4))*sin(q(6))))/(l3*cos(q(2) + q(3)) + l2*sin(q(2))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             (l4*(cos(q(4))*sin(q(6)) + cos(q(5))*cos(q(6))*sin(q(4))))/(l3*cos(q(2) + q(3)) + l2*sin(q(2))), 0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                              (cos(q(6))*sin(q(5)))/(l2*cos(q(3))),                                                                                                                                                                                                                                                                                                                                                                                                                                                           -(sin(q(5))*sin(q(6)))/(l2*cos(q(3))),                                                                                                                                                                                                                                                                                          cos(q(5))/(l2*cos(q(3))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          -(l4*sin(q(5))*sin(q(6)))/(l2*cos(q(3))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          -(l4*cos(q(6))*sin(q(5)))/(l2*cos(q(3))), 0;
                                                                                                                                                                                                                                                                                                                                                        -(l3*cos(q(6))*sin(q(5)) - l2*cos(q(3))*sin(q(4))*sin(q(6)) - l2*cos(q(6))*sin(q(3))*sin(q(5)) + l2*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6)))/(l2*l3*cos(q(3))),                                                                                                                                                                                                                                                                                                                                                       (l3*sin(q(5))*sin(q(6)) + l2*cos(q(3))*cos(q(6))*sin(q(4)) - l2*sin(q(3))*sin(q(5))*sin(q(6)) + l2*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(6)))/(l2*l3*cos(q(3))),                                                                                                                                                                                                                                (l2*cos(q(5))*sin(q(3)) - l3*cos(q(5)) + l2*cos(q(3))*cos(q(4))*sin(q(5)))/(l2*l3*cos(q(3))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    (l4*(l3*sin(q(5))*sin(q(6)) + l2*cos(q(3))*cos(q(6))*sin(q(4)) - l2*sin(q(3))*sin(q(5))*sin(q(6)) + l2*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(6))))/(l2*l3*cos(q(3))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    (l4*(l3*cos(q(6))*sin(q(5)) - l2*cos(q(3))*sin(q(4))*sin(q(6)) - l2*cos(q(6))*sin(q(3))*sin(q(5)) + l2*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6))))/(l2*l3*cos(q(3))), 0;
   -(l3*cos(q(3))*cos(q(5))*sin(q(2))*sin(q(3))*sin(q(6)) - l3*cos(q(2))*cos(q(3))^2*cos(q(5))*sin(q(6)) - l2*cos(q(3))*cos(q(5))*sin(q(2))*sin(q(6)) + l3*cos(q(5))*cos(q(6))*sin(q(2))*sin(q(4))*sin(q(5)) + l2*cos(q(3))*cos(q(4))^2*cos(q(5))*sin(q(2))*sin(q(6)) + l3*cos(q(3))^2*cos(q(4))*sin(q(2))*sin(q(5))*sin(q(6)) - l2*cos(q(5))*cos(q(6))*sin(q(2))*sin(q(3))*sin(q(4))*sin(q(5)) + l2*cos(q(3))*cos(q(4))*cos(q(5))^2*cos(q(6))*sin(q(2))*sin(q(4)) + l3*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(3))*sin(q(5))*sin(q(6)))/(l3*cos(q(3))*sin(q(5))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)))), -(l3*cos(q(3))*cos(q(5))*cos(q(6))*sin(q(2))*sin(q(3)) - l3*cos(q(2))*cos(q(3))^2*cos(q(5))*cos(q(6)) - l2*cos(q(3))*cos(q(5))*cos(q(6))*sin(q(2)) - l3*cos(q(5))*sin(q(2))*sin(q(4))*sin(q(5))*sin(q(6)) + l2*cos(q(3))*cos(q(4))^2*cos(q(5))*cos(q(6))*sin(q(2)) + l3*cos(q(3))^2*cos(q(4))*cos(q(6))*sin(q(2))*sin(q(5)) + l2*cos(q(5))*sin(q(2))*sin(q(3))*sin(q(4))*sin(q(5))*sin(q(6)) - l2*cos(q(3))*cos(q(4))*cos(q(5))^2*sin(q(2))*sin(q(4))*sin(q(6)) + l3*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(6))*sin(q(3))*sin(q(5)))/(l3*cos(q(3))*sin(q(5))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)))),                                                                                          (sin(q(4))*(l3*cos(q(3))^2*sin(q(2)) - l3*cos(q(5))^2*sin(q(2)) + l3*cos(q(2))*cos(q(3))*sin(q(3)) + l2*cos(q(5))^2*sin(q(2))*sin(q(3)) + l2*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2))*sin(q(5))))/(l3*cos(q(3))*sin(q(5))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)))), (cos(q(2))*cos(q(6))*l3^2*cos(q(3))^2 - cos(q(6))*sin(q(2))*sin(q(3))*l3^2*cos(q(3)) - l4*cos(q(6))*sin(q(2))*sin(q(5))*l3*cos(q(3))^2*cos(q(4)) + l4*cos(q(2))*cos(q(6))*l3*cos(q(3))^2*cos(q(5)) - l4*cos(q(2))*cos(q(6))*sin(q(3))*sin(q(5))*l3*cos(q(3))*cos(q(4)) - l4*cos(q(6))*sin(q(2))*sin(q(3))*l3*cos(q(3))*cos(q(5)) + l2*cos(q(6))*sin(q(2))*l3*cos(q(3)) + l4*sin(q(2))*sin(q(4))*sin(q(5))*sin(q(6))*l3*cos(q(5)) - l2*l4*cos(q(6))*sin(q(2))*cos(q(3))*cos(q(4))^2*cos(q(5)) + l2*l4*sin(q(2))*sin(q(4))*sin(q(6))*cos(q(3))*cos(q(4))*cos(q(5))^2 + l2*l4*cos(q(6))*sin(q(2))*cos(q(3))*cos(q(5)) - l2*l4*sin(q(2))*sin(q(3))*sin(q(4))*sin(q(5))*sin(q(6))*cos(q(5)))/(l3*cos(q(3))*sin(q(5))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)))), (l3^2*cos(q(3))*sin(q(2))*sin(q(3))*sin(q(6)) - l3^2*cos(q(2))*cos(q(3))^2*sin(q(6)) - l2*l3*cos(q(3))*sin(q(2))*sin(q(6)) - l2*l4*cos(q(3))*cos(q(5))*sin(q(2))*sin(q(6)) - l3*l4*cos(q(2))*cos(q(3))^2*cos(q(5))*sin(q(6)) + l3*l4*cos(q(3))*cos(q(5))*sin(q(2))*sin(q(3))*sin(q(6)) + l3*l4*cos(q(5))*cos(q(6))*sin(q(2))*sin(q(4))*sin(q(5)) + l2*l4*cos(q(3))*cos(q(4))^2*cos(q(5))*sin(q(2))*sin(q(6)) + l3*l4*cos(q(3))^2*cos(q(4))*sin(q(2))*sin(q(5))*sin(q(6)) + l3*l4*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(3))*sin(q(5))*sin(q(6)) - l2*l4*cos(q(5))*cos(q(6))*sin(q(2))*sin(q(3))*sin(q(4))*sin(q(5)) + l2*l4*cos(q(3))*cos(q(4))*cos(q(5))^2*cos(q(6))*sin(q(2))*sin(q(4)))/(l3*cos(q(3))*sin(q(5))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)))), 0;
                                                                          -(l3*cos(q(3))*cos(q(5))*cos(q(6))*sin(q(2))*sin(q(3)) - l3*cos(q(2))*cos(q(3))^2*cos(q(5))*cos(q(6)) - l3*cos(q(4))*cos(q(6))*sin(q(2))*sin(q(5)) + l2*cos(q(3))*cos(q(4))*sin(q(2))*sin(q(4))*sin(q(6)) + l2*cos(q(4))*cos(q(6))*sin(q(2))*sin(q(3))*sin(q(5)) - l2*cos(q(3))*cos(q(4))^2*cos(q(5))*cos(q(6))*sin(q(2)) + l3*cos(q(3))^2*cos(q(4))*cos(q(6))*sin(q(2))*sin(q(5)) + l3*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(6))*sin(q(3))*sin(q(5)))/(l3*cos(q(3))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)))),                                                                        -(l3*cos(q(4))*sin(q(2))*sin(q(5))*sin(q(6)) + l3*cos(q(2))*cos(q(3))^2*cos(q(5))*sin(q(6)) + l2*cos(q(3))*cos(q(4))*cos(q(6))*sin(q(2))*sin(q(4)) - l3*cos(q(3))*cos(q(5))*sin(q(2))*sin(q(3))*sin(q(6)) - l2*cos(q(4))*sin(q(2))*sin(q(3))*sin(q(5))*sin(q(6)) + l2*cos(q(3))*cos(q(4))^2*cos(q(5))*sin(q(2))*sin(q(6)) - l3*cos(q(3))^2*cos(q(4))*sin(q(2))*sin(q(5))*sin(q(6)) - l3*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(3))*sin(q(5))*sin(q(6)))/(l3*cos(q(3))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)))), -(l3*cos(q(2))*cos(q(3))^2*sin(q(5)) - l3*cos(q(4))*cos(q(5))*sin(q(2)) + l2*cos(q(4))*cos(q(5))*sin(q(2))*sin(q(3)) - l3*cos(q(3))*sin(q(2))*sin(q(3))*sin(q(5)) + l3*cos(q(3))^2*cos(q(4))*cos(q(5))*sin(q(2)) + l2*cos(q(3))*cos(q(4))^2*sin(q(2))*sin(q(5)) + l3*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(3)))/(l3*cos(q(3))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)))),                                                                          -(l3^2*cos(q(2))*cos(q(3))^2*sin(q(6)) - l3^2*cos(q(3))*sin(q(2))*sin(q(3))*sin(q(6)) + l2*l3*cos(q(3))*sin(q(2))*sin(q(6)) + l3*l4*cos(q(4))*sin(q(2))*sin(q(5))*sin(q(6)) + l3*l4*cos(q(2))*cos(q(3))^2*cos(q(5))*sin(q(6)) + l2*l4*cos(q(3))*cos(q(4))*cos(q(6))*sin(q(2))*sin(q(4)) - l3*l4*cos(q(3))*cos(q(5))*sin(q(2))*sin(q(3))*sin(q(6)) - l2*l4*cos(q(4))*sin(q(2))*sin(q(3))*sin(q(5))*sin(q(6)) + l2*l4*cos(q(3))*cos(q(4))^2*cos(q(5))*sin(q(2))*sin(q(6)) - l3*l4*cos(q(3))^2*cos(q(4))*sin(q(2))*sin(q(5))*sin(q(6)) - l3*l4*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(3))*sin(q(5))*sin(q(6)))/(l3*cos(q(3))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)))),                                                                           (l3^2*cos(q(3))*cos(q(6))*sin(q(2))*sin(q(3)) - l3^2*cos(q(2))*cos(q(3))^2*cos(q(6)) - l2*l3*cos(q(3))*cos(q(6))*sin(q(2)) - l3*l4*cos(q(4))*cos(q(6))*sin(q(2))*sin(q(5)) - l3*l4*cos(q(2))*cos(q(3))^2*cos(q(5))*cos(q(6)) + l3*l4*cos(q(3))*cos(q(5))*cos(q(6))*sin(q(2))*sin(q(3)) + l2*l4*cos(q(3))*cos(q(4))*sin(q(2))*sin(q(4))*sin(q(6)) + l2*l4*cos(q(4))*cos(q(6))*sin(q(2))*sin(q(3))*sin(q(5)) - l2*l4*cos(q(3))*cos(q(4))^2*cos(q(5))*cos(q(6))*sin(q(2)) + l3*l4*cos(q(3))^2*cos(q(4))*cos(q(6))*sin(q(2))*sin(q(5)) + l3*l4*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(6))*sin(q(3))*sin(q(5)))/(l3*cos(q(3))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)))), 0;
                                                     -(l2*cos(q(3))*sin(q(2))*sin(q(6)) + l3*cos(q(2))*cos(q(3))^2*sin(q(6)) - l3*cos(q(3))*sin(q(2))*sin(q(3))*sin(q(6)) - l3*cos(q(6))*sin(q(2))*sin(q(4))*sin(q(5)) - l2*cos(q(3))*cos(q(4))^2*sin(q(2))*sin(q(6)) + l2*cos(q(6))*sin(q(2))*sin(q(3))*sin(q(4))*sin(q(5)) + l3*cos(q(3))^2*cos(q(6))*sin(q(2))*sin(q(4))*sin(q(5)) - l2*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(2))*sin(q(4)) + l3*cos(q(2))*cos(q(3))*cos(q(6))*sin(q(3))*sin(q(4))*sin(q(5)))/(l3*cos(q(3))*sin(q(5))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)))),                                                    (l3*cos(q(3))*cos(q(6))*sin(q(2))*sin(q(3)) - l3*cos(q(2))*cos(q(3))^2*cos(q(6)) - l2*cos(q(3))*cos(q(6))*sin(q(2)) - l3*sin(q(2))*sin(q(4))*sin(q(5))*sin(q(6)) + l2*cos(q(3))*cos(q(4))^2*cos(q(6))*sin(q(2)) + l2*sin(q(2))*sin(q(3))*sin(q(4))*sin(q(5))*sin(q(6)) + l3*cos(q(3))^2*sin(q(2))*sin(q(4))*sin(q(5))*sin(q(6)) + l3*cos(q(2))*cos(q(3))*sin(q(3))*sin(q(4))*sin(q(5))*sin(q(6)) - l2*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2))*sin(q(4))*sin(q(6)))/(l3*cos(q(3))*sin(q(5))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)))),                                                                                     -(sin(q(4))*(l2*cos(q(5))*sin(q(2))*sin(q(3)) - l3*cos(q(5))*sin(q(2)) + l3*cos(q(3))^2*cos(q(5))*sin(q(2)) + l3*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(3)) + l2*cos(q(3))*cos(q(4))*sin(q(2))*sin(q(5))))/(l3*cos(q(3))*sin(q(5))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)))),                          -(l3*l4*cos(q(2))*cos(q(3))^2*cos(q(6)) + l2*l4*cos(q(3))*cos(q(6))*sin(q(2)) + l3^2*cos(q(2))*cos(q(3))^2*cos(q(5))*cos(q(6)) - l3^2*cos(q(3))*cos(q(5))*cos(q(6))*sin(q(2))*sin(q(3)) + l2*l3*cos(q(3))*cos(q(5))*cos(q(6))*sin(q(2)) - l3*l4*cos(q(3))*cos(q(6))*sin(q(2))*sin(q(3)) + l3*l4*sin(q(2))*sin(q(4))*sin(q(5))*sin(q(6)) - l2*l4*cos(q(3))*cos(q(4))^2*cos(q(6))*sin(q(2)) - l2*l4*sin(q(2))*sin(q(3))*sin(q(4))*sin(q(5))*sin(q(6)) - l3*l4*cos(q(3))^2*sin(q(2))*sin(q(4))*sin(q(5))*sin(q(6)) + l2*l4*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2))*sin(q(4))*sin(q(6)) - l3*l4*cos(q(2))*cos(q(3))*sin(q(3))*sin(q(4))*sin(q(5))*sin(q(6)))/(l3*cos(q(3))*sin(q(5))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)))),                           (l3*l4*cos(q(2))*cos(q(3))^2*sin(q(6)) + l2*l4*cos(q(3))*sin(q(2))*sin(q(6)) + l3^2*cos(q(2))*cos(q(3))^2*cos(q(5))*sin(q(6)) - l2*l4*cos(q(3))*cos(q(4))^2*sin(q(2))*sin(q(6)) - l3^2*cos(q(3))*cos(q(5))*sin(q(2))*sin(q(3))*sin(q(6)) + l2*l3*cos(q(3))*cos(q(5))*sin(q(2))*sin(q(6)) - l3*l4*cos(q(3))*sin(q(2))*sin(q(3))*sin(q(6)) - l3*l4*cos(q(6))*sin(q(2))*sin(q(4))*sin(q(5)) + l2*l4*cos(q(6))*sin(q(2))*sin(q(3))*sin(q(4))*sin(q(5)) + l3*l4*cos(q(3))^2*cos(q(6))*sin(q(2))*sin(q(4))*sin(q(5)) - l2*l4*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(2))*sin(q(4)) + l3*l4*cos(q(2))*cos(q(3))*cos(q(6))*sin(q(3))*sin(q(4))*sin(q(5)))/(l3*cos(q(3))*sin(q(5))*(l3*cos(q(2) + q(3)) + l2*sin(q(2)))), 1];
  qp=J77i*xp;%mapeo de la velocidad cartesiana a velocidad articular                                             
  robot(q(1),q(2),q(3),q(4),q(5),q(6))
  hold on
  plot3(x,y,z,'b*')
  campos([1 1 1])
  pause(0.001)
  q0=q;
  c=c+1;
end
%Graficas de posición de las juntas
figure(5)
plot(time,rad2deg(q1),'rd');
title('Espacio de juntas');
xlabel('Pasos');
ylabel('Grados');
grid on;
hold on
plot(time,rad2deg(q2),'gd');
plot(time,rad2deg(q3),'bd');
plot(time,rad2deg(q4),'r*');
plot(time,rad2deg(q5),'g*');
plot(time,rad2deg(q6),'b*');

figure(6)
plot(DET)
title('Determinante');
grid on

