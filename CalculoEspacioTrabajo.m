% Calculo del espacio de trabajo
clc; close all; clear all;
 
parametros 
%  Metodo 2
% 1.-Hacer un barrido de las coordenadas articulares
% 2.-Evaluar el determinante con las combinaciones
% 3.-Almacenar puntos cartesianos validos, es decir, donde el 
%    determinante es mayor a cero(Cinematica directa)
% 4.-Dibujar la nube de puntos
 
tic
inc=50;
c=1;
 
for Q1=0:inc:360 
    for Q2=0:inc:360 
        for Q3=0:inc:360 
            for Q4=0:inc:360
                for Q5=0:inc:360
                    for Q6=0:inc:360
            
                        q1=deg2rad(Q1); q2=deg2rad(Q2); q3=deg2rad(Q3);
                        q4=deg2rad(Q4); q5=deg2rad(Q5); q6=deg2rad(Q6);
 
                        DET=-l2*l3*cos(q3)*sin(q5)*(l3*cos(q2 + q3) + l2*sin(q2));
                        if abs(DET)>0.035
                          P(c,1)=l4*(sin(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*sin(q3)*sin(q2 + pi/2) - cos(q1)*cos(q3)*cos(q2 + pi/2))) - cos(q5)*(cos(q1)*cos(q3)*sin(q2 + pi/2) + cos(q1)*cos(q2 + pi/2)*sin(q3))) - l3*(cos(q1)*cos(q3)*sin(q2 + pi/2) + cos(q1)*cos(q2 + pi/2)*sin(q3)) + l2*cos(q1)*cos(q2 + pi/2); %X
                          P(c,2)=l2*cos(q2 + pi/2)*sin(q1) - l3*(cos(q3)*sin(q1)*sin(q2 + pi/2) + cos(q2 + pi/2)*sin(q1)*sin(q3)) - l4*(sin(q5)*(cos(q1)*sin(q4) - cos(q4)*(sin(q1)*sin(q3)*sin(q2 + pi/2) - cos(q3)*cos(q2 + pi/2)*sin(q1))) + cos(q5)*(cos(q3)*sin(q1)*sin(q2 + pi/2) + cos(q2 + pi/2)*sin(q1)*sin(q3))); %Y
                          P(c,3)=l1 + l3*(cos(q3)*cos(q2 + pi/2) - sin(q3)*sin(q2 + pi/2)) + l2*sin(q2 + pi/2) + l4*(cos(q5)*(cos(q3)*cos(q2 + pi/2) - sin(q3)*sin(q2 + pi/2)) - cos(q4)*sin(q5)*(cos(q3)*sin(q2 + pi/2) + cos(q2 + pi/2)*sin(q3))); %Z
                          c=c+1;
                        end
                  end
               end
            end
         end
    end
end
 
plot3(P(:,1) ,P(:,2) ,P(:,3) ,'r.')
% hold on
grid on
% robot(0,0,0,0,0,0)

figure(2)
k = boundary(P,0);
j = boundary(P,1);
 
subplot(1,2,1);
plot3(P(:,1),P(:,2),P(:,3),'.','MarkerSize',10)
hold on
trisurf(k,P(:,1),P(:,2),P(:,3),'FaceColor','red','FaceAlpha',0.1)
axis equal
title('Shrink Factor = 0')
grid on
 
subplot(1,2,2);
plot3(P(:,1),P(:,2),P(:,3),'.','MarkerSize',10)
hold on
trisurf(j,P(:,1),P(:,2),P(:,3),'FaceColor','red','FaceAlpha',0.1)
axis equal
title('Shrink Factor = 1')
grid on
 
figure(3)
DT = delaunayTriangulation(P);
[K,v] = convexHull(DT);
volumen=v % 1.5429 [m^3]
trisurf(K,DT.Points(:,1),DT.Points(:,2),DT.Points(:,3),...
       'FaceColor','cyan')

 
toc
