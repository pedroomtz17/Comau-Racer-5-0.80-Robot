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
inc=35;
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
                        if abs(DET)>0.04
                          x(c) =l4*(sin(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*sin(q3)*sin(q2 + pi/2) - cos(q1)*cos(q3)*cos(q2 + pi/2))) - cos(q5)*(cos(q1)*cos(q3)*sin(q2 + pi/2) + cos(q1)*cos(q2 + pi/2)*sin(q3))) - l3*(cos(q1)*cos(q3)*sin(q2 + pi/2) + cos(q1)*cos(q2 + pi/2)*sin(q3)) + l2*cos(q1)*cos(q2 + pi/2); %X
                          y(c) =l2*cos(q2 + pi/2)*sin(q1) - l3*(cos(q3)*sin(q1)*sin(q2 + pi/2) + cos(q2 + pi/2)*sin(q1)*sin(q3)) - l4*(sin(q5)*(cos(q1)*sin(q4) - cos(q4)*(sin(q1)*sin(q3)*sin(q2 + pi/2) - cos(q3)*cos(q2 + pi/2)*sin(q1))) + cos(q5)*(cos(q3)*sin(q1)*sin(q2 + pi/2) + cos(q2 + pi/2)*sin(q1)*sin(q3))); %Y
                          z(c) =l1 + l3*(cos(q3)*cos(q2 + pi/2) - sin(q3)*sin(q2 + pi/2)) + l2*sin(q2 + pi/2) + l4*(cos(q5)*(cos(q3)*cos(q2 + pi/2) - sin(q3)*sin(q2 + pi/2)) - cos(q4)*sin(q5)*(cos(q3)*sin(q2 + pi/2) + cos(q2 + pi/2)*sin(q3))); %Z
                          c=c+1;
                        end
                  end
               end
            end
         end
    end
end
 
plot3(x,y,z,'r.')
grid on 
 
toc
