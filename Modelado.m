%Modelado de Comau-Racer 5-0.80 Robot
clc; close all; clear all;
%Coordenadas articulares
%q=[q1,q2,q3,q4,q5,q6,q7];
syms q1 q2 q3 q4 q4 q5 q6
%Dimensiones de eslabones
syms l1 l2 l3 l4 pi
S01=DHC(0,0,q1,l1);
S12=DHC(pi/2,0,q2+pi/2,0);
S23=DHC(0,l2,q3,0);
S34=DHC(-pi/2,0,q4,l3);
S45=DHC(pi/2,0,q5,0);
S56=DHC(-pi/2,0,q6,0);
S67=DHC(0,0,0,l4);
%CINEM�?TICA DIRECTA
CD=S01*S12*S23*S34*S45*S56*S67;
simplify(CD);

%Coordenadas cartesianas
%x=[xe,ye,ze,alfa,betha,gamma];
syms xe ye ze alfa betha gamma
 
%CINEM�?TICA INVERSA
%P07 -> Vector de posición que va de 0 a 7
P07=transl(xe,ye,ze)*rotz(gamma)*roty(betha)*roty(alfa);
%CD=P07;
%0=CD-P07
ECU=CD-P07;
 
%Extracción de la matriz homogenea
%Contiene las variables articulares (q1,q2,...,q6)
%y las coordenadas del efector final (xe,ye,ze)
 
ecu1=ECU(1,1);
ecu2=ECU(1,2);
ecu3=ECU(1,3);
ecu4=ECU(1,4);
 
ecu5=ECU(2,1);
ecu6=ECU(2,2);
ecu7=ECU(2,3);
ecu8=ECU(2,4);
 
ecu9=ECU(3,1);
ecu10=ECU(3,2);
ecu11=ECU(3,3);
ecu12=ECU(3,4);

%%%%%%%% Modelo cinemático diferencial %%%%%%%%%
 
%Submatriz de rotación
R01=S01(1:3,1:3);
R12=S12(1:3,1:3);
R23=S23(1:3,1:3);
R34=S34(1:3,1:3);
R45=S34(1:3,1:3);
R56=S34(1:3,1:3);
R67=S34(1:3,1:3);
 
% %Vector de posición (px,py, pz)
P01=S01(1:3,4);
P12=S12(1:3,4);
P23=S23(1:3,4);
P34=S34(1:3,4);
P45=S34(1:3,4);
P56=S34(1:3,4);
P67=S34(1:3,4);
 
syms q1p q2p q3p q4p q5p q6p %velocidad de las articulaciones
 
v00=[0;0;0]; %velocidad de entrada
w00=[0;0;0]; %velocidad angular
Z=[0;0;1];
 
%velocidad lineal 1 vista desde el sistema 1
%es cero, puesto que el sistema 1 esta sobre un poste,
%el cual no se mueve
v11=transpose(R01)*(v00+cross(w00,P01));
%velocidad angular 1 vista desde el sistema 1
w11=transpose(R01)*w00+q1p*Z;

% %velocidad lineal 2 vista desde el sistema 2
v22=transpose(R12)*(v11+cross(w11,P12));
%velocidad angular 2 vista desde el sistema 2
w22=transpose(R12)*w11+q2p*Z;

% %velocidad lineal 3 vista desde el sistema 3
v33=transpose(R23)*(v22+cross(w22,P23));
%velocidad angular 3 vista desde el sistema 3
w33=transpose(R23)*w22+q3p*Z;

% %velocidad lineal 4 vista desde el sistema 4
v44=transpose(R34)*(v33+cross(w33,P34));
%velocidad angular 4 vista desde el sistema 4
w44=transpose(R34)*w33+q4p*Z;

% %velocidad lineal 5 vista desde el sistema 5
v55=transpose(R45)*(v44+cross(w44,P45));
%velocidad angular 5 vista desde el sistema 5
w55=transpose(R45)*w44+q5p*Z;

% %velocidad lineal 6 vista desde el sistema 6
v66=transpose(R56)*(v55+cross(w55,P56));
%velocidad angular 6 vista desde el sistema 6
w66=transpose(R56)*w55+q6p*Z;

% %velocidad lineal 7 vista desde el sistema 7
v77=transpose(R67)*(v66+cross(w66,P67));
%velocidad angular 7 vista desde el sistema 7
w77=transpose(R67)*w66+0*Z;
 
% % Cálculo del jacobiano
% % el numero de columnas es igual al numero de motores del robot
J77=[diff(v77(1),q1p) diff(v77(1),q2p) diff(v77(1),q3p) diff(v77(1),q4p) diff(v77(1),q5p) diff(v77(1),q6p);...
     diff(v77(2),q1p) diff(v77(2),q2p) diff(v77(2),q3p) diff(v77(2),q4p) diff(v77(2),q5p) diff(v77(2),q6p);...
     diff(v77(3),q1p) diff(v77(3),q2p) diff(v77(3),q3p) diff(v77(3),q4p) diff(v77(3),q5p) diff(v77(3),q6p);...    
     diff(w77(1),q1p) diff(w77(1),q2p) diff(w77(1),q3p) diff(w77(1),q4p) diff(w77(1),q5p) diff(w77(1),q6p);...
     diff(w77(2),q1p) diff(w77(2),q2p) diff(w77(2),q3p) diff(w77(2),q4p) diff(w77(2),q5p) diff(w77(2),q6p);...
     diff(w77(3),q1p) diff(w77(3),q2p) diff(w77(3),q3p) diff(w77(3),q4p) diff(w77(3),q5p) diff(w77(3),q6p)];
       
 simplify(J77);

% %Velocidad directa
% qp=[q1p;q2p;q3p;q4p;q5p;q6p]; %Velocidades articulares
% x77p=J77*qp;  %velocidad cartesiana relativa del efector final
% %    6x6 6x1 =6x1
 
% Velocidad inversa 
% inv(J77)*x77p;
 
% syms xe44p ye44p ze44p
% x77p=[xe44p;ye44p;ze44p];%velocidad cartesiana del efector final
 
%El rango determina el número de renglones y columnas linealmente 
%independientes.
rank(J77);
%Linealmente independientes -> Una ecuación no se puede deducir a partir 
%de otras
 
%Si hay una ecuación dependiente, significa que un grado de libertad se 
%pierde, es decir, se pierde un motor, lo cual implica que la matriz sea 
%singular (se atora el motor o demanda mucha potencia), por que el robot
%se vuelve incontrolable
 
% Calculo del determinante
 
DETJ77=det(J77);
simplify(DETJ77)

% Cinematica directa para el espacio de trabajo
x=CD(1,4)
y=CD(2,4)
z=CD(3,4)




