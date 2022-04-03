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
%CINEMÁTICA DIRECTA
CD=S01*S12*S23*S34*S45*S56*S67;
simplify(CD);

%Coordenadas cartesianas
%x=[xe,ye,ze,alfa,betha,gamma];
syms xe ye ze alfa betha gamma
 
%CINEMÁTICA INVERSA
%P07 -> Vector de posición que va de 0 a 7
P07=transl(xe,ye,ze)*rotz(gamma)*roty(betha)*roty(alfa);
%CD=P07;
%0=CD-P07
ECU=CD-P07
 
%Extracción de la matriz homogenea
%Contiene las variables articulares (q1,q2,...,q6)
%y las coordenadas del efector final (xe,ye,ze)
 
ecu1=ECU(1,1)
ecu2=ECU(1,2)
ecu3=ECU(1,3)
ecu4=ECU(1,4)
 
ecu5=ECU(2,1)
ecu6=ECU(2,2)
ecu7=ECU(2,3)
ecu8=ECU(2,4)
 
ecu9=ECU(3,1)
ecu10=ECU(3,2)
ecu11=ECU(3,3)
ecu12=ECU(3,4)
