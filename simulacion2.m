clc; close all; clear all;
parametros

global xe ye ze alfa betha gamma

%Posici�n y orientaci�n de efector
xe=0.45;ye=0.45;ze=0.45; 
alfa=0; betha=0;gamma=0;

q0=[0,0,1,0,0,1]; %Condici�n inicial
q=fsolve(@c_inv,q0)%Funci�n para el m�todo num�rico
robot(q(1),q(2),q(3),q(4),q(5),q(6))