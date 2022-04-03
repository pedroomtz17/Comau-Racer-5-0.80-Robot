%Simulación 1
clc; close all; clear all;
parametros
for i=0:10:360
    clf
    robot(deg2rad(i),0,0,0,0,0)
    pause(0.001)
end

for i=0:10:360
    clf
    robot(0,deg2rad(i),0,0,0,0)
    pause(0.001)
end

for i=0:10:360
    clf
    robot(0,0,deg2rad(i),0,0,0)
    pause(0.001)
end

for i=0:10:360
    clf
    robot(0,0,0,deg2rad(i),0,0)
    pause(0.001)
end

for i=0:10:360
    clf
    robot(0,0,0,0,deg2rad(i),0)
    pause(0.001)
end

for i=0:10:360
    clf
    robot(0,0,0,0,0,deg2rad(i))
    pause(0.001)
end