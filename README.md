# Robótica
## _Robot Comau-Racer 5-0.80_

[![N|Solid](https://media.giphy.com/media/zpuLFclMqtnc04zWSj/giphy.gif)](https://www.comau.com/en/competencies/robotics-automation/robot-team/racer-5-0-80/)

## Parámetros Denavit-Hartenberg 

<img src="/Imagenes/Denavit-Hartenberg.PNG" width="561" height="385">

## Expresión de Cinemática Directa [(Simulación 1)](https://github.com/pedroomtz17/Comau-Racer-5-0.80-Robot/blob/master/simulacion1.m).

<img src="https://media.giphy.com/media/QSSbQwaM9npmFEg6op/giphy.gif" width="560" height="420">



## Expresión de Cinemática Inversa [(Simulación 2)](https://github.com/pedroomtz17/Comau-Racer-5-0.80-Robot/blob/master/simulacion2.m).


- Solución inicial (un solo punto para el efector)
<img src="/Imagenes/simulacion2.png" width="440" height="300">
<img src="/Imagenes/simulacion2.1.png" width="440" height="300">

- Trayectoria [(Simulación 3)](https://github.com/pedroomtz17/Comau-Racer-5-0.80-Robot/blob/master/simulacion3.m).
<img src="/Imagenes/simulacion3.png" width="440" height="300">

## Graficas de posición de las juntas.
<img src="/Imagenes/EspacioJuntas.png" width="440" height="300">

## Jacobiano por el método de propagación de velocidades
```sh
J77=[diff(v77(1),q1p) diff(v77(1),q2p) diff(v77(1),q3p) diff(v77(1),q4p) diff(v77(1),q5p) diff(v77(1),q6p);...
     diff(v77(2),q1p) diff(v77(2),q2p) diff(v77(2),q3p) diff(v77(2),q4p) diff(v77(2),q5p) diff(v77(2),q6p);...
     diff(v77(3),q1p) diff(v77(3),q2p) diff(v77(3),q3p) diff(v77(3),q4p) diff(v77(3),q5p) diff(v77(3),q6p);...    
     diff(w77(1),q1p) diff(w77(1),q2p) diff(w77(1),q3p) diff(w77(1),q4p) diff(w77(1),q5p) diff(w77(1),q6p);...
     diff(w77(2),q1p) diff(w77(2),q2p) diff(w77(2),q3p) diff(w77(2),q4p) diff(w77(2),q5p) diff(w77(2),q6p);...
     diff(w77(3),q1p) diff(w77(3),q2p) diff(w77(3),q3p) diff(w77(3),q4p) diff(w77(3),q5p) diff(w77(3),q6p)];
     
simplify(J77)
     
ans =
 
[ - l4*(cos(q2 + q3)*cos(q6)*sin(q4) - sin(q2 + q3)*sin(q5)*sin(q6) + cos(q2 + q3)*cos(q4)*cos(q5)*sin(q6)) - cos(q4)*sin(q6)*(l3*cos(q2 + q3) + l2*sin(q2)) - cos(q5)*cos(q6)*sin(q4)*(l3*cos(q2 + q3) + l2*sin(q2)), sin(q4)*sin(q6)*(l3 - l2*sin(q3)) - cos(q6)*(cos(q4)*cos(q5)*(l3 - l2*sin(q3)) - l2*cos(q3)*sin(q5)) - l4*(cos(q4)*cos(q6) - cos(q5)*sin(q4)*sin(q6)), l3*sin(q4)*sin(q6) - l4*(cos(q4)*cos(q6) - cos(q5)*sin(q4)*sin(q6)) - l3*cos(q4)*cos(q5)*cos(q6), -l4*sin(q5)*sin(q6), -l4*cos(q6), 0]
[   l4*(cos(q2 + q3)*sin(q4)*sin(q6) + sin(q2 + q3)*cos(q6)*sin(q5) - cos(q2 + q3)*cos(q4)*cos(q5)*cos(q6)) - cos(q4)*cos(q6)*(l3*cos(q2 + q3) + l2*sin(q2)) + cos(q5)*sin(q4)*sin(q6)*(l3*cos(q2 + q3) + l2*sin(q2)), l4*(cos(q4)*sin(q6) + cos(q5)*cos(q6)*sin(q4)) + sin(q6)*(cos(q4)*cos(q5)*(l3 - l2*sin(q3)) - l2*cos(q3)*sin(q5)) + cos(q6)*sin(q4)*(l3 - l2*sin(q3)), l4*(cos(q4)*sin(q6) + cos(q5)*cos(q6)*sin(q4)) + l3*cos(q6)*sin(q4) + l3*cos(q4)*cos(q5)*sin(q6), -l4*cos(q6)*sin(q5),  l4*sin(q6), 0]
[                                                                                                                                                                      sin(q4)*sin(q5)*(l3*cos(q2 + q3) + l2*sin(q2)),                                                                                                l2*cos(q3)*cos(q5) + cos(q4)*sin(q5)*(l3 - l2*sin(q3)),                                                                               l3*cos(q4)*sin(q5),                   0,           0, 0]
[                                                                                                                      - cos(q6)*(sin(q2 + q3)*sin(q5) - cos(q2 + q3)*cos(q4)*cos(q5)) - cos(q2 + q3)*sin(q4)*sin(q6),                                                                                                           - cos(q4)*sin(q6) - cos(q5)*cos(q6)*sin(q4),                                                      - cos(q4)*sin(q6) - cos(q5)*cos(q6)*sin(q4),     cos(q6)*sin(q5),    -sin(q6), 0]
[                                                                                                                        sin(q6)*(sin(q2 + q3)*sin(q5) - cos(q2 + q3)*cos(q4)*cos(q5)) - cos(q2 + q3)*cos(q6)*sin(q4),                                                                                                             cos(q5)*sin(q4)*sin(q6) - cos(q4)*cos(q6),                                                        cos(q5)*sin(q4)*sin(q6) - cos(q4)*cos(q6),    -sin(q5)*sin(q6),    -cos(q6), 0]
[                                                                                                                                                               - sin(q2 + q3)*cos(q5) - cos(q2 + q3)*cos(q4)*sin(q5),                                                                                                                                       sin(q4)*sin(q5),                                                                                  sin(q4)*sin(q5),             cos(q5),           0, 1]     
```
Rango del Jacobiano
```sh
rank(J77)

ans =

     6
```
## Determinante y singularidades
```sh
DETJ77=simplify(det(J77))

DETJ77 =
 
-l2*l3*cos(q3)*sin(q5)*(l3*cos(q2 + q3) + l2*sin(q2))
```
## Trayectoria de perfil quintico


