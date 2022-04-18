# Robótica
## _Robot Comau-Racer 5-0.80_

[![N|Solid](https://media.giphy.com/media/zpuLFclMqtnc04zWSj/giphy.gif)](https://www.comau.com/en/competencies/robotics-automation/robot-team/racer-5-0-80/)

## Tabla de Especificaciones

| Métrica | Valor |
| ------ | ------ |
| Número de ejes | 6 |
| Carga máxima en la muñeca | 5 Kg |
| Alcance horizontal máximo (Radio) | 809 mm |
| Torque en el eje 4 (Nm) | 8.83 Nm |
| Torque en el eje 5 (Nm) | 8.83 Nm |
| Torque en el eje 6 (Nm) | 4.91 Nm |
| Carrera (velocidad) en el eje 1 | +/- 170° (360°/s) |
| Carrera (velocidad) en el eje 2 | -95°/ +135° (300°/s) |
| Carrera (velocidad) en el eje 3 | -155° / +90° (330°/s) |
| Carrera (velocidad) en el eje 4 | +/- 210° (500°/s) |
| Carrera (velocidad) en el eje 5 | +/- 125° (500°/s) |
| Carrera (velocidad) en el eje 6 | +/- 2700° (800°/s) |
| Repetibilidad (mm) | 0.03 mm |
| Peso del robot (Kg) | 32 kg |
| Clase de protección | IP54 |
| Posición de montaje | Piso/techo/pared (permitido con limitaciones de carga útil) |
| Áreas de operación A (mm) | 1124 mm |
| Áreas de operación B (mm) | 809 mm |
| Áreas de operación C (mm) | 8 mm |
| Áreas de operación D (mm) | 708 mm |
| Áreas de operación E (mm) | 286 mm |

-  [Plano](https://www.comau.com/wp-content/uploads/2021/06/Racer5-0.80-2D-PDF.pdf)

## Parámetros Denavit-Hartenberg 

<img src="/Imagenes/Denavit-Hartenberg.PNG" width="761" height="585">

## Expresión de Cinemática Directa [(Simulación 1)](https://github.com/pedroomtz17/Comau-Racer-5-0.80-Robot/blob/master/simulacion1.m).

<img src="https://media.giphy.com/media/QSSbQwaM9npmFEg6op/giphy.gif" width="560" height="420">



## Expresión de Cinemática Inversa [(Simulación 2)](https://github.com/pedroomtz17/Comau-Racer-5-0.80-Robot/blob/master/simulacion2.m).


- Solución inicial (un solo punto para el efector)
<img src="/Imagenes/simulacion2.png" width="440" height="300">
<img src="/Imagenes/simulacion2.1.png" width="440" height="300">

- Trayectoria [(Simulación 3)](https://github.com/pedroomtz17/Comau-Racer-5-0.80-Robot/blob/master/simulacion3.m).
<img src="https://media.giphy.com/media/KO5sEzf0Wgiia0SQMe/giphy.gif" width="560" height="420">
<img src="/Imagenes/simulacion3.png" width="440" height="300">

- Determinante
<img src="/Imagenes/DeterminanteTrayectoriaSimu3.png" width="440" height="300">

## Graficas de posición de las juntas

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
Las singularidades se presentan en los siguientes casos:

<img src="/Imagenes/Singularidad1.png">

Ejemplo

<img src="https://media.giphy.com/media/9bHYg9AnFBFHK77det/giphy.gif" width="560" height="420">
<img src="/Imagenes/Singularidad.png" width="440" height="300">

## Trayectoria de perfil quintico
<img src="https://media.giphy.com/media/bntAxkibYtJFUQXubP/giphy.gif" width="560" height="420">
<img src="/Imagenes/TrayectoriaPQ.png" width="440" height="300">

- Posición Espacio Cartesiano

<img src="/Imagenes/PosicionEspacioCartesiano.png" width="440" height="300">

- Velocidad Espacio Cartesiano

<img src="/Imagenes/VelocidadEspacioCartersiano.png" width="440" height="300">

- Aceleración Espacio Cartesiano

<img src="/Imagenes/AceleracionEspacioCartersiano.png" width="440" height="300">

- Espacio de Juntas

<img src="/Imagenes/EspacioJuntas.png" width="440" height="300">

- Velocidad Articular

<img src="/Imagenes/VelocidadArticular.png" width="440" height="300">

- Aceleración Articular

<img src="/Imagenes/AceleracionArticular.png" width="440" height="300">

- Determinante

<img src="/Imagenes/DeterminantePQ.png" width="440" height="300">
