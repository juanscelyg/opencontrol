%************* Datos de un PT**************
% El PT asume una acelaracion predefinida (a  [m/s^2]) dada en valor absoluto
% V: Velocidad deseada, en [km/h]
%    Su manejo en las ecuaciones es [m/s]
% T: Tiempo de desarrollo del perfil  [s]
% TD: Tiempo de descanso despues de finalizar el PT [s]
% t: variable para el manejo del tiempo en segundos [s]

clear
close all
global a;             % Aceleración   m/s^2

a = 1;
PT1 = [0 5 5];      % Especificacion de un perfil
PT2 = [20 30 20];    
PT3 = [15 30 5];    

dt = 0.25;            %tiempo muestreo
t0=0;
[t1 v1] = Trapezoidal(PT1, dt, t0);

t0 = t0 + PT1(2) + PT1(3);
[t2 v2] = Trapezoidal(PT2, dt, t0);

t0 = t0 + PT2(2) + PT2(3);
[t3 v3] = Trapezoidal(PT3, dt, t0);
t = [t1 t2 t3]'; v = [v1 v2 v3]';

Data = [t v];
plot(t, 3.6*v); 
ylabel('Vel. (Km/h)')
xlabel('Time (s)')
grid

d=string(datetime('now','InputFormat','yyyy-MM-dd HH:mm'));
writematrix(Data, "perfil_trapezoidal.csv");
