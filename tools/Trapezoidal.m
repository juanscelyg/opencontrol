function [tiempo, v] = Trapezoidal(PT, dt, t0)
global a;             % Aceleración

 V = PT(1)/3.6;     % Velocidad en m/s
 T = PT(2);         % Tiempo del perfil
TD = PT(3);         % Tiempo de descanso

tau = V/a;
tiempo = 0:dt:T+TD;
N = length(tiempo);
v = tiempo-tiempo;

for k=1:N
 t = tiempo(k);  
    %Tramo-1
    if (t < tau)
      v(k)= a*t; 
    
    %Tramo-2
    elseif (t>=tau & t<=T-tau)
      v(k) = V;

    %Tramo-3
    elseif (t>T-tau & t<=T)
      v(k)= a*(T - t);

    %Tramo-4
    else
      v(k)=0;
    end
end
tiempo = tiempo + t0;
end