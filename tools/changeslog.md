#Controls.py

+ Lineas 33 y 34

~~~
from datetime import datetime
import time
~~~

+ Linea 860 a la 876 dentro del __controlsd_thread__

~~~
    #KS = car.CarState.new_message()
    #KC = car.CarState.new_message() # CarControl Struct
    t_ini = time.time_ns()
    contador = 0
    separator = "\t"
    my_date = datetime.fromtimestamp(time.time())
    doc = open("data_"+my_date.strftime("%Y%m%d_%H%M")+".txt",'a')
    doc.write("Time: "+separator+"vEgo: "+separator+"hud_speed: "+separator+"v_pid: "+separator+"State: "+separator+"vEgo_raw: "+"\n")
    while True:
      KS, KC = self.step()
      self.rk.monitor_time()
      self.prof.display()
      if (contador%25) == 0:
        dt=(time.time_ns()-t_ini)/1000000
        #str_lB = ("1" if KS.leftBlinker else "0")
        doc.write(str(dt)[:9]+separator+str(KS.vEgo)[:8]+separator+str(KC.hudControl.setSpeed)[:8]+separator+str(self.LoC.v_pid)[:8]+separator+str(self.LoC.long_control_state)[:8]+separator+str(KC.vEgoRaw)[:8]+"\n")
      contador = contador + 1 
~~~

#lib/drive_helpers.py

+ Linea 11 a la 14

~~~
V_CRUISE_MAX = 80.0  # kph
V_CRUISE_MIN = 0.0  # kph
V_CRUISE_ENABLE_MIN = 0.0  # kph
V_CRUISE_INITIAL = 5.0  # kph
~~~

+ Comentar Linea 114

~~~
 #self.v_cruise_kph = clip(round(self.v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

~~~

#lib/longcontrol.py

+ En linea 10 insertar bloque de código

~~~
import time 

LINEAL = 1
TRAPEZOIDAL = 5
~~~

+ Después de la definición de la máquina de estados ̣̣__long_control_state_trans__ en la linea 62 insertar el siguiente bloque de código

~~~

class planner:
    # vel_vector: vector de velocidades deseadas
    # time_vector: vector de tiempos de cambio deseados
    # tipo: tipo de suavizado y transición entre velocidades
    def __init__(self,tipo):
        # --- Variables
        self.enable = False
        self.flag = True
        self.tipo = tipo
        self.vel_vector = []
        self.time_vector = []
        self.separator=','
        if self.tipo == 5:
            self.archivo = '../controls/tests/perfil_trapezoidal.csv'
            with open(self.archivo, 'r') as _archivo:
                for linea in _archivo:
                    linea = linea.rstrip()
                    lista=linea.split(self.separator)
                    self.time_vector.append(float(lista[0]))
                    self.vel_vector.append(float(lista[1]))
        # Velocidades y aceleraciones        
        self.t_a = 10000.0
        self.acc_max = 0.001
        self.stage = 0
        self.cont=1
        self.change_stage = False
        self.t_ini = 0
        
    #time: tiempo actual de simulación en segundos
    def update(self, t, CS):  
        if CS.rightBlinker and self.flag:
           self.enable = True # No usar el toogle
           self.flag = False
           self.t_ini = time.time_ns()/1000000000.0
        else:
           self.enable = False
           self.flag = True
        if self.tipo==5 and self.enable:
            vel=self.read_trapezoidal(t-self.t_ini)
        else:
            vel=0.0
        return vel

    #time: tiempo actual de simulación en segundos
    def read_trapezoidal(self, t):
        longitud=len(self.time_vector)
        if t>self.time_vector[self.cont] and t<self.time_vector[longitud-1]:
            self.cont+=1
        return self.vel_vector[self.cont]
~~~

+ Linea 21 igual condición a False

~~~
stopping_condition = False #planned_stop or stay_stopped
~~~

+ Modificar linea 63 por bloque comentarios

~~~
    self.v_pid = 0.0 
    self.vpid_flag = True
    self.t_ini = time.time_ns()
    self.vel_profile = planner(TRAPEZOIDAL) 
~~~

+ Insertar en la linea 69

~~~
    self.vpid_flag = True
~~~

+ Entre lineas 97 y 98

~~~
    if self.vpid_flag:
      self.v_pid = 0.0 #0.0
      self.vpid_flag=False
    dt_ini=(time.time_ns()-self.t_ini)/1000000000.0 # Diferencial de tiempo en segundos
~~~

+ Reemplazar la linea 117 (self.v_pid = v_target_now) por:

~~~
self.v_pid = self.vel_profile.update(dt_ini, CS)
~~~