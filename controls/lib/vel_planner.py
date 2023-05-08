import numpy as np

LINEAL = 1
TRAPEZOIDAL = 5
separator=','

class planner:
    # vel_vector: vector de velocidades deseadas
    # time_vector: vector de tiempos de cambio deseados
    # tipo: tipo de suavizado y transición entre velocidades
    def __init__(self,tipo):
        # --- Variables
        self.tipo = tipo
        self.vel_vector = []
        self.time_vector = []
        if self.tipo == 5:
            self.archivo = '../controls/profiles/perfil_trapezoidal.csv'
            with open(self.archivo, 'r') as _archivo:
                for linea in _archivo:
                    linea = linea.rstrip()
                    lista=linea.split(separator)
                    self.time_vector.append(float(lista[0]))
                    self.vel_vector.append(float(lista[1]))
        # Velocidades y aceleraciones        
        self.t_a = 10000.0
        self.acc_max = 0.001
        self.stage = 0
        self.cont=1
        self.change_stage = False
        
    #time: tiempo actual de simulación en segundos
    def update(self, time):  
        if self.tipo==1:  
            vel=self.lineal_smooth(time)   
        elif self.tipo==5:
            vel=self.read_trapezoidal(time)
        else:
            vel=0.0
        return vel

    #time: tiempo actual de simulación en segundos
    def read_trapezoidal(self, t):
        if t>self.time_vector[self.cont]:
            self.cont+=1
        return self.vel_vector[self.cont]

    #time: tiempo actual de la ejecución en segundos
    def lineal_smooth(self, t):
        vel=self.vel_vector[self.stage]
        if self.stage < (len(self.vel_vector)-1):
            d_vel=self.vel_vector[self.stage+1]-self.vel_vector[self.stage]
            self.t_a = (abs(d_vel)/self.acc_max)
            if t<self.time_vector[self.stage]:
                vel=self.vel_vector[self.stage]
            elif (self.time_vector[self.stage]) < t and t < (self.time_vector[self.stage]+self.t_a):
                if d_vel < 0.0:
                    vel=self.vel_vector[self.stage] - self.acc_max*(t-self.time_vector[self.stage])
                else:
                    vel=self.vel_vector[self.stage] + self.acc_max*(t-self.time_vector[self.stage])
            elif (self.time_vector[self.stage] + self.t_a) < t:
                self.stage+=1
                vel=self.vel_vector[self.stage]
        else:
            vel=self.vel_vector[-1]
        return vel



