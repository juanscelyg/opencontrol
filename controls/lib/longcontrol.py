from cereal import car
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL, sec_since_boot
from selfdrive.controls.lib.drive_helpers import CONTROL_N, apply_deadzone
from selfdrive.controls.lib.pid import PIDController
from selfdrive.modeld.constants import T_IDXS

LongCtrlState = car.CarControl.Actuators.LongControlState

### -----------------------------
#
# Modificacion JC
#
### -----------------------------
import time 

LINEAL = 1
TRAPEZOIDAL = 5

def long_control_state_trans(CP, active, long_control_state, v_ego, v_target,
                             v_target_1sec, brake_pressed, cruise_standstill):
  # Ignore cruise standstill if car has a gas interceptor
  cruise_standstill = cruise_standstill and not CP.enableGasInterceptor
  accelerating = v_target_1sec > v_target
  planned_stop = (v_target < CP.vEgoStopping and
                  v_target_1sec < CP.vEgoStopping and
                  not accelerating)
  stay_stopped = (v_ego < CP.vEgoStopping and
                  (brake_pressed or cruise_standstill))
  stopping_condition = False #planned_stop or stay_stopped

  starting_condition = (v_target_1sec > CP.vEgoStarting and
                        accelerating and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state


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

class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off  # initialized to off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)
    self.v_pid = 0.0 
    self.last_output_accel = 0.0
    self.vpid_flag = True
    self.t_ini = time.time_ns()
    self.vel_profile = planner(TRAPEZOIDAL) 

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid
    self.vpid_flag = True

  def update(self, active, CS, long_plan, accel_limits, t_since_plan):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory
    speeds = long_plan.speeds
    if len(speeds) == CONTROL_N:
      v_target_now = interp(t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_now = interp(t_since_plan, T_IDXS[:CONTROL_N], long_plan.accels)

      v_target_lower = interp(self.CP.longitudinalActuatorDelayLowerBound + t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_lower = 2 * (v_target_lower - v_target_now) / self.CP.longitudinalActuatorDelayLowerBound - a_target_now

      v_target_upper = interp(self.CP.longitudinalActuatorDelayUpperBound + t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_upper = 2 * (v_target_upper - v_target_now) / self.CP.longitudinalActuatorDelayUpperBound - a_target_now

      v_target = min(v_target_lower, v_target_upper)
      a_target = min(a_target_lower, a_target_upper)

      v_target_1sec = interp(self.CP.longitudinalActuatorDelayUpperBound + t_since_plan + 1.0, T_IDXS[:CONTROL_N], speeds)
    else:
      v_target = 0.0
      v_target_now = 0.0
      v_target_1sec = 0.0
      a_target = 0.0

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    output_accel = self.last_output_accel

    if self.vpid_flag:
      self.v_pid = 0.0 #0.0
      self.vpid_flag=False
    dt_ini=(time.time_ns()-self.t_ini)/1000000000.0 # Diferencial de tiempo en segundos

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       v_target, v_target_1sec, CS.brakePressed,
                                                       CS.cruiseState.standstill)

    if self.long_control_state == LongCtrlState.off:
      self.reset(CS.vEgo)
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset(CS.vEgo)

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset(CS.vEgo)

    elif self.long_control_state == LongCtrlState.pid:

      self.v_pid = self.vel_profile.update(dt_ini, CS)
      
      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      # TODO too complex, needs to be simplified and tested on toyotas
      #prevent_overshoot = not self.CP.stoppingControl and CS.vEgo < 1.5 and v_target_1sec < 0.7 and v_target_1sec < self.v_pid
      prevent_overshoot=False
      deadzone = interp(CS.vEgo, self.CP.longitudinalTuning.deadzoneBP, self.CP.longitudinalTuning.deadzoneV)
      freeze_integrator = prevent_overshoot

      error = self.v_pid - CS.vEgo
      error_deadzone = apply_deadzone(error, deadzone)
      output_accel = self.pid.update(error_deadzone, speed=CS.vEgo,
                                     feedforward=a_target,
                                     freeze_integrator=freeze_integrator)

    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

    return self.last_output_accel
