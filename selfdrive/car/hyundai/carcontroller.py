from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import clip
from common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from common.params import Params, put_nonblocking
from selfdrive.car import apply_driver_steer_torque_limits
from selfdrive.car.hyundai import hyundaicanfd, hyundaican
from selfdrive.car.hyundai.hyundaicanfd import CanBus
from selfdrive.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CANFD_CAR, CAR
from common.logger import sLogger
import numpy as np

import statistics
import datetime
import math

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2

def lerp(a, b, t):
  return (b * t) + (a * (1.0 - t))

def reject_outliers(data, m=2.):
  data = np.array(data)
  d = np.abs(data - np.median(data))
  mdev = np.median(d)
  s = d / mdev if mdev else np.zeros(len(d))
  return data[s < m].tolist()

def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)
    self.angle_limit_counter = 0
    self.frame = 0
    self.Options = Params()
    self.usingAccel = self.Options.get_bool("UseAccel")

    self.accel_last = 0
    self.accels = []
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0
    self.lkas11_cnt = 0
    self.mdpsBus = 0

    self.lead_distance_hist = []
    self.lead_distance_times = []
    self.lead_distance_histavg = []
    self.lead_distance_accuracy = []

    self.temp_disable_spamming = 0

  def update(self, CC, sm, vcruise, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)

    if not CC.latActive:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    clu11_speed = CS.out.vEgo * 2.23694 # convert to MS -> MPH

    # accel + longitudinal
    accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    #stopping = actuators.longControlState == LongCtrlState.stopping
    #set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)
    if 2.0 > actuators.accel > -3.5:
      self.accels.append(actuators.accel)
    if len(self.accels) > 6:
      self.accels.pop(0)
    avg_accel = statistics.fmean(reject_outliers(self.accels))

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    can_sends = []

    # >90 degree steering fault prevention
    # Count up to MAX_ANGLE_FRAMES, at which point we need to cut torque to avoid a steering fault
    if CC.latActive and abs(CS.out.steeringAngleDeg) >= MAX_ANGLE:
      self.angle_limit_counter += 1
    else:
      self.angle_limit_counter = 0

    # Cut steer actuation bit for two frames and hold torque with induced temporary fault
    torque_fault = CC.latActive and self.angle_limit_counter > MAX_ANGLE_FRAMES
    lat_active = CC.latActive and not torque_fault

    if self.frame == 0: # initialize counts from last received count signals
      self.lkas11_cnt = CS.lkas11["CF_Lkas_MsgCount"] + 1
    self.lkas11_cnt %= 0x10

    if self.angle_limit_counter >= MAX_ANGLE_FRAMES + MAX_ANGLE_CONSECUTIVE_FRAMES:
      self.angle_limit_counter = 0

    can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.car_fingerprint, apply_steer, lat_active,
                                              torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                              hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                              left_lane_warning, right_lane_warning))

    # 20 Hz LFA MFA message
    if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
      can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled))

    # phr00t fork start for cruise spamming
    path_plan = sm['lateralPlan']

    max_speed_in_mph = vcruise * 0.621371
    driver_doing_speed = CS.out.brakeLightsDEPRECATED or CS.out.gasPressed

    # get biggest upcoming curve value, ignoring the curve we are currently on (so we plan ahead better)
    vcurv = 0
    curv_len = len(path_plan.curvatures)
    if curv_len > 0:
      curv_middle = math.floor((curv_len - 1)/2)
      for x in range(curv_middle, curv_len):
        acurval = abs(path_plan.curvatures[x] * 100)
        if acurval > vcurv:
          vcurv = acurval

    # lead car info
    radarState = sm['radarState']
    l0prob = radarState.leadOne.modelProb
    l0d = radarState.leadOne.dRel
    l0v = radarState.leadOne.vRel
    lead_vdiff_mph = l0v * 2.23694
    raw_vlead = radarState.leadOne.vLead * 2.23694

    # store distance history of lead car to merge with l0v to get a better speed relative value
    time_interval_for_distspeed = 0.666
    overall_confidence = 0
    l0v_distval_mph = 0
    if l0prob > 0.5:
      # ok, start averaging this distance value
      self.lead_distance_histavg.append(l0d)
      # if we've got enough data to average, do so into our main list
      if len(self.lead_distance_histavg) >= 16:
        # get some statistics on the data we've collected
        finalavg = statistics.fmean(self.lead_distance_histavg)
        # calculate accuracy based on variance within X meters
        finalacc = 1.0 - (statistics.pvariance(self.lead_distance_histavg) / 3.5)
        if finalacc < 0.0:
          finalacc = 0.0
        self.lead_distance_hist.append(finalavg)
        self.lead_distance_accuracy.append(finalacc)
        self.lead_distance_histavg.clear()
        # timestamp
        self.lead_distance_times.append(datetime.datetime.now())
        # should we remove an old entry now that we just added a new one?
        if len(self.lead_distance_times) > 2 and (self.lead_distance_times[-1] - self.lead_distance_times[1]).total_seconds() > time_interval_for_distspeed:
          self.lead_distance_hist.pop(0)
          self.lead_distance_times.pop(0)
          self.lead_distance_accuracy.pop(0)
      # do we have enough averaged data to calculate a speed?
      if len(self.lead_distance_times) > 1:
        time_diff = (self.lead_distance_times[-1] - self.lead_distance_times[0]).total_seconds()
        # if we've got enough data, calculate a speed based on our distance data
        # also get confidence based on the individual distance values compared
        if time_diff > time_interval_for_distspeed:
          l0v_distval_mph = ((self.lead_distance_hist[-1] - self.lead_distance_hist[0]) / time_diff) * 2.23694
          overall_confidence = self.lead_distance_accuracy[-1] * self.lead_distance_accuracy[0]
          # values shouldn't be less than my current speed (cars are not driving into me, hopefully)
          if l0v_distval_mph < -clu11_speed:
            l0v_distval_mph = -clu11_speed
          # clamp values far over model speed
          if l0v_distval_mph > lead_vdiff_mph + 20:
            l0v_distval_mph = lead_vdiff_mph + 20
          # reduce confidence of large values different from model's values
          difference_factor = 1.0 - ((abs(l0v_distval_mph - lead_vdiff_mph) / 15.0) ** 1.5)
          # sanity checks / clamping
          if difference_factor < 0:
            difference_factor = 0
          elif difference_factor > 1.0:
            difference_factor = 1.0
          # ok, apply factor for final confidence
          overall_confidence *= difference_factor
    else:
      # no lead, clear data
      self.lead_distance_hist.clear()
      self.lead_distance_times.clear()
      self.lead_distance_histavg.clear()

    # if we got a distspeed value, mix it with l0v based on overall confidence
    # otherwise, just use the model l0v
    if overall_confidence > 0:
      lead_vdiff_mph = lerp(lead_vdiff_mph, l0v_distval_mph, overall_confidence * 0.5)

    # start with our picked max speed
    desired_speed = max_speed_in_mph

    # if we are apporaching a turn, slow down in preparation
    # also note how much of a speed difference we need for this turn
    vcurv_adj = 0.35 + (0.65 / (0.425 * vcurv + 1))
    desired_speed *= vcurv_adj
    curve_speed_ratio = clu11_speed / desired_speed

    # is there a lead?
    if l0prob > 0.5 and clu11_speed > 5:
      # amplify large lead car speed differences a bit so we react faster
      lead_vdiff_mph *= ((abs(lead_vdiff_mph) * 0.033) ** 1.2) + 1
      # calculate an estimate of the lead car's speed for purposes of setting our speed
      lead_speed = clu11_speed + lead_vdiff_mph
      # calculate lead car time
      speed_in_ms = clu11_speed * 0.44704
      lead_time = l0d / speed_in_ms
      # caculate a target lead car time, which is generally 3 seconds unless we are driving fast
      # then we need to be a little closer to keep car within good visible range
      # and prevent big gaps where cars always are cutting in
      target_time = 3-((clu11_speed/72)**3)
      # do not go under a certain lead car time for safety
      if target_time < 2.1:
        target_time = 2.1
      # calculate the difference of our current lead time and desired lead time
      lead_time_ideal_offset = lead_time - target_time
      # don't sudden slow for certain situations, as this causes significant braking
      # 1) if the lead car is far away in either time or distance
      # 2) if the lead car is moving away from us
      leadcar_going_faster = lead_vdiff_mph >= 1
      dont_sudden_slow = lead_time_ideal_offset > target_time * 0.39 or l0d >= 80 or leadcar_going_faster
      # depending on slowing down or speeding up, scale
      if lead_time_ideal_offset < 0:
        lead_time_ideal_offset = -(-lead_time_ideal_offset * (10.5/target_time)) ** 1.4 # exponentially slow down if getting closer and closer
      else:
        lead_time_ideal_offset = (lead_time_ideal_offset * 3) ** 1.25 # exponentially not consider lead car the further away
      # calculate the final max speed we should be going based on lead car
      max_lead_adj = lead_speed + lead_time_ideal_offset
      # if the lead car is going faster than us, but we want to slow down for some reason (to make space etc)
      # don't go much slower than the lead car, and cancel any sudden slowing that may be happening
      fasterleadcar_imposed_speed_limit = max(clu11_speed - 2, lead_speed - 3)
      if leadcar_going_faster and max_lead_adj < fasterleadcar_imposed_speed_limit:
        max_lead_adj = fasterleadcar_imposed_speed_limit
      elif dont_sudden_slow and max_lead_adj < clu11_speed * 0.8:
        max_lead_adj = clu11_speed * 0.8
      # cap our desired_speed to this final max speed
      if desired_speed > max_lead_adj:
        desired_speed = max_lead_adj

    reenable_cruise_atspd = desired_speed * 1.02 + 2.0

    # are we using the accel option?
    if self.frame % 100 == 0:
      self.usingAccel = self.Options.get_bool("UseAccel")

    if self.usingAccel:
      # does our model think we should be slowing down? if so, definitely don't speed up
      if avg_accel < -0.5 and desired_speed > clu11_speed:
        desired_speed = clu11_speed

      # do we really want to stop?
      if avg_accel < -1.5:
        desired_speed = 0
    else:
      # note that we are not using this value with a big one
      avg_accel += 100

    # what is our difference between desired speed and target speed?
    speed_diff = desired_speed - clu11_speed

    # apply a spam overpress to amplify speed changes
    desired_speed += speed_diff * 0.6

    slow_speed_factor = 1.5
    # this can trigger sooner than lead car slowing, because curve data is much less noisy
    if curve_speed_ratio > 1.175:
      desired_speed = 0

    # if we are going much faster than we want, disable cruise to trigger more intense regen braking
    if clu11_speed > desired_speed * slow_speed_factor:
      desired_speed = 0

    # sanity checks
    if desired_speed > max_speed_in_mph:
      desired_speed = max_speed_in_mph
    if desired_speed < 0:
      desired_speed = 0

    # if we recently pressed a cruise button, don't spam more to prevent errors for a little bit
    if CS.cruise_buttons != 0:
      self.temp_disable_spamming = 6
    elif driver_doing_speed and abs(clu11_speed - CS.out.cruiseState.speed) > 4 and CS.out.cruiseState.speed >= 20 and clu11_speed >= 20 and self.temp_disable_spamming <= 0:
      # if our cruise is on, but our speed is very different than our cruise speed, hit SET to set it
      can_sends.append(hyundaican.create_cpress(self.packer, CS.clu11, Buttons.SET_DECEL)) #slow cruise
      self.temp_disable_spamming = 6

    # count down self spamming timer
    if self.temp_disable_spamming > 0:
      self.temp_disable_spamming -= 1

    # print debug data
    sLogger.Send("A" + "{:.2f}".format(avg_accel) + " Pr?" + str(CS.out.cruiseState.nonAdaptive) + " Rs?" + "{:.1f}".format(reenable_cruise_atspd) + " DS" + "{:.1f}".format(desired_speed) + " ds" + "{:.1f}".format(l0v_distval_mph) + " c" + "{:.2f}".format(overall_confidence) + " VL" + "{:.1f}".format(raw_vlead) + " VD" + "{:.1f}".format(l0d))

    cruise_difference = abs(CS.out.cruiseState.speed - desired_speed)
    cruise_difference_max = round(cruise_difference) # how many presses to do in bulk?
    if cruise_difference_max > 4:
      cruise_difference_max = 4 # do a max of presses at a time

    # ok, apply cruise control button spamming to match desired speed, if we have cruise on and we are not taking a break
    # also dont press buttons if the driver is hitting the gas or brake
    if cruise_difference >= 0.666 and CS.out.cruiseState.speed >= 20 and self.temp_disable_spamming <= 0 and not driver_doing_speed:
      if desired_speed < 20:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL)) #disable cruise to come to a stop
        self.temp_disable_spamming = 6 # we disabled cruise, don't spam more cancels
        CS.time_cruise_cancelled = datetime.datetime.now() # timestamp when we disabled it, used for autoresuming
      elif CS.out.cruiseState.speed > desired_speed:
        for x in range(cruise_difference_max):
          can_sends.append(hyundaican.create_cpress(self.packer, CS.clu11, Buttons.SET_DECEL)) #slow cruise
        self.temp_disable_spamming = 3 # take a break
      elif CS.out.cruiseState.speed < desired_speed:
        for x in range(cruise_difference_max):
          can_sends.append(hyundaican.create_cpress(self.packer, CS.clu11, Buttons.RES_ACCEL)) #speed cruise
        self.temp_disable_spamming = 3 # take a break

    # are we using the auto resume feature?
    if CS.out.cruiseState.nonAdaptive and self.temp_disable_spamming <= 0 and clu11_speed <= reenable_cruise_atspd:
      can_sends.append(hyundaican.create_cpress(self.packer, CS.clu11, Buttons.SET_DECEL)) # re-enable cruise at our current speed
      self.temp_disable_spamming = 5 # take a break

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends
