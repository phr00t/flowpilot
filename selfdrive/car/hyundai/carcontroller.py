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
from common.numpy_fast import interp
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
TICKS_FOR_HARDSTEERING_SMOOTHING = 50
DISTSPEED_TIMEFRAME = 0.9
DISTSPEED_AVERAGING = 7

def signsquare(x):
  return x * abs(x)

def clamp(num, min_value, max_value):
  return max(min(num, max_value), min_value)

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
    self.usingDistSpeed = self.Options.get_bool("UseDistSpeed")
    self.sensitiveSlow = self.Options.get_bool("SensitiveSlow")

    self.lead_accel_accum = 0.0
    self.accel_last = 0
    self.accels = []
    self.accels_max = []
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0
    self.lkas11_cnt = 0
    self.mdpsBus = 0
    self.prevent_heavy_steer_timer = 0

    self.speed_ratios =  [0.0, 1.0,  1.05,  1.1,  1.15,  1.2,  1.25,  1.3]
    self.target_accels = [2.0, 0.0,  -0.3, -0.7,  -1.2, -1.8,  -2.5, -3.0]

    self.lead_distance_hist = []
    self.lead_distance_times = []
    self.lead_distance_distavg = []
    self.lead_seen_counter = 0

    self.temp_disable_spamming = 0

  def update(self, VM, CC, sm, vcruise, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)

    # if we are disabled, or the driver is doing a sharp turn themselves, don't apply any additional steering
    if CS.out.steeringPressed:
      self.prevent_heavy_steer_timer = TICKS_FOR_HARDSTEERING_SMOOTHING
    elif self.prevent_heavy_steer_timer > 0:
      self.prevent_heavy_steer_timer -= 1

    if not CC.latActive:
      apply_steer = 0
    elif abs(CS.out.steeringAngleDeg) > 89 and self.prevent_heavy_steer_timer > 0:
      # gradually bring steering back in on sharp turns
      apply_steer = lerp(apply_steer, 0, self.prevent_heavy_steer_timer / TICKS_FOR_HARDSTEERING_SMOOTHING)

    self.apply_steer_last = apply_steer

    clu11_speed = CS.out.vEgo * 2.23694 # convert to MS -> MPH

    # accel + longitudinal
    accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    #stopping = actuators.longControlState == LongCtrlState.stopping
    #set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    can_sends = []

    # >90 degree steering fault prevention
    # Count up to MAX_ANGLE_FRAMES, at which point we need to cut torque to avoid a steering fault
    steering_amount = abs(CS.out.steeringAngleDeg)
    if CC.latActive and steering_amount >= MAX_ANGLE:
      self.angle_limit_counter += 1
    else:
      self.angle_limit_counter = 0

    # Cut steer actuation bit for two frames and hold torque with induced temporary fault
    torque_fault = CC.latActive and self.angle_limit_counter > MAX_ANGLE_FRAMES
    lat_active = CC.latActive and not torque_fault

    # if driver is steering at a high angle, cut our steering here
    # this avoids ugly fighting and steering actuator noises
    if lat_active and steering_amount >= MAX_ANGLE and CS.out.steeringPressed:
      lat_active = False

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
    long_plan = sm['longitudinalPlan']
    model_out = sm['modelV2']

    stoplinesp = model_out.gpuExecutionTime

    max_speed_in_mph = vcruise * 0.621371
    driver_doing_speed = CS.out.brakeLightsDEPRECATED or CS.out.gasPressed

    # what speed does the model want us going?
    # this looks for stop signs and red lights
    # accels is a weird type so we will iterate over it and take a lowest average
    accel_count = len(long_plan.accels)
    avg_accel_min = max_speed_in_mph
    avg_accel_max = max_speed_in_mph
    if accel_count > 4:
      accel_smallest = 9999
      accel_biggest = 0
      for i in range(0, accel_count):
        if long_plan.accels[i] < accel_smallest:
          accel_smallest = long_plan.accels[i]
        if long_plan.accels[i] > accel_biggest:
          accel_biggest = long_plan.accels[i]
      self.accels.append(accel_smallest * CV.MS_TO_MPH)
      self.accels_max.append(accel_biggest * CV.MS_TO_MPH)
      if len(self.accels) > 20:
        avg_accel_min = statistics.fmean(self.accels)
        avg_accel_max = statistics.fmean(self.accels_max)
        self.accels.pop(0)
        self.accels_max.pop(0)

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
    l0dstd = radarState.leadOne.aLeadK
    l0v = radarState.leadOne.vRel
    l0vstd = radarState.leadOne.vLeadK
    l0time = radarState.leadOne.aLeadTau

    # default use basic speed adjustment
    use_basic_speedadj = True

    # if we are using the distspeed feature, monitor distance to better estimate speed within standard deviation
    if l0prob > 0.5 and self.usingDistSpeed:
      # ok, start collecting data on the lead car
      self.lead_distance_hist.append(l0d)
      self.lead_distance_times.append(l0time)
      # if we've got enough data to calculate a distspeed
      if len(self.lead_distance_hist) >= 2:
        time_diff = self.lead_distance_times[-1] - self.lead_distance_times[0]
        if time_diff >= DISTSPEED_TIMEFRAME:
            dist_diff = self.lead_distance_hist[-1] - self.lead_distance_hist[0]
            # clamp speed to model's speed uncertainty window
            max_allowed = (l0v + l0vstd) * CV.MS_TO_MPH
            min_allowed = (l0v - l0vstd) * CV.MS_TO_MPH
            distspeed = clamp((dist_diff / time_diff) * CV.MS_TO_MPH, min_allowed, max_allowed)
            # wait, if we have a bunch of distance uncertainty, use the model speed more
            distspeed = interp(l0dstd, [3.5, 10.0], [distspeed, l0v * CV.MS_TO_MPH])
            # add this value to be averaged later
            self.lead_distance_distavg.append(distspeed)
            # clean out existing entries
            self.lead_distance_hist.pop(0)
            self.lead_distance_times.pop(0)
            # do we have enough distances over time to get a distspeed estimate?
            if len(self.lead_distance_distavg) >= DISTSPEED_AVERAGING:
              use_basic_speedadj = False
              lead_vdiff_mph = clamp(statistics.fmean(self.lead_distance_distavg), min_allowed, max_allowed)
              self.lead_distance_distavg.pop(0)
    else:
      # no lead, clear data
      self.lead_distance_hist.clear()
      self.lead_distance_times.clear()
      self.lead_distance_distavg.clear()

    # if we didn't get an adjusted lead car speed with distspeed estimate, use a basic speed adjustment system
    if use_basic_speedadj:
      # adjust l0v based on l0vstd and l0v
      l0vstd_multiplier = 1.75 / (1 + math.exp(-l0v-0.289)) - 1.0

      # if we think we should have the lead car going faster, verify we are not too close to the lead car
      # before applying this fully
      if l0vstd_multiplier > 0:
        cutoff_distance = clamp(CS.out.vEgo * 1.75, 35, 60)
        l0vstd_multiplier *= interp(l0d, [10.0, cutoff_distance], [0.0, 1.0])

      # ok, get a good estimate of the lead car speed
      lead_vdiff_mph = (l0v + l0vstd_multiplier * l0vstd) * 2.23694

    # start with our picked max speed
    desired_speed = max_speed_in_mph

    # if we are apporaching a turn, slow down in preparation
    # also note how much of a speed difference we need for this turn
    vcurv_adj = 0.35 + (0.65 / (0.35 * vcurv + 1))
    desired_speed *= vcurv_adj
    curve_speed_ratio = clu11_speed / desired_speed

    # is there a lead worth making decisions on?
    if l0prob > 0.5:
      self.lead_seen_counter += 1
      if clu11_speed > 5:
        # calculate an estimate of the lead car's speed
        lead_speed = clu11_speed + lead_vdiff_mph
        # calculate lead car time
        speed_in_ms = clu11_speed * 0.44704
        lead_time = l0d / speed_in_ms
        # caculate a target lead car time, which is generally 3 seconds unless we are driving fast
        # then we need to be a little closer to keep car within good visible range
        # and prevent big gaps where cars always are cutting in
        target_time = 3 - ((clu11_speed / 70) ** 3)
        # do not go under a certain lead car time for safety
        if target_time < 2.1:
          target_time = 2.1
        # calculate the speed difference we should be going
        adjust_speed = (lead_vdiff_mph * 1.33) + ((5.5 * (lead_time - target_time))/target_time) ** 3
        # don't sudden slow for certain situations, as this causes significant braking
        # 1) if the lead car is far away in either time or distance
        # 2) if the lead car is moving away from us
        leadcar_going_faster = lead_vdiff_mph >= 0.6
        # calculate the final max speed we should be going based on lead car
        max_lead_adj = clu11_speed + adjust_speed
        # if the lead car is going faster than us, but we want to slow down for some reason (to make space etc)
        # don't go much slower than the lead car, and cancel any sudden slowing that may be happening
        fasterleadcar_imposed_speed_limit = max(clu11_speed - 1.5, lead_speed - 2.3)
        if leadcar_going_faster and max_lead_adj < fasterleadcar_imposed_speed_limit:
          # slowly make space between cars
          max_lead_adj = fasterleadcar_imposed_speed_limit
        elif leadcar_going_faster and max_lead_adj < clu11_speed - 1.5:
          # slow down, but not aggresively
          max_lead_adj = clu11_speed - 1.5
        elif not leadcar_going_faster and self.lead_seen_counter < 150 and max_lead_adj > clu11_speed:
          max_lead_adj = clu11_speed # dont speed up if we see a new car and its not going faster than us
        # cap our desired_speed to this final max speed
        if desired_speed > max_lead_adj:
          desired_speed = max_lead_adj
    else:
      self.lead_seen_counter = 0

    allow_reenable_cruise = False
    target_accel = 0.0

    # get option updates
    if self.frame % 100 == 0:
      self.usingAccel = self.Options.get_bool("UseAccel")
      self.usingDistSpeed = self.Options.get_bool("UseDistSpeed")
      self.sensitiveSlow = self.Options.get_bool("SensitiveSlow")

    if self.usingAccel and (avg_accel_min < 8.0 or stoplinesp > 0.7) and clu11_speed <= 40:
      # stop sign or red light, stop!
      desired_speed = 0
      #CS.time_cruise_cancelled = datetime.datetime(2000, 10, 1, 1, 1, 1,0)
    elif desired_speed > 0:
      # does the model think we should be really slowing down?
      if self.usingAccel and avg_accel_min < clu11_speed - 10 and desired_speed > clu11_speed - 1.5:
        desired_speed = clu11_speed - 1.5

      # clamp for the following divisions
      desired_speed = clamp(desired_speed, 0.001, max_speed_in_mph)

      # what is our speed to desired speed ratio?
      target_speed_ratio = clu11_speed / desired_speed
      target_accel_lead = interp(target_speed_ratio, self.speed_ratios, self.target_accels)
      target_accel_curv = interp(curve_speed_ratio, self.speed_ratios, self.target_accels)
      if min(target_accel_curv, target_accel_lead) >= 0 or target_accel > CS.out.aEgo:
        # we don't need to brake this hard for any case, re-enable cruise
        allow_reenable_cruise = True
        self.lead_accel_accum = 0.0
      elif target_accel_curv < CS.out.aEgo - 0.2 and clu11_speed - desired_speed > 1.25:
        # easy check to slow down for a curve
        desired_speed = 0
      else:
        # we might want to slow for a lead car infront of us, but we don't want to make quick small brakes
        # lets see if we should be braking enough before doing so
        lead_accel_diff = signsquare(1.1 * (0.165 + target_accel_lead - CS.out.aEgo))
        if lead_accel_diff < 0:
          self.lead_accel_accum += lead_accel_diff * (20/100) # based off of 20 fps model and this function @ 100hz
        else:
          self.lead_accel_accum = 0.0
        # if it seems like we should be slowing down enough over time, kill cruise to brake harder
        if self.lead_accel_accum < (-1.4 if self.sensitiveSlow else -1.5) and clu11_speed - desired_speed >= 1.85:
          desired_speed = 0
    else:
      # we are stopping for some other reason, clear our lead accumulator
      self.lead_accel_accum = 0.0

    # what is our difference between desired speed and target speed?
    speed_diff = desired_speed - clu11_speed

    # apply a spam overpress to amplify speed changes
    # then clamp
    desired_speed = clamp(desired_speed + speed_diff * 0.6, 0.0, max_speed_in_mph)

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
    sLogger.Send("0ac" + "{:.2f}".format(CS.out.aEgo) + " c" + "{:.1f}".format(CS.out.cruiseState.speed) + " v" + "{:.2f}".format(l0v * 2.23694) + " ta" + "{:.2f}".format(target_accel) + " lt" + str(int(use_basic_speedadj)) + " ds" + "{:.1f}".format(desired_speed) + " dv" + "{:.2f}".format(lead_vdiff_mph) + " vs" + "{:.2f}".format(l0vstd * 2.23694) + " ld" + "{:.1f}".format(l0d) + " ms" + "{:.1f}".format(avg_accel_min) + " du" + "{:.1f}".format(l0dstd))

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
    if CS.out.cruiseState.nonAdaptive and self.temp_disable_spamming <= 0 and allow_reenable_cruise:
      can_sends.append(hyundaican.create_cpress(self.packer, CS.clu11, Buttons.SET_DECEL)) # re-enable cruise at our current speed
      self.temp_disable_spamming = 5 # take a break

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends
