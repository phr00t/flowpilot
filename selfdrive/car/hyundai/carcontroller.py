from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import clip
from common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_driver_steer_torque_limits
from selfdrive.car.hyundai import hyundaicanfd, hyundaican
from selfdrive.car.hyundai.hyundaicanfd import CanBus
from selfdrive.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CANFD_CAR, CAR
from common.logger import sLogger

import datetime
import math

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2


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

    self.accel_last = 0
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0
    self.lkas11_cnt = 0
    self.mdpsBus = 0

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
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    can_sends = []

    # *** common hyundai stuff ***

    # REMOVED, looks like SCC stuff
    # tester present - w/ no response (keeps relevant ECU disabled)
    #if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and self.CP.openpilotLongitudinalControl:
      # for longitudinal control, either radar or ADAS driving ECU
      #addr, bus = 0x7d0, 0
      #if self.CP.flags & HyundaiFlags.CANFD_HDA2.value:
        #addr, bus = 0x730, self.CAN.ECAN
      #can_sends.append([addr, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", bus])

      # for blinkers
      #if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        #can_sends.append([0x7b1, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", self.CAN.ECAN])

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

    # perhaps instead of 'stoplinep', we should take info from the long planner which considers all sorts of things
    # may need to fudge the openpilotLongitudinalControl so it parses all the accel stuff out for use here instead of
    # CAN messages for SCC ("stopping" might be useful above, or an negative accel value?)
    #stoplinesp = sm['longitudinalPlan'].stoplineProb

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

    #sLogger.Send("vC>" + "{:.2f}".format(vcurv) + " Pr?>" + str(CS.out.cruiseState.nonAdaptive) + " Rs?>" + "{:.1f}".format(reenable_cruise_atspd) + " DS>" + "{:.1f}".format(desired_speed) + " CCr>" + "{:.1f}".format(CS.current_cruise_speed) + " StP>" + "{:.2f}".format(stoplinesp) + " DSpd>" + "{:.1f}".format(l0v_distval_mph) + " DSpM>" + "{:.1f}".format(lead_vdiff_mph) + " Conf>" + "{:.2f}".format(overall_confidence))
    sLogger.Send(
      "vC>" + "{:.2f}".format(vcurv) + " CCr>" + "{:.1f}".format(CS.speed) + " l0v>" + "{:.1f}".format(lead_vdiff_mph))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends
