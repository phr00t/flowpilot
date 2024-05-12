import math
import numpy as np
from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp
from common.realtime import DT_MDL
from system.swaglog import cloudlog
from common.logger import sLogger
from common.params import Params

TRAJECTORY_SIZE = 33
# positive numbers go right
CAMERA_OFFSET = 0.08
MIN_LANE_DISTANCE = 2.6
MAX_LANE_DISTANCE = 3.7
TYPICAL_MIN_LANE_DISTANCE = 2.7
TYPICAL_MAX_LANE_DISTANCE = 3.4
CENTER_FORCE_GENERAL_SCALE = 0.4

def clamp(num, min_value, max_value):
  # weird broken case, do something reasonable
  if min_value > num > max_value:
    return (min_value + max_value) * 0.5
  # ok, basic min/max below
  if num < min_value:
    return min_value
  if num > max_value:
    return max_value
  return num

def sigmoid(x, scale=1, offset=0):
  return (1 / (1 + math.exp(x*scale))) + offset

def lerp(start, end, t):
  t = clamp(t, 0.0, 1.0)
  return (start * (1.0 - t)) + (end * t)

def max_abs(a, b):
  return a if abs(a) > abs(b) else b

class LanePlanner:
  def __init__(self):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.le_y = np.zeros((TRAJECTORY_SIZE,))
    self.re_y = np.zeros((TRAJECTORY_SIZE,))
    self.ultimate_path = np.zeros((TRAJECTORY_SIZE,))
    self.lane_width_estimate = FirstOrderFilter(3.2, 9.95, DT_MDL)
    self.lane_width_certainty = FirstOrderFilter(1.0, 0.95, DT_MDL)
    self.lane_width = 3.2
    self.lane_change_multiplier = 1
    self.Options = Params()
    self.UseModelPath = self.Options.get_bool("UseModelPath")
    self.BigModel = self.Options.get_bool("F3")
    self.updateOptions = 100

    self.lll_prob = 0.
    self.rll_prob = 0.
    self.d_prob = 0.
    self.center_force = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

  def parse_model(self, md):

    self.updateOptions -= 1
    if self.updateOptions <= 0:
      self.updateOptions = 100
      self.UseModelPath = self.Options.get_bool("UseModelPath")

    lane_lines = md.laneLines
    edges = md.roadEdges

    if len(lane_lines) == 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(lane_lines[1].t) + np.array(lane_lines[2].t))/2
      # left and right ll x is the same
      self.ll_x = lane_lines[1].x
      self.lll_y = np.array(lane_lines[1].y)
      self.rll_y = np.array(lane_lines[2].y)
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    if len(edges[0].t) == TRAJECTORY_SIZE:
      self.le_y = np.array(edges[0].y) + md.roadEdgeStds[0] * 0.4
      self.re_y = np.array(edges[1].y) - md.roadEdgeStds[1] * 0.4
    else:
      self.le_y = self.lll_y
      self.re_y = self.rll_y

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]

  def get_stock_path(self, CS, v_ego, path_t, path_xyz, vcurv):
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in (0.0, 1.5, 3.0):
      width_at_t = interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      prob_mods.append(interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    # Find current lanewidth
    self.lane_width_certainty.update(l_prob * r_prob)
    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
    self.lane_width_estimate.update(current_lane_width)
    speed_lane_width = interp(v_ego, [0., 31.], [2.8, 3.5])
    self.lane_width = self.lane_width_certainty.x * self.lane_width_estimate.x + \
                      (1 - self.lane_width_certainty.x) * speed_lane_width

    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    self.d_prob = l_prob + r_prob - l_prob * r_prob
    self.d_prob *= self.lane_change_multiplier
    lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
    safe_idxs = np.isfinite(self.ll_t)
    if safe_idxs[0]:
      lane_path_y_interp = np.interp(path_t, self.ll_t[safe_idxs], lane_path_y[safe_idxs])
      path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]

    # debug
    if len(vcurv) > 0:
      sLogger.Send("vC" + "{:.2f}".format(vcurv[0]) + " LX" + "{:.1f}".format(self.lll_y[0]) + " RX" + "{:.1f}".format(self.rll_y[0]) + " LW" + "{:.1f}".format(self.lane_width) + " LP" + "{:.1f}".format(l_prob) + " RP" + "{:.1f}".format(r_prob) + " RS" + "{:.1f}".format(self.rll_std) + " LS" + "{:.1f}".format(self.lll_std))
    else:
      sLogger.Send("vC--- LX" + "{:.1f}".format(self.lll_y[0]) + " RX" + "{:.1f}".format(self.rll_y[0]) + " LW" + "{:.1f}".format(self.lane_width) + " LP" + "{:.1f}".format(l_prob) + " RP" + "{:.1f}".format(r_prob) + " RS" + "{:.1f}".format(self.rll_std) + " LS" + "{:.1f}".format(self.lll_std))

    path_xyz[:, 1] += CAMERA_OFFSET

    return path_xyz

  def get_d_path(self, CS, v_ego, path_t, path_xyz, vcurv):
    if self.BigModel:
      return self.get_nlp_path(CS, v_ego, path_t, path_xyz, vcurv)

    return self.get_stock_path(CS, v_ego, path_t, path_xyz, vcurv)

  def get_nlp_path(self, CS, v_ego, path_t, path_xyz, vcurv):
    # how visible is each lane?
    l_vis = (self.lll_prob * 0.9 + 0.1) * interp(self.lll_std, [0, 0.3, 0.9], [1.0, 0.4, 0.0])
    r_vis = (self.rll_prob * 0.9 + 0.1) * interp(self.rll_std, [0, 0.3, 0.9], [1.0, 0.4, 0.0])
    lane_trust = clamp(1.2 * max(l_vis, r_vis) ** 0.5, 0.0, 1.0)
    # make sure we have something with lanelines to work with
    # otherwise, we will default to laneless
    if lane_trust > 0.025 and len(vcurv) == len(self.lll_y):
      # normalize to 1
      total_prob = l_vis + r_vis
      l_prob = l_vis / total_prob
      r_prob = r_vis / total_prob

      # Find current lanewidth
      width_trust = min(l_vis, r_vis)
      raw_current_width = abs(min(self.rll_y[0], self.re_y[0]) - max(self.lll_y[0], self.le_y[0]))
      current_lane_width = clamp(raw_current_width, MIN_LANE_DISTANCE, MAX_LANE_DISTANCE)
      self.lane_width_estimate.update(current_lane_width)
      speed_lane_width = interp(v_ego, [0., 31.], [TYPICAL_MIN_LANE_DISTANCE, TYPICAL_MAX_LANE_DISTANCE])
      self.lane_width = lerp(speed_lane_width, self.lane_width_estimate.x, width_trust)
      lane_tightness = min(raw_current_width, self.lane_width_estimate.x)

      # track how wide the lanes are getting up ahead
      max_lane_width_seen = current_lane_width
      half_len = len(self.lll_y) // 2

      # additional centering force, if needed
      rightBorder = min(self.re_y[0], self.rll_y[1])
      leftBorder = max(self.le_y[0], self.lll_y[1])
      nearRightEdge = abs(self.rll_y[0] - rightBorder) < MIN_LANE_DISTANCE * 0.8
      nearLeftEdge = abs(self.lll_y[0] - leftBorder) < MIN_LANE_DISTANCE * 0.8
      # ok, how far off of center are we, considering we want to be closer to edges of the road?
      target_centering = self.rll_y[0] + self.lll_y[0]
      # if we are not near an edge, center harder away from it (since it is probably another lane)
      if target_centering > 0:
        # we want to push right, near a left edge?
        target_centering *= 0.9 if nearLeftEdge else 1.5
      elif target_centering < 0:
        # we want to push left, not near a right edge?
        target_centering *= 0.9 if nearRightEdge else 1.5
      # fancy smooth increasing centering force based on lane width
      self.center_force = CENTER_FORCE_GENERAL_SCALE * (TYPICAL_MAX_LANE_DISTANCE / self.lane_width) * target_centering
      # if we are lane changing, cut center force
      self.center_force *= self.lane_change_multiplier
      # if we are in a small lane, reduce centering force to prevent pingponging
      self.center_force *= interp((lane_tightness + self.lane_width) * 0.5, [2.6, 2.8], [0.0, 1.0])
      # likewise if the lane is really big, reduce centering force to not throw us around in it
      self.center_force *= interp((lane_tightness + self.lane_width) * 0.5, [4.0, 6.0], [1.0, 0.0])
      # apply a cap centering force
      self.center_force = clamp(self.center_force, -0.8, 0.8)
      # apply less lane centering for a direction we are already turning
      # this helps avoid overturning in an existing turn
      if math.copysign(1, self.center_force) == math.copysign(1, vcurv[0]):
        self.center_force *= 0.7

      # go through all points in our lanes...
      for index in range(len(self.lll_y) - 1, -1, -1):
        # left/right lane or edge
        right_anchor = min(self.rll_y[index], self.re_y[index])
        left_anchor = max(self.lll_y[index], self.le_y[index])
        # get the raw lane width for this point
        lane_width = right_anchor - left_anchor
        # is this lane getting bigger relatively close to us? useful for later determining if we want to mix in the
        # model path with very large lanes (that might be splitting into multiple lanes)
        if lane_width > max_lane_width_seen and index <= half_len:
          max_lane_width_seen = lane_width
        # how much do we trust this? we want to be seeing both pretty well
        final_lane_width = clamp(min(lane_width, self.lane_width), MIN_LANE_DISTANCE, MAX_LANE_DISTANCE)
        # ok, get ideal point from each lane
        ideal_left = left_anchor + final_lane_width * 0.5
        ideal_right = right_anchor - final_lane_width * 0.5
        # merge them to get an ideal center point, based on which value we want to prefer
        ideal_point = lerp(ideal_left, ideal_right, r_prob)
        # add it to our ultimate path
        self.ultimate_path[index] = ideal_point

      # do we want to mix in the model path a little bit if lanelines are going south?
      ultimate_path_mix = 0.0
      if not self.UseModelPath:
        ultimate_path_mix = lane_trust * interp(max_lane_width_seen, [4.0, 6.0], [1.0, 0.0])
      final_ultimate_path_mix = clamp(self.lane_change_multiplier * ultimate_path_mix, 0.0, 0.8) # always have at least 20% model path in there

      # debug
      sLogger.Send("Cf" + "{:.2f}".format(self.center_force) + " Mx" + "{:.2f}".format(final_ultimate_path_mix) + " vC" + "{:.2f}".format(vcurv[0]) + " LX" + "{:.1f}".format(self.lll_y[0]) + " RX" + "{:.1f}".format(self.rll_y[0]) + " LW" + "{:.1f}".format(self.lane_width) + " LP" + "{:.1f}".format(l_prob) + " RP" + "{:.1f}".format(r_prob) + " RS" + "{:.1f}".format(self.rll_std) + " LS" + "{:.1f}".format(self.lll_std))

      safe_idxs = np.isfinite(self.ll_t)
      if safe_idxs[0] and final_ultimate_path_mix > 0.0:
        path_xyz[:,1] = final_ultimate_path_mix * np.interp(path_t, self.ll_t[safe_idxs], self.ultimate_path[safe_idxs]) + (1 - final_ultimate_path_mix) * path_xyz[:,1]
    else:
      self.center_force = 0.0
      sLogger.Send("Lanes lost completely! Using model path entirely...")

    # apply camera offset and centering force after everything
    path_xyz[:, 1] += CAMERA_OFFSET + self.center_force

    return path_xyz
