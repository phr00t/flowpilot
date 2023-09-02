import numpy as np
from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp
from common.realtime import DT_MDL
from system.swaglog import cloudlog
from common.logger import sLogger

TRAJECTORY_SIZE = 33
# positive numbers go right
CAMERA_OFFSET = 0.09
KEEP_MIN_DISTANCE_FROM_LANE = 1.2

def clamp(num, min_value, max_value):
  return max(min(num, max_value), min_value)

class LanePlanner:
  def __init__(self):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.le_y = np.zeros((TRAJECTORY_SIZE,))
    self.re_y = np.zeros((TRAJECTORY_SIZE,))
    self.lane_width_estimate = FirstOrderFilter(3.2, 9.95, DT_MDL)
    self.lane_width = 3.2
    self.lane_change_multiplier = 1

    self.lll_prob = 0.
    self.rll_prob = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

  def parse_model(self, md):
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

  def get_d_path(self, CS, v_ego, path_t, path_xyz):
    # Reduce reliance on uncertain lanelines
    l_prob = self.lll_prob * interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_prob = self.rll_prob * interp(self.rll_std, [.15, .3], [1.0, 0.0])

    # always consider seeing the lanes, just prefer the one more likely be the model and stds
    # normalize to always be 1
    total_prob = l_prob + r_prob
    if total_prob < 0.01:
      l_prob = 0.5
      r_prob = 0.5
    else:
      l_prob = l_prob / total_prob
      r_prob = r_prob / total_prob

    # Find current lanewidth
    current_lane_width = clamp(abs(min(self.rll_y[0], self.re_y[0]) - max(self.lll_y[0], self.le_y[0])), 2.6, 4.0)
    self.lane_width_estimate.update(current_lane_width)
    self.lane_width = self.lane_width_estimate.x

    # ideally we are half distance of lane width
    # but clamp lane distances to not push us over the current lane width
    use_min_distance = min(current_lane_width * 0.5, KEEP_MIN_DISTANCE_FROM_LANE)
    lane_distance = clamp(self.lane_width * 0.5, use_min_distance, current_lane_width - use_min_distance)

    # debug
    sLogger.Send("LX" + "{:.1f}".format(self.lll_y[0]) + " RX" + "{:.1f}".format(self.rll_y[0]) + " LW" + "{:.1f}".format(self.lane_width) + " LP" + "{:.1f}".format(l_prob) + " RP" + "{:.1f}".format(r_prob))

    path_from_left_lane = self.lll_y + lane_distance
    path_from_right_lane = self.rll_y - lane_distance

    lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane)
    path_xyz[:,1] = self.lane_change_multiplier * np.interp(path_t, self.ll_t[safe_idxs], lane_path_y[safe_idxs]) + (1 - self.lane_change_multiplier) * path_xyz[:,1]

    # apply path offset after everything
    path_xyz[:, 1] += CAMERA_OFFSET

    return path_xyz
