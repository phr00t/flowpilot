import math
from numpy import interp

MIN_LANE_DISTANCE = 2.6
MAX_LANE_DISTANCE = 3.7
TYPICAL_MIN_LANE_DISTANCE = 2.7
TYPICAL_MAX_LANE_DISTANCE = 3.4
CENTER_FORCE_GENERAL_SCALE = 0.4
selflane_change_multiplier = 1.0

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

# additional centering force, if needed
def calcTest(selfrll_y0, selflll_y0, selfre_y0, selfle_y0, selfrll_y1, selflll_y1, selflane_width, vcurv0, run):
    lane_tightness = selfrll_y0 - selflll_y0
    rightBorder = min(selfre_y0, selfrll_y1)
    leftBorder = max(selfle_y0, selflll_y1)
    nearRightEdge = abs(selfrll_y0 - rightBorder) < MIN_LANE_DISTANCE * 0.7
    nearLeftEdge = abs(selflll_y0 - leftBorder) < MIN_LANE_DISTANCE * 0.7
    targetRightCentering = min(selfrll_y0 + 0.15, rightBorder) if nearRightEdge else selfrll_y0
    targetLeftCentering = max(selflll_y0 - 0.15, leftBorder) if nearLeftEdge else selflll_y0
    # ok, how far off of center are we, considering we want to be closer to edges of the road?
    target_centering = targetRightCentering + targetLeftCentering
    # fancy smooth increasing centering force based on lane width
    selfcenter_force = CENTER_FORCE_GENERAL_SCALE * (TYPICAL_MAX_LANE_DISTANCE / selflane_width) * target_centering# * target_centering
    # make sure we get the sign right after squaring
    #if target_centering < 0:
    #  selfcenter_force = -selfcenter_force
    # if we are lane changing, cut center force
    selfcenter_force *= selflane_change_multiplier
    # if we are in a small lane, reduce centering force to prevent pingponging
    selfcenter_force *= interp((lane_tightness + selflane_width) * 0.5, [2.6, 2.8], [0.0, 1.0])
    # apply a cap centering force
    selfcenter_force = clamp(selfcenter_force, -0.8, 0.8)
    # apply less lane centering for a direction we are already turning
    if math.copysign(1, selfcenter_force) == math.copysign(1, vcurv0):
      selfcenter_force *= 0.8
  
    print(run, ' ]------------------------------------------')
    print('Center Force: ', selfcenter_force)
    print('Near Edges Left/Right?', nearLeftEdge, nearRightEdge)
    print('Target Centering: ', target_centering)
    print('Lane Tightness: ', lane_tightness)

# selfrll_y0, selflll_y0, selfre_y0, selfle_y0, selfrll_y1, selflll_y1, selflane_width, vcurv0
calcTest(1.75,     -1.24,      1.84,      -6.0,        2.4,       -4.5,            3.0,  -0.36, 1)
calcTest(1.5,       -1.5,       1.8,      -5.0,        2.4,       -4.5,            3.0,    0.0, 2)
calcTest(1.1,       -1.9,       1.2,      -6.0,        2.4,       -4.5,            3.0,   0.36, 3)
calcTest(2.0,       -0.9,       2.1,      -6.0,        2.4,       -4.5,            2.9,   -0.6, 4)
calcTest(2.2,       -1.0,       2.3,      -6.0,        2.4,       -4.5,            3.0,  -0.18, 5)
