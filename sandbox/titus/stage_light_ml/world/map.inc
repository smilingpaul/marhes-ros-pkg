# map.inc - useful setup for a floorplan bitmap
# Authors: Richard Vaughan
# $Id$

define floorplan model
(
  # sombre, sensible, artistic
  color "gray30"

  # most maps will need a bounding box
  boundary 1

  gui_nose 0
  gui_grid 0
  gui_move 0
  gui_outline 0
  gripper_return 0
  fiducial_return 0
  laser_return 1
)

define zone model
(
#  color	"orange"
  size [ 0.8 0.8 0.0 ]

  gui_nose 0
  gui_grid 0
  gui_move 0
  gui_outline 0

  # insensible to collision and range sensors
  obstacle_return 0
  laser_return 0
  ranger_return 0  
)

# a block for gripping
define puck model(
  size [ 0.5 0.5 0.5 ]
  obstacle_return 1
  gripper_return 1
  gui_move 0
  gui_nose 0
  gui_outline 1
#  fiducial_return 2
  laser_return 1
  ranger_return 1
)

