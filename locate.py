#!/usr/bin/python
import sys
sys.path[0:0] = '~/'
import pixy
from ctypes import *

# ==========================================================================
# Prints coordinates of object
# ==========================================================================

# Initialize Pixy Interpreter thread #
pixy.pixy_init()

# NOT USED #
class Blocks (Structure):
  _fields_ = [ ("type", c_uint),
               ("signature", c_uint),
               ("x", c_uint),
               ("y", c_uint),
               ("width", c_uint),
               ("height", c_uint),
               ("angle", c_uint) ]

# Create block object #
blocks = pixy.Block()

# Wait for blocks #
while True:

  count = pixy.pixy_get_blocks(1, blocks)

  if count > 0:
    # Blocks found #
    print '[X=%3d Y=%3d]' % (blocks.x, blocks.y)
