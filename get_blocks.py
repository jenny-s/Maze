import sys
sys.path[0:0] = '~/'
import pixy
from ctypes import *

# Pixy Python SWIG get blocks example #

print ("Pixy Python SWIG Example -- Get Blocks")

# Initialize Pixy Interpreter thread #
pixy.pixy_init()

"""
class Blocks (Structure):
  _fields_ = [ ("type", c_uint),
               ("signature", c_uint),
               ("x", c_uint),
               ("y", c_uint),
               ("width", c_uint),
               ("height", c_uint),
               ("angle", c_uint) ]
"""
blocks = pixy.Block()

# Wait for blocks #
while 1:

  count = pixy.pixy_get_blocks(1, blocks)

  if count > 0:
    # Blocks found #
    print '[BLOCK_TYPE=%d SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (blocks.type, blocks.signature, blocks.x, blocks.y, blocks.width, blocks.height)
