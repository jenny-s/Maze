# tests for I2C
import time
import math
from AdafruitLibrary.Adafruit_I2C.Adafruit_I2C import Adafruit_I2C

print 'Pi Revision: {}' .format(Adafruit_I2C.getPiRevision())
print 'Bus Number: {}' .format(Adafruit_I2C.getPiI2CBusNumber())

class PWM:
  # Registers/etc
  SELF_TEST_X_GYRO  = 0x00
  SELF_TEST_Y_GYRO  = 0x01
  SELF_TEST_Z_GYRO  = 0x02
  GYRO_CONFIG       = 0x1B
  I2C_MST_CTRL      = 0x24
  I2C_SLV0_ADDR     = 0x25
  I2C_SLV0_REG      = 0x26
  I2C_SLV0_CTRL     = 0x27
  
  
  # Bits
  __RESTART         = 0x80
  __sleep           = 0x10

Gyro = Adafruit_I2C(0x68)

def Start():
  Gyro.write(0x80, 0x68)
