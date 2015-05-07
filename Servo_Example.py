#!/usr/bin/python

from AdafruitLibrary.Adafruit_PWM_Servo_Driver.Adafruit_PWM_Servo_Driver import PWM
import time

# ===========================================================================
# Example Code
# ===========================================================================

# Initialise the PWM device using the default address
pwm = PWM(0x40)
# Note if you'd like more debug output you can instead run:
#pwm = PWM(0x40, debug=True)

servo1Max = 500  # Min pulse length out of 4096
servo1Min = 350 # Max pulse length out of 4096
servo2Min = 350
servo2Max = 150

def setServoPulse(channel, pulse):
  pulseLength = 1000000                   # 1,000,000 us per second
  pulseLength /= 60                       # 60 Hz
  print "%d us per period" % pulseLength
  pulseLength /= 4096                     # 12 bits of resolution
  print "%d us per bit" % pulseLength
  pulse *= 1000
  pulse /= pulseLength
  pwm.setPWM(channel, 0, pulse)

pwm.setPWMFreq(60)                        # Set frequency to 60 Hz
while (True):
  # Change speed of continuous servo on channel O
  pwm.setPWM(8, 0, servo1Max)
  pwm.setPWM(15, 0, servo2Max)
  time.sleep(1)
  pwm.setPWM(8, 0, servo1Min)
  pwm.setPWM(15, 0, servo2Min)
  time.sleep(1)



