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

#servo1 = 395  # Min pulse length out of 4096
#servo2 = 318

servo1 = 625  # Min pulse length out of 4096
servo2 = 540

def setServoPulse(channel, pulse):
  pulseLength = 1000000                   # 1,000,000 us per second
  pulseLength /= 60                       # 60 Hz
  print "%d us per period" % pulseLength
  pulseLength /= 4096                     # 12 bits of resolution
  print "%d us per bit" % pulseLength
  pulse *= 1000
  pulse /= pulseLength
  pwm.setPWM(channel, 0, pulse)

pwm.setPWMFreq(100)                        # Set frequency to 60 Hz
while (True):
  # Change speed of continuous servo on channel O
  pwm.setPWM(0, 0, servo1)
  pwm.setPWM(15, 0, servo2)


