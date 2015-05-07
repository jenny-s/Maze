#!/usr/bin/python
from AdafruitLibrary.Adafruit_PWM_Servo_Driver.Adafruit_PWM_Servo_Driver import PWM
import pixy
import ctypes
import math

# set servo channels
servo1 = 0
servo2 = 15

# Set servo pulse with frequency of 60Hz
def setServoPulse(channel, pulse):
  pulseLength = 1000000                   # 1,000,000 us per second
  pulseLength /= 60                       # 60 Hz
  print "%dus per period" % pulseLength
  pulseLength /= 4096                     # 12 bits of resolution
  print "%d us per bit" % pulseLength
  pulse *= 1000
  pulse /= pulseLength
  pwm.setPWM(channel, 0, pulse)

# Transform vector into new plane rotated by 45 degrees
def transform(vector):
  coordinates = [0, 0]
  coordinates[0] = math.cos(45) * vector[0] - math.sin(45) * vector[1]
  coordinates[1] = math.sin(45) * vector[0] + math.cos(45) * vector[1]
  return coordinates

# Calculate error in x direction
def Error_x(x):
  GOAL_X = 166           # Goal x-coordinate
  return GOAL_X - x

# Calculate error in y direction
def Error_y(y):
  GOAL_Y = 88
  return GOAL_Y - y

# Feedback control based on independent PID in x, y
def PID_Control(x, y):   # Currently only does P control
  Kp = 0

  cmp_x = Kp * Error_x(x)
  cmp_y = Kp * Error_y(y)

  if cmp_x < servoMin: cmp_x = servoMin
  elif cmp_x > servoMax: cmp_x = servoMax
  if cmp_y < servoMin: cmp_y = servoMin
  elif cmp_y > servoMax: cmp_y = servoMax

  setServoPulse(servo1, cmp_x)
  setServoPulse(servo2, cmp_y)

# __MAIN__
def main():
  # Vector in x-y plane
  vector = [0,0]
  
  # Transform matrix
  coordinates = transform(vector)

  # Initialize Pixy Interpreter thread #
  pixy.pixy_init()
  blocks = pixy.Block()

  # Initialize the PWM device using the default address
  pwm = PWM(0x40)

  # Uncomment for debugging mode
  # pwm = PWM(0x40, debug = True)

  # Define suitable pulse ranges
  servo1Min = 350 
  servo1Max = 500
  servo2Min = 350
  servo2Max = 150

  # Initialize servos
  pwm.setPWMFreq(60)   # Set PWM frequencies to 60Hz
  setServoPulse(servo1, 400)
  setServoPulse(servo2, 200)

  while True:

    count = pixy.pixy_get_blocks(1, blocks)

    if count > 0:

      # Print coordinates for testing
      print '[BLOCK_TYPE=%d, SIG=%d X=%3d Y=%3d WIDTH-%3d HEIGHT=%3d]' % (blocks.type, blocks.signature, blocks.x, blocks.y, blocks.width, blocks.height)
      vector = [blocks.x, blocks.y]

      # Transform coordinates
      coordinates = transform(vector)
    
      # Execute PID control
      PID_Control(coordinates[0], coordinates[1])

# Run MAIN
main()
