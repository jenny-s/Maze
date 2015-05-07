#!/usr/bin/python
from AdafruitLibrary.Adafruit_PWM_Servo_Driver.Adafruit_PWM_Servo_Driver import PWM
import pixy
import ctypes

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

# Create a NxN boolean matrix
def matrix(n):
  matrix = []
  for i in range(n):
    inside = []
    for j in range(n):
      inside.append(False)
    matrix.append(inside)
  return matrix

# Print matrix
def printMatrix(m):
  for row in m:
    print row
  print

# Create NxN transformation matrix
def rotate(m, degrees):
  transform = []

  for layer in range(layers): #for each layer
    for i in range(layer, length - layer): #loop through
      temp = m[layer][i] #save the top element, it takes
      #left -> top
      m[layer][i] = m[length - i][layer]
      #Right -> bottom
      m[length - i]

# Calculate error in x direction
def Error_x(x):
  GOAL_X = 135           # Goal x-coordinate
  return GOAL_X - x

# Calculate error in y direction
def Error_y(y):
  GOAL_Y = 135
  return GOAL_Y - y

# Feedback control based on independent PID in x, y
def PID_Control(x, y):   # Currently only does P control
  Kp = 0

  if cmp_x < servoMin: cmp_x = servoMin
  elif cmp_x > servoMax: cmp_x = servoMax
  if cmp_y < servoMin: cmp_y = servoMin
  elif cmp_y > servoMax: cmp_y = servoMax

  setServoPulse(0, cmp_x)
  setServoPulse(1, cmp_y)

# __MAIN__
def main():

  print ("Balance Ball:")

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
  setServoPulse(Servo1, openLoop1)
  setServoPulse(Servo2, openLoop2)

  while True:

    count = pixy.pixy_get_blocks(1, blocks)

    if count > 0:

      # Print coordinates for testing
      print '[BLOCK_TYPE=%d, SIG=%d X=%3d Y=%3d WIDTH-%3d HEIGHT=%3d]' % (blocks.type, blocks.signature, blocks.x, blocks.y, blocks.width, blocks.height)
      x = blocks.x
      y = blocks.y   

      # Transform coordinates
       
    
      # Execute PID control
      PID_Control(x, y)

# Run MAIN
main()
