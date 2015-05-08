#!/usr/bin/python
from AdafruitLibrary.Adafruit_PWM_Servo_Driver.Adafruit_PWM_Servo_Driver import PWM
from collections import deque
import pixy
import ctypes
import math
import time

# ==================================================================================
# PID control to balance ball on platform with 2 degrees of freedom
# ==================================================================================

# CONTROL ON
ON = True

############################### __GLOBAL VARIABLES__ ###############################
# Set PID constants
Kp = 1.0
Ki = 0.0
Kd = 100.0

# Set CENTER and GOAL coordinates
CENTER = [119, 96]
GOAL = [33, 37]
angle = 40
V1 = []
V2 = []
V3 = []
V4 = []

# Set servo channels
servo1 = 0
servo2 = 15

# Initialize Pixy Interpreter thread
pixy.pixy_init()
blocks = pixy.Block()

# Initialize the PWM device using the default address
pwm = PWM(0x40)

# Uncomment for debugging mode
# pwm = PWM(0x40, debug = True)

# Define suitable pulse ranges
servo1Min = 10.0
servo1Max = 600.0
servo2Max = 600.0
servo2Min = 10.0

# Set open loop pulses
openLoop1 = 450
openLoop2 = 340

# vars for integral control
xIntegralSum = 0
yIntegralSum = 0

# vars for derivative control
xQue = deque()
yQue = deque()
count = 0
prevTime = 0

################################## __FUNCTIONS__ ###################################
# Get system time to seconds
def Time():
  return time.clock() / 1000000

# Transform vector into new plane rotated by 45 degrees
def transform(vector):
  coordinates = [0, 0]
  translation = [0, 0]
  translation[0] = -(vector[0] - CENTER[0])
  translation[1] = vector[1] - CENTER[1]
  coordinates[0] = math.cos(angle) * translation[0] + math.sin(angle) * translation[1]
  coordinates[1] = -math.sin(angle) * translation[0] + math.cos(angle) * translation[1]

  return coordinates

# Calculate error in x direction
def Error_x(x):
  return GOAL[0] - x

# Calculate error in y direction
def Error_y(y):
  return GOAL[1] - y

# Integral Control
def IControl(x, y):
  global xNavIntegralSum, yIntegralSum

  xIntegralSum = xIntegralSum + Error_x(x)
  yIntegralSum = yIntegralSum + Error_y(y)

# Derivative control
def DControl(x, y):
  global xQue, yQue, count

  dx = 0
  dy = 0

  if count < 100:
    xQue.append(x)
    yQue.append(y)
    count = count + 1
  elif count == 100: 
    dx = xQue.popleft() - xQue.pop()
    dy = yQue.popleft() - yQue.pop()
    count = count - 2
  dx = dx / 5
  dy = dy / 5
      
  return [dx, dy]

# Feedback control based on independent PID in x, y
def PID_Control(x, y):   # Currently only does P control  
  # Calculate feedback for x and y, respectively
  cmp_x = openLoop1 + Kp * Error_x(x) + Ki * xIntegralSum + Kd * DControl(x, y)[0]
  cmp_y = openLoop2 - Kp * Error_y(y) + Ki * yIntegralSum + Kd * DControl(x, y)[1]

  if cmp_x < servo1Min: cmp_x = servo1Min
  elif cmp_x > servo1Max: cmp_x = servo1Max
  if cmp_y > servo2Max: cmp_y = servo2Max
  elif cmp_y < servo2Min: cmp_y = servo2Min

  pwm.setPWM(servo1, 0, int(cmp_x))
  pwm.setPWM(servo2, 0, int(cmp_y))

##################################### __MAIN__ #####################################
def main():
  global prevTime

  # Vector in x-y plane
  vector = [0,0]
  
  # Transform matrix
  coordinates = transform(vector)

  # Initialize servos
  pwm.setPWMFreq(60)   # Set PWM frequencies to 60Hz
  pwm.setPWM(servo1, 0, openLoop1)
  pwm.setPWM(servo2, 0, openLoop2)

  while True:
    t = Time()
    #print "time: ", t - prevTime

    count = pixy.pixy_get_blocks(1, blocks)

    if count > 0:

      # Save coordinates of ball
      vector = [blocks.x, blocks.y]
      
      # Use coordinates if valid
#      if vector[0] > 60 and vector[0] < 250 and vector[1] < 200:
      if True:
        # Transform coordinates
        coordinates = transform(vector)
        
        # Print statements for testing
        print "transformed: ", coordinates
        #print "error: ", Error_x(coordinates[0]), Error_y(coordinates[1])
        
        # Execute PID control
        if ON:
          PID_Control(coordinates[0], coordinates[1])

    prevTime = t

################################## __Run MAIN__ ####################################
main()
 
