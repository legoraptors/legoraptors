# LegoRaptors library of common functions

from time import sleep
from sys import stderr
from time import sleep
from math import pi

from ev3dev2.motor import SpeedPercent, MoveDifferential, SpeedRPS, SpeedNativeUnits, follow_for_ms
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from ev3dev2.stopwatch import StopWatch

sound = Sound()
leds = Leds()

def eprint(*args, **kwargs):
  ''' Print to terminal. '''
  print(*args, file = stderr, **kwargs)

def limit(x, lowLimit = -100, highLimit = 100):
    ''' Limit x to between lowLimit and highLimit.

    Args:
      x: value to limit
      lowLimit: lowest value to return
      highLimit: highest value to return
    
    Returns:
      x if lowLimit <= x <= highLimit, otherwise lowLimit or highLimit '''
    return min(max(x,lowLimit),highLimit)

def avgReflection(sensor, samples = 2):
  ''' Takes several sensor reflection readings and returns their average. 
  
  Args:
    sensor: color sensor to average.
    samples: number of samples to average

  Returns:
    Average reflection value in percent. 
     '''
  readings = []
  for i in range(samples):
    readings.append(sensor.reflected_light_intensity)
  return(sum(readings)/len(readings))

class Stop:
  ''' Constants for use with motor commands. '''
  BRAKE = True
  COAST = False

class Direction:
  ''' Constants for use with arc commands. '''
  CW = 0
  CCW = 1

class LineEdge:
  ''' Constants to use with line follower. '''
  LEFT = 0
  RIGHT = 1

class SimpleRobotDriveBase(MoveDifferential):
  ''' A simplified drive base class. 
  
  Args:
      leftDriveMotor: motor port for left side drive motor
      rightDriveMotor: motor port for right side drive motor
      tire: tire to use
      axle_track: distance between centerline of drive wheels (mm)
      leftMotor: port for left side attachment motor
      rightMotor: port for right side attachment motor
      leftColor: sensor port for left side color sensor
      rightColor: sensor port for right side color sensor
      centerColor: sensor port for center color sensor
      useGyro (bool): enable gyro functions
      '''

  def __init__(self, leftDriveMotor, rightDriveMotor, tire, axle_track,
               leftMotor, rightMotor, leftColor, rightColor, centerColor, useGyro = False):
    ''' Initialize the SimpleRobotDriveBase. 
    
      '''
    self.defaultSpeed = 200 # mm/s
    self.defaultTurnTime = 2 # time to complete turn in place (s)
    self.blackThreshold = 10 # values less than this are black
    self.whiteThreshold = 55 # values more than this are white
    self.defaultTimeout = 15000

    # Set up two drive motors
    self.leftDriveMotor = leftDriveMotor
    self.rightDriveMotor = rightDriveMotor
        
    # set up output shafts
    self.leftMotor = leftMotor
    self.rightMotor = rightMotor

    # aliases for output shafts when using XY table
    self.xMotor = self.leftMotor
    self.yMotor = self.rightMotor

    # set up color sensors
    self.leftColor = leftColor
    self.rightColor = rightColor
    self.centerColor = centerColor

    # set up gryo sensor
    if useGyro:
      self.gyro = GyroSensor()
      self.gyro.reset()

    # store wheel diameter
    self.wheel_diameter = tire().diameter_mm

    # initialize the parent class: MoveDifferential
    super().__init__(leftDriveMotor.address, rightDriveMotor.address, tire, axle_track)

    # # Invert motor polarities: positive drives CCW
    # rightDriveMotor.polarity = "inversed"
    # leftDriveMotor.polarity = "inversed"

    # set the ratio of motor angle to linear motion (deg/mm)
    self.xRatio = 2392 / 144 # deg/mm 
    self.yRatio = 1695 / 80 # deg/mm

    # set time to accelerate or deccelerate to 100% speed (ms)
    self.ramp_time = 1000
    self.leftDriveMotor.ramp_up_sp = self.ramp_time
    self.leftDriveMotor.ramp_down_sp = self.ramp_time
    self.rightDriveMotor.ramp_up_sp = self.ramp_time
    self.rightDriveMotor.ramp_down_sp = self.ramp_time
    self.leftMotor.ramp_up_sp = self.ramp_time
    self.leftMotor.ramp_down_sp = self.ramp_time
    self.rightMotor.ramp_up_sp = self.ramp_time
    self.rightMotor.ramp_down_sp = self.ramp_time

  def speed_mm_s(self, speed):
    ''' Convert speed in mm/s into percent of max speed.
    
    Args:
      speed: speed in mm/s.
      
    Returns:
      speed in SpeedRPS form. '''
    rps = speed/(pi*self.wheel_diameter)
    speed=SpeedRPS(rps)
    return speed
  

  def driveArc(self, distance, radius, speed = 200,  direction = Direction.CW, brake = True, block = True):
    ''' Drive for given distance along arc of specified radius.

    Args:
      distance: distance to drive in mm
      radius: radius of arc in mm
      speed: speed in mm/s
      direction: direction to travel along arc -- Direction.CW or Direction.CCW
      brake (bool): brake at end of move
      block (bool): block until move is complete
    '''
    speed = self.speed_mm_s(speed)

    self.enableRampDown(brake)

    if direction == Direction.CW:
      self.on_arc_right(speed, radius, distance, brake, block)
    if direction == Direction.CCW:
      self.on_arc_left(speed, radius, distance, brake, block)

    self.enableRampDown(True)

  def pointTurn(self, degrees_to_turn, speed = 8, brake = True, block = True):
    ''' Turn in place.

    Args:
      degrees_to_turn: degrees to turn (positive to right)
      speed: percentage of maximum speed
      brake (bool): brake at end of move
      block (bool): block until move is complete
    '''

    self.enableRampDown(brake)

    if degrees_to_turn >= 0:
      self.turn_right(speed, degrees_to_turn, brake, block)
    else:
      self.turn_left(speed, degrees_to_turn, brake, block)

    self.enableRampDown(True)

  def pointTurnGyro(self, degrees_to_turn, speed = 10, brake = True, block = True):
    ''' Turn in place using gyro sensor.

    Args:
      degrees_to_turn: degrees to turn (positive to right)
      speed: percentage of maximum speed
      brake (bool): brake at end of move
      block (bool): block until move is complete (unused for now)
    '''
    startAngle = self.gyro.angle
    # start rotation
    if degrees_to_turn >= 0:
      self.turn_right(speed, 5000, brake, False)
    else:
      self.turn_left(speed, 5000, brake, False)
    # loop until we hit target angle
    while self.gyro.angle - startAngle < degrees_to_turn:
      sleep(.01)
    # stop both motors
    self.stop(brake = brake)

  def enableRampDown(self, enable):
    ''' Enable or disable the motor decceleration for drive motors.

    Args:
      enable (bool): Enable deccleration if True
    '''

    if enable:
      self.leftDriveMotor.ramp_down_sp = self.ramp_time
      self.rightDriveMotor.ramp_down_sp = self.ramp_time
    else:
      self.leftDriveMotor.ramp_down_sp = 0
      self.rightDriveMotor.ramp_down_sp = 0
    
  def driveStraight(self, distance, speed = None, brake = True, block = True):
    ''' Drive for given distance.

    Args:
      distance: distance to drive in mm
      speed: speed in mm/s. Uses default speed if not specified.
      brake (bool): brake at end of movement
      block (bool): block until move is complete
    '''
    if speed == None or speed == 0:
      speed = self.defaultSpeed
    
    speed = self.speed_mm_s(speed)

    self.enableRampDown(brake)
    
    self.on_for_distance(speed, distance, brake, block)

    self.enableRampDown(True)

  def driveStraightForTime(self, time, speedLeft = None, speedRight = None, brake = True, block = True):
    ''' Drive for given time.

    Args:
      time: time for move in s
      speedLeft: speed in mm/s (left motor)
      speedRight: speed in mm/s (right motor)
      brake (bool): brake at end of movement
      block (bool): block until move is complete
    '''
    if speedLeft == None or speedLeft == 0:
      speedLeft = self.defaultSpeed
    
    speedLeft = self.speed_mm_s(speedLeft)

    if speedRight == None or speedRight == 0:
      speedRight = self.defaultSpeed
    
    speedRight = self.speed_mm_s(speedRight)

    self.enableRampDown(brake)
    
    self.on_for_seconds(speedLeft, speedRight, time, brake, block)

    self.enableRampDown(True)


  def driveStraightForDistance(self, distance, speedLeft = None, speedRight = None, stopAtEnd = True):
    ''' Drive for given distance.

    Args:
      distance: distance to drive in mm
      speedLeft: speed in mm/s (left motor)
      speedRight: speed in mm/s (right motor)
      stopAtEnd (bool): brake at end of movement
    '''
    if speedLeft == None or speedLeft == 0:
      speedLeft = self.defaultSpeed
    
    if speedRight == None or speedRight == 0:
      speedRight = self.defaultSpeed
    
    degPerSecLeft = 360*speedLeft/(pi*self.wheel_diameter)
    degPerSecRight = 360*speedRight/(pi*self.wheel_diameter)

    driveTime = StopWatch()
    driveTime.start()
    
    self.leftDriveMotor.on(SpeedNativeUnits(degPerSecLeft), block = False)
    self.rightDriveMotor.on(SpeedNativeUnits(degPerSecRight), block = False)

    while driveTime.value_ms <= abs(distance/speedLeft)*1000:
      sleep(0.02)

    if stopAtEnd:
      self.leftDriveMotor.stop()
      self.rightDriveMotor.stop()
      sleep(0.01)
    
  def driveStraightGyro(self, distance, speed = None, brake = False):
    ''' Drive for given distance using gyro sensor.

    Robot goes in straight line in the direction it was facing when this 
    function is called. Uses proportional control.

    Args:
      distance: distance to drive in mm
      speed: speed in mm/s
      brake (bool): brake at end of movement
    '''
    # starting angle for this move
    startAngle = self.gyro.angle

    if speed == None or speed == 0:
      speed = self.defaultSpeed

    if distance < 0:
      distance = abs(distance)
      speed = -speed

    assert (distance > 0 or speed > 0), "At least one of distance or speed must be posiitve in driveStraightGyro."

    # calculate motor speeds in percentage units
    rps = speed/(pi*self.wheel_diameter)
    left_speed = rps
    right_speed = rps
    
    # set encoder counts to zero
    self.rightDriveMotor.position = 0

    # proportional gain (rps/deg)
    kp = 0.01

    # number of wheel rotations in this move
    targetRotations = distance/(pi*self.wheel_diameter)

    while abs(self.rightDriveMotor.rotations) < targetRotations:
      error = self.gyro.angle - startAngle
      left_speed = SpeedRPS(rps - kp*error)
      right_speed = SpeedRPS(rps + kp*error)
      self.on(left_speed, right_speed)
      sleep(0.01)
    
    # stop both motors
    self.stop(brake = brake)

  def followLineDistNative(self, distance, sensor, sideToFollow, stopAtEnd, speed, gain_mult = 1,  brake = False):
    ''' Drive forward following line. 

    Args:
      distance: distance to follow line in mm
      sensor: colorsensor to use for line following
      sideToFollow: which side of line to follow:  LineEdge.LEFT or LineEdge.RIGHT
      stopAtEnd (bool): stop at end of movement
      speed: speed in mm/s
      gain_mult: multiplier for P and D gains
      brake (bool): brake at end of move
    '''
    # calculate the target reflectance, halfway between black ad white
    target = (self.blackThreshold + self.whiteThreshold) / 2

    # set parameters for which side to follow
    if sideToFollow == LineEdge.LEFT:
      followLeft = True
    else:
      followLeft = False

    # calculate the required motor rotation rate in RPS
    rotationRate = speed/(pi*self.wheel_diameter)

    # time to follow line
    timeToFollow_ms = abs(distance/speed*1000)

    # set which color sensor to use
    self.cs = sensor

    # call line follower built-in function
    self.follow_line(
      kp = 5 * gain_mult,
      ki = 0 * gain_mult,
      kd = 1 * gain_mult,
      speed = SpeedRPS(rotationRate),
      target_light_intensity = target, 
      follow_left_edge = followLeft,
      white = self.whiteThreshold,
      off_line_count_max = 100,
      sleep_time = 0.01,
      follow_for = follow_for_ms,
      ms = timeToFollow_ms)

    self.stop(brake = brake)

  def followLineDist(self, distance, sensor, sideToFollow, stopAtEnd, speed, gain_multiplier = 1):
    ''' Drive forward following line.  

    Uses direct control of drive motors and PD control. 

    Args:
      distance: distance to follow line in mm
      sensor: colorsensor to use for line following
      sideToFollow: which side of line to follow:  LineEdge.LEFT or LineEdge.RIGHT
      stopAtEnd (bool): Stop motors at end of line following
      speed: speed in mm/s
      gain_multiplier: multiplier for P and D gains
      
    '''
    degPerSec = 360*speed/(pi*self.wheel_diameter)
    propGain = 6.0 * gain_multiplier
    derivGain = 6 * gain_multiplier

    target = (self.blackThreshold + self.whiteThreshold) / 2

    print("******* starting line following ********")
    # print("error,Pterm,Dterm")

    # initialize term for derivative calc
    previousError = target - avgReflection(sensor, 2)
    
    i = 0
    driveTime = StopWatch()
    driveTime.start()

    while driveTime.value_ms <= abs(distance/speed)*1000:
      # calc error and proportional term
      error = target - avgReflection(sensor, 2)
      Pterm = propGain * error

      # calc d(error)/d(t) and derivative term
      d_error_dt = error - previousError
      Dterm = derivGain * d_error_dt
      previousError = error

      if sideToFollow == LineEdge.RIGHT:
        self.leftDriveMotor.on(SpeedNativeUnits(degPerSec + Pterm + Dterm),block=False)
        self.rightDriveMotor.on(SpeedNativeUnits(degPerSec - Pterm - Dterm),block=False)
        # eprint("{:7.2f},{:7.2f},{:7.2f}".format(error, Pterm, Dterm)) # for debugging

      if sideToFollow == LineEdge.LEFT:
        self.leftDriveMotor.on(SpeedNativeUnits(degPerSec - Pterm - Dterm),block=False)
        self.rightDriveMotor.on(SpeedNativeUnits(degPerSec + Pterm + Dterm),block=False)
        # eprint("{:7.2f},{:7.2f},{:7.2f}".format(error, Pterm, Dterm)) # for debugging

    # eprint("line following complete")
 
    if stopAtEnd:
      self.leftDriveMotor.stop()
      self.rightDriveMotor.stop()
      sleep(0.01)
    
    # Play sound when we stop line following
    sound.beep()  

  def homeXY(self, speed = 60):
    ''' Move xy table to home position.

    Args:
      speed: linear speed of the axes (mm/s)
    '''
    # self.xMotor.set_dc_settings(80, 0)
    self.xMotor.position = 0
    self.yMotor.position = 0
    self.gotoXY(-200, -200, speed)
    self.xMotor.position = 0
    self.yMotor.position = 0
    # self.xMotor.set_dc_settings(100, 0)

  def homeXY2(self, speed = 60):
    ''' Move xy table to home position.

    Args:
      speed: linear speed of the axes (mm/s)
    '''
    eprint("starting homeXY")
    # run motors at fixed duty cycle
    self.yMotor.duty_cycle_sp = -90
    self.xMotor.run_direct(duty_cycle_sp = -50)
    self.yMotor.run_direct(duty_cycle_sp = -80)

    # sleep until motors stop (or timeout is reached)
    xExitType = self.xMotor.wait_until_not_moving(self.defaultTimeout)
    yExitType = self.yMotor.wait_until_not_moving(self.defaultTimeout)

    # set xMotor and yMotor encoders to zero 
    self.xMotor.position = 0
    self.yMotor.position = 0

    eprint("HomeXY complete.",xExitType, yExitType)
  

  def gotoXY(self, x, y, speed = 30, block = True):
      ''' Move to absolute position (x,y).
      
      moves axes simultaneously.
      homeXY must be called first.

      Args:
        x: position of the x-axis (mm)
        y: position of the y-axis (mm)
        speed: the speed of the axes (mm/s)
        block (bool): block until move is complete
      '''
      # start move to x position 
      self.xMotor.on_to_position(
        SpeedNativeUnits(speed*self.xRatio),
        x*self.xRatio,
        brake = False,
        block  = False
        )

      # start move to y position 
      self.yMotor.on_to_position(
        SpeedNativeUnits(speed*self.yRatio),
        y*self.yRatio,
        brake = False,
        block = False
        )
        
      if block:
        # sleep until either move is complete or motors stall
        self.xMotor.wait_until_not_moving(self.defaultTimeout)
        self.yMotor.wait_until_not_moving(self.defaultTimeout)
  
  def incrementXY(self,dx,dy,speed = 30):
    ''' Move xy table by a given amount.

      Moves axes simultaneously.

      Args:
        dx: amount to move x-axis (mm)
        dy: amount to move  y-axis (mm)
        speed: the speed of the axes (mm/s)
    '''
    # get current table positions in mm from home position
    x = self.xMotor.position/self.xRatio
    y = self.yMotor.position/self.yRatio
    eprint("XY table position",x,y)

    self.gotoXY(x + dx, y + dy, speed)

  def squareToLine(self):
    ''' Position robot perpendicular to a line.

    '''
    # drive forward until one sensor is on edge of line. 
    firstSensor = self.stopOnLine()
    sound.beep()
    target = (self.blackThreshold + self.whiteThreshold)/2
    # target = self.blackThreshold
    targetTolerance = 2 # how close we need to be to target reflectance
    leds.set_color('LEFT','RED')
    # drive opposite wheel forward to square robot on line
    if firstSensor == self.leftColor:  # left sensor on line
      eprint("found left sensor first")
      # start right wheel rotating
      self.rightDriveMotor.speed_sp = 30
      self.rightDriveMotor.run_forever()
      self.leftDriveMotor.speed_sp = -30
      self.leftDriveMotor.run_forever()
      # drive slowly forward until sensor hits white 
      while True:
        if avgReflection(self.rightColor) >= self.whiteThreshold:
          # found a white line
          eprint("found white")
          break
        sleep(0.010)
      while True:
        if abs(avgReflection(self.rightColor) - target) < targetTolerance :
          # we are centered over edge of line
          self.rightDriveMotor.stop()
          self.leftDriveMotor.stop()
          eprint("found line edge")
          break
        sleep(0.010)
    else: # right sensor is on line
      # start left wheel rotating
      eprint("found right sensor first")
      self.leftDriveMotor.on(30)
      while True:
        if avgReflection(self.leftColor) >= self.whiteThreshold:
          # found a white line
          break
      while True:
        if abs(avgReflection(self.leftColor) - target) < targetTolerance :
          # we are centered over edge of line
          self.leftDriveMotor.stop()
          break
        sleep(0.010)

    # set brick light when we are done 
    leds.set_color('LEFT','GREEN')
    # play long beep
    sound.beep()
    eprint(avgReflection(self.rightColor), avgReflection(self.leftColor,5))

  def stopOnLine(self, driveForward = True, brake = False):
    ''' Stop when one sensor is centered over edge of a line.

        Drives slowly to line. 

        Args:
          driveForward (bool): drive in forward direction
          brake (bool): brake at end of move

        Returns:
          firstSensor: the sensor that hit line first
    '''
    # calculate the target sensor reading: halfway between black and white
    target = (self.blackThreshold + self.whiteThreshold)/2
    # set brick light while we are seeking white 
    leds.set_color('LEFT','RED')
    if driveForward:
      # drive slowly until at least one sensor hits white 
      self.on(left_speed = 5, right_speed = 5) # speeds in percent
    else:
      # drive slowly until at least one sensor hits white 
      self.on(left_speed = -5, right_speed = -5) # speeds in percent

    while True:
      if avgReflection(self.leftColor) >= self.whiteThreshold:
        # found a white line
        break
      if avgReflection(self.rightColor) >= self.whiteThreshold:
        # found a white line
        break
      sleep(0.010)

    # set brick light while we are seeking the line 
    leds.set_color('LEFT','ORANGE')

    targetTolerance = 5 # how close we need to be to target reflectance

    # continue driving until we see the target reflectance
    while True:
      if abs(avgReflection(self.leftColor) - target) < targetTolerance :
        # we are centered over edge of line
        firstSensor = self.leftColor # left sensor hit first
        eprint("left sensor hit first in stop on line")
        break
      if abs(avgReflection(self.rightColor) - target) < targetTolerance :
        # we are centered over edge of line
        firstSensor = self.rightColor # right sensor hit first
        eprint("right sensor hit first in stop on line")
        break
      sleep(0.010) 

    # stop moving
    self.stop(brake = brake)
    return(firstSensor)

  def stopOnLineOneSensor(self,  sensor, driveForward = False, brake = False):
    ''' Stop when given sensor is centered over edge of a line.

        Drives slowly to line. 

        Args:
          sensor: Color sensor to use
          Drive forward (bool): weather to drive forward
          brake (bool): brake at end of move

        Returns:
          distanceTravelled: distance travelled in mm
    '''
    # calculate the target sensor reading: halfway between black and white
    target = (self.blackThreshold + self.whiteThreshold) / 2
  
    # set brick light while we are seeking white
    leds.set_color('LEFT','RED')

    # set encoder counts to zero
    self.rightDriveMotor.position = 0
    if driveForward:
      # drive slowly until at least one sensor hits white 
      self.on(left_speed = 15, right_speed = 15) # speeds in percent
    else:
      # drive slowly until at least one sensor hits white 
      self.on(left_speed = -15, right_speed = -15) # speeds in percent

    while avgReflection(sensor) < self.whiteThreshold:
        sleep(0.010)

    targetTolerance = 5 # how close we need to be to target reflectance
    leds.set_color('LEFT','ORANGE')

    # continue driving until we see the target reflectance
    while abs(avgReflection(sensor) - target) > targetTolerance: 
      sleep(0.010) 

    # stop moving
    self.stop(brake = brake)

    leds.set_color('LEFT','GREEN')

    distanceTravelled = self.rightDriveMotor.rotations * self.wheel_diameter * pi
    eprint("Distance travelled:", distanceTravelled)
    return distanceTravelled
