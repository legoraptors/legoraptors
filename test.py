#!/usr/bin/env micropython

from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.wheel import Wheel

from legoraptors import LineEdge, Direction, eprint, SimpleRobotDriveBase, avgReflection

# Wheel used by the robot
class Test_Wheel(Wheel):
  def __init__(self):
      Wheel.__init__(self, 62.4, 20)

# Create and initialize an instance of SimpleRobotDriveBase called "robot" 
robot = SimpleRobotDriveBase(
        leftDriveMotor = LargeMotor(OUTPUT_B), 
        rightDriveMotor = LargeMotor(OUTPUT_A),
        tire = Test_Wheel,
        axle_track = 126,
        leftMotor = MediumMotor(OUTPUT_C), 
        rightMotor = MediumMotor(OUTPUT_D),
        leftColor = ColorSensor(INPUT_1),
        rightColor = ColorSensor(INPUT_2),
        centerColor = ColorSensor(INPUT_3),
        useGyro = False
        )

# Examples of function and method calls.
# These assume you have a robot with two drive wheels,
# two auxiliary output motors, and
# two color sensors mounted near the front of your robot.  

# ********************** Drive functions *********************

# drive forward in an arc
robot.driveArc(distance = 500, radius = 300, speed = 200,  direction = Direction.CW, brake = True, block = True)

# # point turn 
# robot.pointTurn(degrees_to_turn = 90, speed = 8, brake = True, block = True)

# # drive straight for given distance (using built-in functions)
# robot.driveStraight(distance = 300, speed = None, brake = True, block = True)

# # drive straight for specified time
# robot.driveStraightForTime(time = 5, speedLeft = None, speedRight = None, brake = True, block = True)

# # drive straight for specified distance
# robot.driveStraightForDistance(distance = 300, speedLeft = None, speedRight = None, stopAtEnd = True)

# # ********************* Gyro functions ***********************

# # point turn using gyro sensor
# robot.pointTurnGyro(degrees_to_turn = 90, speed = 10, brake = True, block = True)

# # drive straight using gyro
# robot.driveStraightGyro(distance = 300, speed = None, brake = False)

# # ******************** Line functions (follow, squareToLine, stopOnLine) ****************************

# # follow line for a specified distance using built-in line follower
# robot.followLineDist(
#     distance = 300, 
#     sensor = robot.leftColor, 
#     sideToFollow = LineEdge.LEFT, 
#     stopAtEnd = True, 
#     speed = 100, 
#     gain_mult = 1,  
#     brake = False)

# # follow line for given distance using our line follower
# robot.followLineDistOriginal(
#     distance = 300, 
#     sensor = robot.leftColor, 
#     sideToFollow = LineEdge.LEFT, 
#     stopAtEnd = True, 
#     speed = 100, 
#     gain_multiplier = 1)

# # position robot perpendicular to line. 
# # assumes robot has two color sensors mounted at the front of the robot:
# # leftColor and rightColor
# robot.squareToLine()

# # drive until one of two front sensors hits a line
# # returns which sensor hit first
# firstHit = robot.stopOnLine(driveForward = True, brake = False)
# eprint(firstHit, "hit line first")

# # drive until specified sensor hits line
# robot.stopOnLineOneSensor(sensor = robot.leftColor, driveForward = False, brake = False)

# # *********************** XY table functions *****************

# # Send XY table to home position (must be called before using other XY table functions)
# robot.homeXY(speed = 60)

# # alternate version of homeXY (limits torque on motors)
# robot.homeXY2(speed = 60)

# # move XY table to specified coordinates
# robot.gotoXY(x = 50, y = 50, speed = 30, block = True)

# # move XY table incrementally from current position
# # Does not check to see if limits will be exceeded!
# robot.incrementXY(dx = 10, dy = 10, speed = 30)

# # *********************** Helper function tests **************

# eprint("This prints to terminal.")

# # read a color sensor and print its value
# reflection  = avgReflection(robot.leftColor, samples = 5)
# eprint("The average reflection value is", reflection)

# speed_in_RPS = robot.speed_mm_s(speed = 100)
# eprint("100 mm/s =  ", speed_in_RPS, "revs/sec." )