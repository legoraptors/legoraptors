Python library for FLL robots
===================================



A Python3 library implementing a high-level interface for FLL robots 
using the ev3dev2 library.

Developed by the Lego Raptors team (Boulder, Colorado, USA) for use in
the 2019-20 FLL CityShaper competition, but useful in general. 

Getting Started
---------------

This library runs on ev3dev2. Before continuing, make sure that you have set up
your EV3 as explained in the
`ev3dev Getting Started guide`. Make sure you have an ev3dev-stretch version
greater than ``2.2.0``. You can check the kernel version by selecting
"About" in Brickman and scrolling down to the "kernel version".

Usage
-----

To start out, you'll need a way to work with Python. We recommend the
`ev3dev Visual Studio Code extension`. 


The template for a Python script
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Every Python program should have a few basic parts. Use this template
to get started:

.. code-block:: python

    #!/usr/bin/env micropython
    from time import sleep
    from ev3dev2.sound import Sound
    from legoraptors import LineEdge, Direction, eprint, SimpleRobotDriveBase

    # TODO: Add code here

The first line should be included in every Python program you write
for ev3dev2. It allows you to run this program from Brickman, the graphical
menu that you see on the device screen. The other lines are import statements
which give you access to the library functionality. You will need to add
additional classes to the import list if you want to use other types of devices
or additional utilities.

You should use the ``.py`` extension for your file, e.g. ``my-file.py``.


Setting up a robot
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This code will create an instance of the SimpleRobotDriveBase called ``robot``. 
Adjust the ports to match your configuration. 

.. code-block:: python

    # Create and initialize an instance of SimpleRobotDriveBase called "robot" 
        robot = SimpleRobotDriveBase(
                leftDriveMotor = LargeMotor(OUTPUT_B), 
                rightDriveMotor  = LargeMotor(OUTPUT_A),
                tire, 
                axle_track = 126, 
                leftMotor = MediumMotor(OUTPUT_C), 
                rightMotor = MediumMotor(OUTPUT_D),
                leftColor = ColorSensor(INPUT_1),
                rightColor = ColorSensor(INPUT_2),
                centerColor = ColorSensor(INPUT_3),
                useGyro = False
                )

Once this is done, you can call the methods of ``robot`` like this:

``robot.driveArc(distance = 500, radius = 300, speed = 200,  direction = Direction.CW, brake = True, block = True)``

Moving the robot
~~~~~~~~~~~~~~~~~~~~~~

This will cause the robot to drive straight forward for 300 mm at 100 mm/s.

.. code-block:: python

  robot.driveStraight(distance = 300, speed = 100, brake = True, block = True)
  

Other functions in the library allow you to drive in a arc for a specified distance, 
follow lines, stop on lines, control the auxiliary motors, and use the gyro functions. 

For more information about the First Lego League, see the following links:

- http://www.firstlegoleague.org/
- https://legoraptors.com/

Library Documentation
---------------------

Class documentation for this library can be found on
the API reference. 

