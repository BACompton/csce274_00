# CSCE 274 Resources
This is a central repository with helpful interfaces for CSCE274. Each resource is written in python
with the iRobot Create 2 in mind. These resources were created to increase the readibility of a projects
code and simplify its implementation by automatically handling complicated situations.  

These interfaces are meant to be the base for future projects. Projects using any of the resources within
this repository should build on these resources. Augmentations to this repository's resources will be
consider if the change is useful in a number of different situations. However, any necessary augmentations 
that apply to a specific project should be done within that projects copy of the resources.

# Resources Dependencies
In general, check the import statemnt at the top of each resource's source code for the list of python modules
used by that resource.

However here is a quick description and list of dependencies for each resource:
* __main.py__
  * _Description:_ This file will sent ASCII based commands while reading any raw data from serial connection.
  * _Purpose:_ Can be used to test the ability to send commands and read data on a serial connection.
  * _Dependencies:_ 
    * serial_inf.py
* __robot_emulator.py__
  * _Description:_ This is a basic robot emulator. The required behavior of a robot will need to be altered
  for each project.
  * _Purpose:_ Can be used to test an algorithm on a higher level whenever you dont have access to the robot.
  * _Dependencies:_ 
    * serial_inf.py
    * robot_inf.py
