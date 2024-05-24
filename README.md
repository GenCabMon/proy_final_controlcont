# proy_final_controlcont
An implementation of the flood-fill algorithm in a maze solving robot

## Arduino Hardware
- An Arduino Nano is used as the "brain" of the robot
- An L298N module permits the appropriate management of the 4 DC motors attached to the robot.
- A 9v battery to power the L298N module and the motors and 2 rechargable batteries of 3.7v as the voltage supply of the microcontroller of the robot.
- 3 ultrasonic sensors connected to the microcontroller in order to detect the distance between the robot and the walls of the labyrinth that are in front, to the left side and to the right side of the robot.


## Arduino programming
By using C++, we are going to control the input received from the ultrasonic sensors and give a PWM output signal to the motors.

### Flood - Filling Algorithm
This algorithm provides us a way to determine the path the robot takes through the maze, by using a 2D grid representation of the maze. It works by expanding from a starting point / pixel of the maze to all adjacent pixels that have the same value as the seed pixel, modifying these values to a specific new value.