# proy_final_controlcont
An implementation of the flood-fill algorithm in a maze solving robot

## Arduino Hardware
- An Arduino Nano is used as the "brain" of the robot
- Ultrasonic sensors allows us to detect obstacles and determine the distance between the robot and them.
- An L298N module permits the appropriate management of the 4 DC motors attached to the robot.
- A 9v battery to power the L298N module and the motors and 2 rechargable batteries of 3.7v to power the microcontroller of the robot.


## Arduino programming
Using C++, we are going to control the input received from the ultrasonic sensors and give a PWM output signal to the motors.

### Flood - Filling Algorithm
This algorithm provides us a way to determine the path the robot takes through the maze, by using a 2D grid representation of the maze. It works by expanding from a starting point / pixel of the maze to all adjacent pixels that have the same value as the seed pixel, modifying these values to a specific new value.