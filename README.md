# proy_final_controlcont
An implementation of the flood-fill algorithm in a maze solving robot

## Arduino Hardware
- An Arduino Nano is used as the "brain" of the robot.
- Terminal block shield module to protect the microcontroller and facilitate replacement of this component in case of damage or malfunction.
- 4 wheels of 65mm radius and 26mm of width connected to one DC motor each.
- An L298N module permits the appropriate management of the 4 DC motors attached to the robot.
- 4 DC motors of 200RPM with a torque of 1 Kg/cm, current of 350 mA, and a voltage range between 4 and 9 volts.
- 3 rechargable batteries of 3.7v to power the L298N module that controls the 4 motors of the robot and a 9v battery as the voltage supply of the microcontroller of the robot.
- 3 ultrasonic sensors (HC-SR04) connected to the microcontroller in order to detect the distance between the robot and the walls of the maze that are in front, to the left side and to the right side of the robot.


## Arduino programming
By using C++, we are going to control the input received from the ultrasonic sensors and give a PWM output signal to the motors.

### Flood - Filling Algorithm
This algorithm provides us a way to determine the path the robot takes through the maze, by using a 2D grid representation of the maze. It works by expanding from a starting point / pixel of the maze to all adjacent pixels that have the same value as the seed pixel, modifying these values to a specific new value. It is important to point out that the maze cells which the robot can move to are initialized with a value that depends on the distance between that cell and the goal cell, while the cells that represent a place where it is possible to place a wall of the maze are set to 1 and modified accordingly via sensors readings.
## Feedback loop
By modifying the values of the 2D grid representation of the maze based on the sensor detections, the robot is able to decide the path which will take it to the goal faster. In fact, if we execute this program two times, once from the starting point to the final point and then make the robot go back to the start without using the path previously taken, we could create an efficient way to, not only solve a maze, but also examine which path is the shortest way to get to the goal in terms of cell shifting. 
