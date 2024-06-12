/*

  Continuous Control Systems Project - Solving a maze via flooding algorithm implementation
  Edited by: Juan Diego Cabrera Moncada
  Robot: Cuchau
  Institution: Universidad de Antioquia
  Department: Electronics and Telecommunication Engineering Department
  Last Update: 07/06/2024
  GitHub Repository: https://github.com/GenCabMon/proy_final_controlcont.git
  Tinkercad Simulation: https://www.tinkercad.com/things/aU9Vbg2W2rE-copy-of-proycontrolcont/editel?sharecode=oTeLaLMKO4r3vKxnA6kjj40KziEj-_Xl-RLQpkTR7c4

*/

#define PechoL 12
#define PtrigL 13
#define PechoF 10
#define PtrigF 11
#define PechoR 8
#define PtrigR 9

#define ENA 5 // Cable negro
#define ENB 3 // Cable verde
#define IN1 4
#define IN2 2
#define IN3 7
#define IN4 6

#include <LMotorController.h>

// To manage the PWM of the L298N
double motorSpeedFactorLeft = 1;
double motorSpeedFactorRight = 1;
int MIN_ABS_SPEED = 100;
int pwm = 90;
LMotorController motorController(ENA, IN1, IN2, ENB, IN4, IN3, motorSpeedFactorLeft, motorSpeedFactorRight);

// Maze size
const int ROWS = 6;
const int COLS = 4;
int maze[ROWS*2 + 1][COLS*2 + 1]; // 2D representation of the labyrinth

// Distance between sensors and walls
const int distSensL = 20; // Left Sensor
const int distSensF = 20; // Front Sensor
const int distSensR = 20; // Right Sensor
int distSens[] = {distSensL, distSensF, distSensR};

// Start and Goal Positions
int currPos[] = {11,7}; // Initializes at starting position
int finalPos[] = {1,1}; // Final position of the car

// Initialization of readings from Ultrasonic Sensors (4 different positions)
bool ultrasonicReads[] = {0,0,0,0}; // Left, Right, Front, Back

// To know the direction the car is facing
int facingDir = 0; // North = 0, East = 1, South = 2, West = 3

double duration, distance;
int numSens = 3;
int countSens = 0;
int countWalls = 0;
int lastBifurcation[] = {1,1}; //Initialized at the same value of the final position

// Set to True if there is a wall behind the car in the starting position
bool isAWallBehind = true;

bool isBifurcationFound = true; //Used to avoid dead end path loops generation

// Functions
bool MeasureDist(int, int, int);
void movFW();
void rotRight();
void rotLeft();
void stopMov(int);
int findMinValue(int arr[], int);
void initializeMaze();
void ultrasonicSensorsReading();
void floodFilling(int* mazePos);
void robotShifting(int);
void avoidPathLooping();

void setup() {
  Serial.begin(9600); //Booting up serial monitor for testing

  initializeMaze();
  // Initialize the 2 pins of each ultrasonic sensor
  pinMode(PechoL, INPUT);
  pinMode(PtrigL, OUTPUT);    
  pinMode(PechoR, INPUT);     
  pinMode(PtrigR, OUTPUT);
  pinMode(PechoF, INPUT);
  pinMode(PtrigF, OUTPUT);

  // Initialize as output digital pins to control the 4 DC motors
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);

  // Initialize pins for PWM control
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  }
  
void loop() {
  
  ultrasonicSensorsReading();
  int mazePos[] = {-1,-1,-1,-1};
  floodFilling(mazePos);
  avoidPathLooping(); //Trying to avoid dead end path loops generation

  // Deciding on which path to take
  int minValPos = findMinValue(mazePos, 4);
  delay(200);
  robotShifting(minValPos);
  facingDir = minValPos; // Updating the current direction the car is facing
  countSens = 0;
  // Checking if the car is currently at the final position
  Serial.println(currPos[0]);
  Serial.println(currPos[1]);
  if(currPos[0] == finalPos[0] && currPos[1] == finalPos[1]){
    while(true){
      Serial.println("Goal has been reached");
      delay(500);
    }
  }
}

bool MeasureDist(int sensorTrig, int sensorEcho, int distSens)
{
  /* Needs the ping assignment of the ultrasonic sensor and the distance expected
  between the sensor and a wall. Returns true if it detects a wall, false otherwise*/
  digitalWrite(sensorTrig, LOW);
  delayMicroseconds(4);
  digitalWrite(sensorTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorTrig, LOW);
  
  duration = pulseIn(sensorEcho, HIGH);
  distance = (duration/2) / 29;
  Serial.print("Distance = ");
  Serial.println(distance);
  if (distance < distSens) {
    return true;
  } else{
    return false;
  }
}

void movFW()
{
  // Uses the L298N inputs to make the car move forward
  Serial.println("movFW");
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
  digitalWrite(5,LOW); //IN1
  digitalWrite(4,HIGH); //IN2
  digitalWrite(7,LOW); //IN3
  digitalWrite(6,HIGH); //IN4 
}
void rotRight()
{
  /* Uses the L298N inputs to make the car rotate 
  on its own axis in a clockwise direction.
  */
  Serial.println("rotRight");
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}
void rotLeft()
{
  /* Uses the L298N inputs to make the car rotate 
  on its own axis in a counterclockwise direction.
  */
  Serial.println("rotLeft");
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}
void stopMov(int timeBeforeStop)
{
  /* Uses the L298N inputs to stop the car's current
  	 moving action after a delay received as a parameter.
  */
  Serial.println("stopMov");
  Serial.println(timeBeforeStop);
  delay(timeBeforeStop);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}
int findMinValue(int arr[], int size) {
  /* Receives a list of values and its size. Returns the position
  	 of the minimum nonnegative value of that list.
  */
  int minValue = 31200;
  // If there is no wall, assume first element as the minimum value
  if (arr[0] >= 0) {
    minValue = arr[0];
  }
  int pos = 0;
  // Updating the minimum value
  for (int i = 1; i < size; i++) {
  	if (arr[i] < minValue && arr[i] >= 0) {
    	minValue = arr[i];
        pos = i;
    }
  }
  return pos;
}
void initializeMaze() {
  // Initialize elements of the matrix: Walls and Cells
  for (int i = 0; i < ROWS * 2 + 1; i++) {
    for (int j = 0; j < COLS * 2 + 1; j++) {
      if (i % 2 == 0 || j % 2 == 0) {
        maze[i][j] = 1; // Walls
      } else {
        //maze[i][j] = 0; //Empty cells
        maze[i][j] = i - finalPos[0] + j - finalPos[1]; // Empty cells with a value dependent on its distance to the goal cell 
      }
    }
  }
  
  /* Additional configuration to know if the car is placed
  	 with a wall behind it.
  */
  if (isAWallBehind) {
    if(facingDir == 0){
      maze[currPos[0] + 1][currPos[1]] = -1;
    } else if (facingDir == 1) {
      maze[currPos[0]][currPos[1] - 1] = -1;      
    } else if (facingDir == 2) {
      maze[currPos[0] - 1][currPos[1]] = -1;      
    } else {
      maze[currPos[0]][currPos[1] + 1] = -1;      
    }
  }
}
void ultrasonicSensorsReading() {
  do{
  // Reading the ultrasonic sensors one by one
    int posSens = (countSens + facingDir) % 4;
    ultrasonicReads[posSens] = MeasureDist(PtrigL-2*countSens,PechoL-2*countSens, distSens[countSens]);
    // Detecting walls
    if (ultrasonicReads[posSens]) {
       if(posSens == 0) {
        maze[currPos[0]][currPos[1]-1] = -1;
       } else if (posSens == 1) {
        maze[currPos[0]-1][currPos[1]] = -1;
       } else if (posSens == 2) {
        maze[currPos[0]][currPos[1]+1] = -1;
       } else if (posSens == 3) {
        maze[currPos[0]+1][currPos[1]] = -1;
       }
       countWalls += 1;
  	}
    countSens += 1;
    delay(70); // A delay between readings to secure good measurements
  } while (countSens < numSens);
}
void floodFilling(int* mazePos){
  // Flooding current position
  maze[currPos[0]][currPos[1]] = maze[currPos[0]][currPos[1]] + 1;
  // Flooding adjacent cells
  if (currPos[0] > 1 && maze[currPos[0] - 1][currPos[1]] != -1) {
    maze[currPos[0] - 2][currPos[1]] = maze[currPos[0] - 2][currPos[1]] + 1;
    mazePos[0] = maze[currPos[0] - 2][currPos[1]];
  } // Upper Row
  if (currPos[1] < COLS * 2 && maze[currPos[0]][currPos[1] + 1] != -1) { 			
    maze[currPos[0]][currPos[1] + 2] = maze[currPos[0]][currPos[1] + 2] + 1;
  	mazePos[1] = maze[currPos[0]][currPos[1] + 2];
  } // Right Column
  if (currPos[0] < ROWS * 2 && maze[currPos[0] + 1][currPos[1]] != -1) { 	
    maze[currPos[0] + 2][currPos[1]] = maze[currPos[0] + 2][currPos[1]] + 1;
  	mazePos[2] = maze[currPos[0] + 2][currPos[1]];
  } // Lower Row
  if (currPos[1] > 1 && maze[currPos[0]][currPos[1] - 1] != -1) {
    maze[currPos[0]][currPos[1] - 2] = maze[currPos[0]][currPos[1] - 2] + 1; 
  	mazePos[3] = maze[currPos[0]][currPos[1] - 2];
  } // Left Column
}
void robotShifting(int minValPos) {
  // Define the time it takes to make each movement of the car
  int timeRotLeft = 1000;
  int timeRotRight = 1200;
  int timeMovFW = 1000;
  /* Rotation of the car:
  		Depends on the minimum value between the adjacent
        available cells the car can go to, and the direction
        the car is currently pointing at.
  */
  if (minValPos == 0) {
    if(facingDir == 1) {
      rotLeft();
      stopMov(timeRotLeft);
    } else if (facingDir == 2) {
      rotRight();
      stopMov(2*timeRotRight);
    } else if (facingDir == 3) {
      rotRight();
      stopMov(timeRotRight);
    }
    currPos[0] = currPos[0] - 2;
  } else if (minValPos == 1) {
    if (facingDir == 0) {
      rotRight();
      stopMov(timeRotRight);
    } else if (facingDir == 2) {
      rotLeft();
      stopMov(timeRotLeft);
    } else if (facingDir == 3) {
      rotRight();
      stopMov(2*timeRotRight);
    }
    currPos[1] = currPos[1] + 2;
  } else if (minValPos == 2) {
    if (facingDir == 0) {
      rotLeft();
      stopMov(2*timeRotRight);
    } else if(facingDir == 1) {
      rotRight();
      stopMov(timeRotRight);
    } else if (facingDir == 3) {
      rotLeft();
      stopMov(timeRotLeft);
    }
    currPos[0] = currPos[0] + 2;
  } else if (minValPos == 3) {
    if (facingDir == 0) {
      rotLeft();
      stopMov(timeRotLeft);
    } else if(facingDir == 1) {
      rotRight();
      stopMov(2*timeRotRight);
    } else if (facingDir == 2) {
      rotRight();
      stopMov(timeRotRight);
    }
    currPos[1] = currPos[1] - 2;
  }
    // Movement of the car
  movFW();
  motorController.move(pwm, MIN_ABS_SPEED); // Managing the motor speed
  stopMov(timeMovFW);
  motorController.move(pwm, MIN_ABS_SPEED);
}
void avoidPathLooping(){
    //Avoiding dead end path loops
  if(lastBifurcation[0] == currPos[0] && lastBifurcation[1] == currPos[1]) { //Robot is back to the bifurcation location
    isBifurcationFound = true;
  } else if (isBifurcationFound == false) { // Executed while getting out of the dead end path
    maze[currPos[0]][currPos[1]] += abs(maze[currPos[0]][currPos[1]] - maze[lastBifurcation[0]][lastBifurcation[1]]);
  }
  if (countWalls == 1) { // Bifurcation encountered
    lastBifurcation[0] = currPos[0]; lastBifurcation[1] = currPos[1];
  } else if(countWalls == 3) { // Dead end found
    isBifurcationFound = false;
    maze[currPos[0]][currPos[1]] += abs(maze[currPos[0]][currPos[1]] - maze[lastBifurcation[0]][lastBifurcation[1]]);
  }
}