#define PechoL 12
#define PtrigL 13
#define PechoR 10
#define PtrigR 11
#define PechoF 8
#define PtrigF 9

// Maze size
const int ROWS = 6;
const int COLS = 4;
int maze[ROWS*2 + 1][COLS*2 + 1]; // 2D representation of the labyrinth

// unit per grid in cm
const int unit = 30;

// Distance between sensors and walls
const int distSensL = 10; // Left Sensor
const int distSensR = 10; // Right Sensor
const int distSensF = 10; // Front Sensor
int distSens[] = {distSensL, distSensR, distSensF};

// Start and Goal Positions
int currPos[] = {0,0}; // Initializes at starting position
int movDone[] = {0,0}; // To know which move has been pulled off
int finalPos[] = {6,8}; // Final position of the robot

// Initialization of readings from Ultrasonic Sensors (4 different positions)
bool ultrasonicReads[] = {0,0,0,0}; // Left, Right, Front, Back

// To know the direction the robot is facing
int facingDir = 2; // North = 0, East = 1, South = 2, West = 3

// Times of rotation and translation
int movDelay = 2000;
int rotateDelay = 1500;

double duration, distance;
int numSens = 3;
int countSens = 0;

// Functions
double MeasureDist(int, int, int);
void movFW();
void rotRight();
void rotLeft();
void stopMov(int);

void setup() {
  // Initialize elements of the matrix to cero
  for (int i = 0; i < ROWS * 2 + 1; i+2) {
    for (int j = 0; j < COLS * 2 + 1; j+2) {
      maze[i][j] = 0;
    }
  }
  maze[finalPos[0]][finalPos[1]] = 1;                
  Serial.begin(9600);
  // Initialize the 2 pins of each ultrasonic sensor
  pinMode(PechoL, INPUT);
  pinMode(PtrigL, OUTPUT);    
  
  pinMode(PechoR, INPUT);     
  pinMode(PtrigR, OUTPUT);
  
  pinMode(PechoF, INPUT);
  pinMode(PtrigF, OUTPUT);

  // Initialize as output digital pins to control the 4 DC motors
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT); 
  }
  
void loop() {
  
  // Updating current position of the robot
  for (int updatePos = 0; updatePos < 2; updatePos++){
    currPos[updatePos] = currPos[updatePos] + movDone[updatePos];
  }
  do{
  // Reading the ultrasonic sensors one by one
    int posSens = (countSens + facingDir) % 4;
    ultrasonicReads[posSens] = MeasureDist(PtrigL-2*countSens,PechoL-2*countSens, distSens[countSens]);
    // Detecting walls
    if (ultrasonicReads[countSens]) {
       if(posSens == 0) {
        maze[currPos[0]][currPos[1]-1] = -1;
       } else if (posSens == 1) {
        maze[currPos[0]-1][currPos[1]] = -1;
       } else if (posSens == 2) {
        maze[currPos[0]][currPos[1]+1] = -1;
       } else if (posSens == 3) {
        maze[currPos[0]+1][currPos[1]] = -1;
       }
  	}
    countSens += 1;
  } while (countSens < numSens);
  int mazePos = {0,0,0,0};

  // Flooding current position
  maze[currPos[0]][currPos[1]] = maze[currPos[0]][currPos[1]] + 1;
  // Flooding adjacent cells
  if (currPos[0] > 1 || maze[currPos[0] - 1][currPos[1]] != -1) {
    maze[currPos[0] - 2][currPos[1]] = maze[currPos[0] - 2][currPos[1]] + 1;
  }
  if (currPos[1] > 1 || maze[currPos[0]][currPos[1] - 1] != -1) {
    maze[currPos[0]][currPos[1] - 2] = maze[currPos[0]][currPos[1] - 2] + 1;
  }
  if (currPos[0] < COLS * 2 || maze[currPos[0] + 1][currPos[1]] != -1) {
    maze[currPos[0] + 2][currPos[1]] = maze[currPos[0] + 2][currPos[1]] + 1;
  }
  if (currPos[1] < ROWS * 2 || maze[currPos[0]][currPos[1] + 1] != -1) {
    maze[currPos[0]][currPos[1] + 2] = maze[currPos[0]][currPos[1] + 2] + 1;
  }
  // Deciding on which path to take
  
  delay(100);
  countSens = 0;
  
  if(currPos == finalPos){
    while(true){
      Serial.println("Goal has been reached");
      delay(10000);
    }
  }
}

double MeasureDist(int sensorTrig, int sensorEcho, int distSens)
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
  digitalWrite(3,LOW); //IN1
  digitalWrite(2,HIGH); //IN2
  digitalWrite(5,LOW); //IN3
  digitalWrite(4,HIGH); //IN4
}
void rotRight()
{
  digitalWrite(3,LOW); //IN1
  digitalWrite(2,HIGH); //IN2
  digitalWrite(5,LOW); //IN3
  digitalWrite(4,HIGH); //IN4

}
void rotLeft()
{
  digitalWrite(3,LOW);
  digitalWrite(2,HIGH);
  digitalWrite(5,LOW);
  digitalWrite(4,LOW);
}
void stopMov(int timeBeforeStop)
{
  delay(timeBeforeStop);
  digitalWrite(3,LOW);
  digitalWrite(2,LOW);
  digitalWrite(5,LOW);
  digitalWrite(4,LOW);
}