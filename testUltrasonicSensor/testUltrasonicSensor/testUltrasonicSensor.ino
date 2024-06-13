#define PechoL 12
#define PtrigL 13
#define PechoF 10
#define PtrigF 11
#define PechoR 8
#define PtrigR 9

double duration, distance;
bool MeasureDist(int, int, int);
int countSens = 0; int numSens = 3;
// Distance between sensors and walls
const int distSensL = 13; // Left Sensor
const int distSensF = 13; // Front Sensor
const int distSensR = 13; // Right Sensor
int distSens[] = {distSensL, distSensF, distSensR};

void setup() {
  Serial.begin(9600);
  // Initialize the 2 pins of each ultrasonic sensor
  pinMode(PechoL, INPUT);
  pinMode(PtrigL, OUTPUT);    
  pinMode(PechoR, INPUT);     
  pinMode(PtrigR, OUTPUT);
  pinMode(PechoF, INPUT);
  pinMode(PtrigF, OUTPUT);

}

void loop() {

  do {
    MeasureDist(PtrigL-2*countSens,PechoL-2*countSens, distSens[countSens]);
    countSens += 1;
    delay(100);
  } while(countSens < numSens);
  countSens = 0;
  delay(500);
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
  distance = (duration/2) / 29.1;
  Serial.print(sensorTrig);
  Serial.print(" Distance = ");
  Serial.println(distance);
  if (distance < distSens) {
    return true;
  } else{
    return false;
  }
}