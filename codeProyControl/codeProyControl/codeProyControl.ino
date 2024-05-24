#define Pecho1 12
#define Ptrig1 13
#define Pecho2 10
#define Ptrig2 11
#define Pecho3 8
#define Ptrig3 9
#define Pecho4 6
#define Ptrig4 7   

// Maze size
const int ROWS = 6;
const int COLS = 4;
int maze[ROWS*2 - 1][COLS*2 - 1]; // 2D representation of the labyrinth

// unit per grid in cm
const int unit = 30;

// Distance between sensors and walls
const int distSensL = 10; // Left Sensor
const int distSensR = 10; // Right Sensor
const int distSensF = 10; // Front Sensor
int distSens[] = {distSensL, distSensR, distSensF};

// Start and Goal Positions
int initPos[] = {0,0};
int finalPos[] = {5,7};

// Initialization of readings from Ultrasonic Sensors
int ultrasonicR = 0; // Right Sensor
int ultrasonicL = 0; // Left Sensor
int ultrasonicF = 0; // Front Sensor
int ultrasonicReads[] = {ultrasonicL, ultrasonicR, ultrasonicF};

double MedirDist(int, int);
int x = 0;
double d, duracion, distancia;
int numSens = 3;  // Esto puede cambiar segun la asignacion de pines
int countSens = 0;
void setup() {
  // Initialize elements of the matrix to cero
  for (int i = 0; i < ; i++) {
    for (int j = 0; j < 3; j++) {
      maze[i][j] = 0;
    }
  }                
  Serial.begin(9600);        // inicializa el puerto seria a 9600 baudios
  pinMode(Pecho1, INPUT);     // define el pin 6 como entrada (echo)
  pinMode(Ptrig1, OUTPUT);    // define el pin 7 como salida  (triger)
  
  pinMode(Pecho2, INPUT);     
  pinMode(Ptrig2, OUTPUT);
  
  pinMode(Pecho3, INPUT);
  pinMode(Ptrig3, OUTPUT);
  }
  
void loop() {
  // Updating the wall data
  do{
    d = MedirDist(13-2*x,12-2*x);
    Serial.println("Distance = ");
    Serial.println(x);
    if (d <= distSens[x]){  // si la distancia es mayor a 120cm o menor a 0cm 
    	Serial.println("");                  // no mide nada
  	}
    else {
    	Serial.print(d);           // envia el valor de la distancia por el puerto serial
    	Serial.println("cm");              // le coloca a la distancia los centimetros "cm"                   
    }
    countSens += 1;
  } while (x<numSens);
  delay(100);
  countSens = 0;
}

double MedirDist(int sensorTrig, int sensorEcho)
{
  digitalWrite(sensorTrig, LOW);
  delayMicroseconds(4);
  digitalWrite(sensorTrig, HIGH);   // genera el pulso de triger por 10us
  delayMicroseconds(10);
  digitalWrite(sensorTrig, LOW);
  
  duracion = pulseIn(sensorEcho, HIGH);
  distancia = (duracion/2) / 29;   
  return distancia;
}
