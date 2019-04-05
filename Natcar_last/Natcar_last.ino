#include <QTRSensors.h>


#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2
const int pinPWMA = 6;
const int pinAIN2 = 4;
const int pinAIN1 = 5;
const int pinBIN1 = 8;
const int pinBIN2 = 3;
const int pinPWMB = 9;
//const int pinSTBY = 12;
float MD=0;
float MI=0;
float dArray = 0;
float diff = 0;
float kp = 0.03;
float ki = 0.00;
float kv = 1;
float P = 0;
float I = 0;
float Ia = 0;
float dt = 0;
float vcm = 100;
double t=0;
// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]) {7, 6, 5, 4, 3, 2, 1, 0},
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];


void setup()
{
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  // motores
  
}


void loop()
{
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtra.read(sensorValues); instead of unsigned int position = qtra.readLine(sensorValues);
  int position = qtra.readLine(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
  //  Serial.print(sensorValues[i]);
  //  Serial.print('\t');
  }
  //Serial.println(); // uncomment this line if you are using raw values
 // Serial.println(position); // comment this line out if you are using raw values
  dArray = (position- 3500)*1 ; 
  dt=micros()/1000000.0-t;
  I = Ia+ki*dArray*dt ;
  t=micros()/1000000.0;
  P = kp*dArray ;
  diff = P + I;
  MI = (2*vcm-diff)/2  ;
  MD = (2*vcm+diff)/2  ;
  kv= (1400000/(3*abs(dArray)+7000))/vcm;  
  MI=kv*MI;
  MD=kv*MD;
  MI= max(min(MI,255),-255);
  MD= max(min(MD,255),-255);
  if (kv == 0.8){
    if (MD<MI) MD=0;
    else MI=0; 
    }
  
  motores(MD,MI);
  delay(10);
    Serial.print(MI);
    Serial.print('\t');
    Serial.print(MD);
    Serial.print('\t');
    Serial.print(dArray);
    Serial.print('\t');
    Serial.println(kv);
    Serial.print('\t');
    Serial.print(dt);
    Serial.print('\t');
    Serial.print(P);
    Serial.print('\t');
    Serial.print(I);
    Serial.print('\t');
    Serial.print(diff);
    Serial.print('\t');
Ia = I ;  
}

void motores(int izq, int der)
{
  if(izq>=0)
  {
    digitalWrite(pinAIN1,HIGH);
    digitalWrite(pinAIN2,LOW);
    analogWrite(pinPWMA,izq);
  }
  if(izq<0)
  {
    digitalWrite(pinAIN1,LOW);
    digitalWrite(pinAIN2,HIGH);
    //izq=izq*-1;
    analogWrite(pinPWMA,abs(izq));
  }

  if(der>=0)
  {
    digitalWrite(pinBIN1,HIGH);
    digitalWrite(pinBIN2,LOW);
    analogWrite(pinPWMB,der);
  }
  if(der<0)
  {
    digitalWrite(pinBIN1,LOW);
    digitalWrite(pinBIN2,HIGH);
    //der=der*-1;
    analogWrite(pinPWMB,abs(der));
  }
}
