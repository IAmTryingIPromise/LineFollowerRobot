#include <QTRSensors.h>
#include <Adafruit_MCP3008.h>

// Create an instance of the ADC object
Adafruit_MCP3008 adc; 

//Initialise the sensors
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

//QTRSensors object with ADC as argument for the constructor
QTRSensors qtr(adc);

// variable declaration
int position;
int error;
int lastError;
int P;
int I;
int D;
int motorSpeedChange;

int motorSpeedR;
int motorSpeedL;

// ?????
int PWM1 = 4;
int DIR1 = 5;
int PWM2 = 3;
int DIR3 = 15;

//Settings for PID
float Kp = 0.1;
float Ki = 0;
float Kd = 0.2;
int desiredPosition = 2500;


void setup() {

  // setting pin modes
  pinMode(DIR1, OUTPUT);
  pinMode(DIR3, OUTPUT);

  // Configures the ADC object
  adc.begin(18, 19, 16, 17); // Initialize with SPI CS pin at digital pin 10

  // Configures the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){0, 1, 2, 3, 4, 5}, SensorCount);
  qtr.setEmitterPin(2);

  // calibrates the sensors when the robot turns on
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate(adc);
  }

  Serial.begin(9600);

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

}


void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  position = qtr.readLineBlack(sensorValues,adc);

  // calculating the error
  error = desiredPosition - position ;

  // setting the PID variable values
  P = error;
  I = I + error;
  D = error - lastError ;
  lastError = error;

  // calculating the PID
  motorSpeedChange = P * Kp + I * Ki + D * Kd;

  // setting the speed values for the wheels
  motorSpeedL = constrain(210 + motorSpeedChange, 0, 230);
  motorSpeedR = constrain(210 - motorSpeedChange, 0, 230);

  // ?????
  digitalWrite(DIR1,HIGH);
  digitalWrite(DIR3,HIGH);


  analogWrite(PWM1, abs(motorSpeedL));   //PWM Speed Control LEFT
  analogWrite(PWM2, abs(motorSpeedR));   //PWM Speed Control RIGHT



// the following code helps with troubleshooting
/*

  Serial.print("SpeedR: ");
  Serial.print(motorSpeedR);
  Serial.print(" SpeedL: ");
  Serial.print(motorSpeedL);
  Serial.print(" Change: ");
  Serial.print(motorSpeedChange);
  Serial.print(" position: ");
  Serial.println(position);
*/

}
