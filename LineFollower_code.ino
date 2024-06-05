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
float motorSpeedChange;

int motorSpeedR;
int motorSpeedL;

// Speed/Direction init
int PWM1 = 4;
int DIR1 = 5;
int PWM2 = 3;
int DIR2 = 15;

//Settings for PID
float Kp = 0.1;
float Ki = 0;
float Kd = 2;
int desiredPosition = 2500;

//init for calibration 
int threshold = 120;
//bool flag_calib = true;
bool flag_stop = true;

//init state for behavior
int state = 1;

void setup() {

  // setting pin modes
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);

  // Configures the ADC object
  adc.begin(18, 19, 16, 17); // Initialize with SPI CS pin at digital pin 10

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // Configures the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){0, 1, 2, 3, 4, 5}, SensorCount);
  qtr.setEmitterPin(2);

  // calibrates the sensors when the robot turns on
  //and makes the robot move left and right so all the sensor can see the black line until calibration time finishes

  //start turnig left
  analogWrite(PWM1, 30);
  analogWrite(PWM2, 30);
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, HIGH);

  for (uint16_t i = 0; i < 400; i++) {
    if (flag_stop){
      // Check if the last sensor and the one before detects the black line
      //then turn Left
      if (adc.readADC(5) > threshold) {
        delay(70);
        // Turn left
        analogWrite(PWM1, 30);
        analogWrite(PWM2, 30);
        digitalWrite(DIR1, LOW);
        digitalWrite(DIR2, HIGH);
        Serial.println("Left");
      }
      // Check if the first sensor and the one after detects the black line
      //then turn Right
      else if (adc.readADC(0) > threshold){
        delay(50);
        // Turn Right
        analogWrite(PWM1, 30);
        analogWrite(PWM2, 30);
        digitalWrite(DIR1, HIGH);
        digitalWrite(DIR2, LOW);
        Serial.println("Right");
      }

      //when calibration time is about to stop and the robot is centered with the line stop
      //this will make the robot stop exactly centered with the line before start racing
      if ((i > 370) && ((adc.readADC(2) > threshold)||(adc.readADC(3) > threshold))){
            analogWrite(PWM1, 0);
            analogWrite(PWM2, 0);
            flag_stop=false;
      }

      // Calibrate the sensors
      qtr.calibrate(adc);
    }
  }
    
  // turn off Arduino's LED to indicate we are through with calibration
  digitalWrite(LED_BUILTIN, LOW); 

  //Give some time before starting so for the operator to check if everythink is fine
  delay(1500); 

  //for testing the robot (commented when not in use!)
  //Serial.begin(9600);

}


void loop()
{
//Runs for ever but stop if robot is at state 2
//state 2 = the four midle sensors see black
if (state != 2){
  // read calibrated sensor values and obtain a measure of the line position
  position = qtr.readLineBlack(sensorValues,adc);
  
  // Check if the 4 midle sensors detect black if even one sensor is not (bellow 500) then it breaks 
  state = 2;
  for (int i = 1; i < (SensorCount-1); i++) {
    if (sensorValues[i] < 500) {
      state = 1;
      break;
    }
  }

switch (state) {
  case 1:
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
    motorSpeedL = constrain(190 + motorSpeedChange, 0, 230);
    motorSpeedR = constrain(190 - motorSpeedChange, 0, 230);
    
    analogWrite(PWM1, abs(motorSpeedL));   //PWM Speed Control LEFT
    analogWrite(PWM2, abs(motorSpeedR));   //PWM Speed Control RIGHT

    //Direction forward
    digitalWrite(DIR1,HIGH);
    digitalWrite(DIR2,HIGH);
    break;
  case 2:
    //stops the robot
    analogWrite(PWM1, 140);
    analogWrite(PWM2, 140);
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, LOW);
    delay(330);
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    break;
  default:
    //stops the robot
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, LOW);
    break;
  }
}

// the following code helps with troubleshooting (commented when not in use!)
/*
  Serial.print("SpeedR: ");
  Serial.print(motorSpeedR);
  Serial.print(" SpeedL: ");
  Serial.print(motorSpeedL);
  Serial.print(" Change: ");
  Serial.print(motorSpeedChange);
  Serial.print(" position: ");
  Serial.print(position);
  Serial.print(" Case: ");
  Serial.println(var);
*/
}