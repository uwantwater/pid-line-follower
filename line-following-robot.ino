#include <AFMotor.h>
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);

#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

float Kp = 0.15;
float Ki = 0;
float Kd = 0.1;

float Pvalue;
float Ivalue;
float Dvalue;

uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 100;

void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A8, A9, A10, A11, A12}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  
  Serial.begin(9600);
  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are done with calibration

  // print the calibration minimum values measured when emitters were on
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i])/2;
    Serial.print(threshold[i]);
    Serial.print("  ");
  }
  Serial.println();

  delay(1000);
}

void loop()
{
  robot_control();
}

void robot_control(){
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 4000 (for a white line, use readLineWhite() instead)
  position = qtr.readLineBlack(sensorValues);
  // Serial.println(position);
  error = 2000 - position;
  Serial.println(error);
  while(sensorValues[0]>=90 && sensorValues[1]>=90 && sensorValues[2]>=90 && sensorValues[3]>=90 && sensorValues[4]>=90){ // A case when the line follower leaves the line
    position = qtr.readLineBlack(sensorValues);
    if(previousError>0){       //Turn left if the line was to the left before
      motor_drive(75, -75);
    }
    else {
      motor_drive(-75, 75); // Else turn right
    }
  }
  
  PID_Linefollow(error);
  // PID_Linefollow(error);
}
void PID_Linefollow(int error){
    P = error;
    I = I + error;
    D = error - previousError;
    
    Pvalue = (Kp)*P;
    Ivalue = (Ki)*I;
    Dvalue = (Kd)*D; 

    float PIDvalue = Pvalue + Ivalue + Dvalue;
    previousError = error;

    lsp = lfspeed + PIDvalue;
    rsp = lfspeed - PIDvalue;

    if (lsp > 100) {
      lsp = 100;
    }
    if (lsp < -100) {
      lsp = -100;
    }
    if (rsp > 100) {
      rsp = 100;
    }
    if (rsp < -100) {
      rsp = -100;                                                                                                                                                                                ;
    }
    motor_drive(lsp,rsp);
}

void motor_drive(int left, int right){
  
  if(right>0)
  {
    motor2.setSpeed(right);
    motor2.run(FORWARD);
  }
  else 
  {
    motor2.setSpeed(right);
    motor2.run(BACKWARD);
  }
  
 
  if(left>0)
  {
    motor1.setSpeed(left);
    motor1.run(FORWARD);
  }
  else 
  {
    motor1.setSpeed(left);
    motor1.run(BACKWARD);
  }
}