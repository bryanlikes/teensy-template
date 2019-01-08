#include "WProgram.h"
#include <Encoder.h>
#include <PID_v1.h>

#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm);
void motorOff(int motor);
void blink();

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=10, Ki=1, Kd=0.015;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
int inApin[2] = {11};  // INA: Clockwise input
int inBpin[2] = {12}; // INB: Counter-clockwise input
int pwmpin[2] = {10}; // PWM input

Encoder motor1(5, 6);
long positionMotor1  = 0;
long speedMotor1 = 0;
int speedDeltaMotor1 = 10;
int led = 13;
int dir = CW;
char tuning[20];

void setup()
{
  Serial.begin(115200);
  
  // Initialize digital pins as outputs
  for (int i=0; i<1; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i=0; i<1; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }

  motor1.write(0);

  Setpoint = 1;
  Input = motor1.read();
  //turn the PID on
  myPID.SetOutputLimits(-255, 255);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  
}

void loop()
{
  if (Serial.available()) {
    for( int i = 0; i < sizeof(tuning);  ++i )
      tuning[i] = (char)0;

    int size = Serial.readBytes(tuning, 20);
    if (size == 20) {

    String pi = String(tuning);
    double kp = atof(pi.substring(0,6).c_str());
    double ki = atof(pi.substring(7,13).c_str());
    double kd = atof(pi.substring(14,20).c_str());
    myPID.SetTunings(kp, ki, kd);

    Serial.print(myPID.GetKp(), 3);
    Serial.print(",");
    Serial.print(myPID.GetKi(), 3);
    Serial.print(",");
    Serial.println(myPID.GetKd(), 3);
    Serial.println();
    }
  }

  double lastInput = Input;
  double lastOutput = Output;
  

  Input = motor1.read();
  myPID.Compute();

  // if (Input < 0 && myPID.GetDirection() == DIRECT) {
  //   myPID.SetControllerDirection(REVERSE);
  // } else if (myPID.GetDirection() == REVERSE){
  //   myPID.SetControllerDirection(DIRECT);
  // }

  dir = CW;
  if (Output < 0) {
    dir = CCW;
  }
   
  int speed = abs(Output);
  motorGo(0, dir, speed);


  if (lastInput != Input || lastOutput != Output) {
    Serial.print(Input);
    Serial.print("\t\t");
    Serial.print(Output);
    Serial.print("\t\t");
    Serial.print(Setpoint);
    Serial.print("\t\t");
    Serial.print(myPID.GetDirection());

    Serial.println();
  }
}

void blink() {
  digitalWrite(led, LOW);    // turn the LED off
  delay(100);               // wait
  digitalWrite(led, HIGH);   // turn the LED on
  delay(1);               // wait
}

void motorOff(int motor)
{
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 
 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled
 
 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND
 
 pwm: should be a value between ? and 256, higher the number, the faster
 it'll go
 */
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}
