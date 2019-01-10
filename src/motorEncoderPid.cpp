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
int Setpoint, Input;
float Output;

//Specify the links and initial tuning parameters
// double aggKp=8, aggKi=16, aggKd=0.35;
//these work ok at 55 speed: double aggKp=25, aggKi=16, aggKd=1.3;
int aggKp=100000, aggKi=10000, aggKd=31000;
PID myPID(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, REVERSE);

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
long change = 0;

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
  
  motorOff(0);
  motor1.write(0);

  Setpoint = 0;
  Input = motor1.read();
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  change = 0;
}

void loop()
{

  //double lastInput = Input;
  //double lastOutput = Output;

  // Read encoder and compute output
  Input = motor1.read();
  myPID.Compute();

  // Adjust deadzone for specific motor
  int speed = abs(Output);
  if (speed > 65535) speed = 65535;
  speed = speed / 65535.0*255.0;

  // Set direction and run motor
  dir = CW;
  if (Output < 0) {
    dir = CCW;
  }
  motorGo(0, dir, speed);

  // // Debug serial output
  // if (lastInput != Input || lastOutput != Output) {
  //   Serial.print("input:");
  //   Serial.print(Input);
  //   Serial.print("   \toutput:");
  //   Serial.print(Output);
  //   Serial.print("   \tspeed:");
  //   Serial.print(speed);
  //   Serial.print("   \tsetpoint:");
  //   Serial.print(Setpoint);
  //   Serial.println();
  // }


  // //Tuning Initial
  // change++;
  // if (change == 40000) {
  //   Setpoint = 500;
  // }
  // if (change < 80000) {
  //   if (change % 100 == 0) {
  //     String str = String(change);
  //     str.append(",");
  //     str.append(String(Input).c_str());
  //     str.append(",");
  //     str.append(String(Setpoint).c_str());
  //     Serial.println(str.c_str());
  //   }
  // } else if (change == 80001) {
  //   Serial.print("\n\n\n\n");
  // }

  // //Tuning round 2
  // change++;
  // if (change == 40000) {
  //   Setpoint = 500;
  // } else if (change == 80000) {
  //   Setpoint = 5000;
  // } else if (change == 80000) {
  //   Setpoint = 5000;
  // } else if (change == 100000) {
  //   Setpoint = 0;
  // }
  // if (change < 130000) {
  //   if (change % 100 == 0) {
  //     String str = String(change);
  //     str.append(",");
  //     str.append(String(Input).c_str());
  //     str.append(",");
  //     str.append(String(Setpoint).c_str());
  //     Serial.println(str.c_str());
  //   }
  // } else if (change == 80001) {
  //   Serial.print("\n\n\n\n");
  // }













  //JuNK

  //   change++;
  //   if (change > 100000) {
  //     change =0;
  //   //if (Serial.available()) {
  //   // for( int i = 0; i < sizeof(tuning);  ++i )
  //   //   tuning[i] = (char)0;

  //   // int size = Serial.readBytes(tuning, 20);
  //   // if (size == 20) {

  //   // // Brake motor
  //   // motorGo(0, BRAKEVCC, 0);
  //   // delay(500);

  //   // // Parse new tuning
  //   // String pi = String(tuning);
  //   // double kp = atof(pi.substring(0,6).c_str());
  //   // double ki = atof(pi.substring(7,13).c_str());
  //   // double kd = atof(pi.substring(14,20).c_str());

  //   // // Apply tune and output values to serial
  //   // myPID.SetTunings(kp, ki, kd);
  //   // Serial.print(myPID.GetKp(), 3);
  //   // Serial.print(",");
  //   // Serial.print(myPID.GetKi(), 3);
  //   // Serial.print(",");
  //   // Serial.println(myPID.GetKd(), 3);
  //   // Serial.println();
  //   // }

  //   // int size = Serial.readBytes(tuning, 20);
  //   // String pi = String(tuning);
  //   // double setpoint = atof(pi.c_str());
  //   if (Setpoint == 0) {
  //     Setpoint = 800;
  //   } else {
  //     Setpoint = 0;
  //   }
  // }
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
