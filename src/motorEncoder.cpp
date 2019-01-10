// #include "WProgram.h"
// #include <Encoder.h>

// #define BRAKEVCC 0
// #define CW   1
// #define CCW  2
// #define BRAKEGND 3
// #define CS_THRESHOLD 100

// void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm);
// void motorOff(int motor);
// void blink();

// /*  VNH2SP30 pin definitions
//  xxx[0] controls '1' outputs
//  xxx[1] controls '2' outputs */
// int inApin[2] = {11};  // INA: Clockwise input
// int inBpin[2] = {12}; // INB: Counter-clockwise input
// int pwmpin[2] = {10}; // PWM input

// Encoder motor1(5, 6);
// long positionMotor1  = 0;
// long speedMotor1 = 0;
// int speedDeltaMotor1 = 10;
// int led = 13;

// void setup()
// {
//   Serial.begin(9600);
  
//   // Initialize digital pins as outputs
//   for (int i=0; i<1; i++)
//   {
//     pinMode(inApin[i], OUTPUT);
//     pinMode(inBpin[i], OUTPUT);
//     pinMode(pwmpin[i], OUTPUT);
//   }
//   // Initialize braked
//   for (int i=0; i<1; i++)
//   {
//     digitalWrite(inApin[i], LOW);
//     digitalWrite(inBpin[i], LOW);
//   }

//   motor1.write(0);
// }

// void loop()
// {
//   motorGo(0, 1, 255);
//   delay(1000);
//   motorGo(0, BRAKEVCC, 255);
//   delay(1000);

//   // if (speedMotor1 > 240 || speedMotor1 < -240) {
//   //   speedDeltaMotor1 = speedDeltaMotor1*-1;
//   // }
  
//   // speedMotor1 += speedDeltaMotor1;
  
//   // int dir = CW;
//   // if (speedMotor1 < 0) {
//   //   dir = CCW;
//   // }


//   // motorGo(0, dir, abs(speedMotor1));
//   // delay(100);


//   // Serial.print(dir);
//   // Serial.print("...\t");
//   // Serial.print(abs(speedMotor1));
//   // Serial.print("...\t");
//   // Serial.print(speedMotor1);
//   // Serial.print("...\t");
  

//   // long newMotor1;
//   // newMotor1 = motor1.read();
//   // //if (positionMotor1 != newMotor1) {
//   //   Serial.print(newMotor1);
//   //   positionMotor1 = newMotor1;
//   // //}

//   // Serial.println();
// }

// void blink() {
//   digitalWrite(led, LOW);    // turn the LED off
//   delay(100);               // wait
//   digitalWrite(led, HIGH);   // turn the LED on
//   delay(1);               // wait
// }

// void motorOff(int motor)
// {
//   // Initialize braked
//   for (int i=0; i<2; i++)
//   {
//     digitalWrite(inApin[i], LOW);
//     digitalWrite(inBpin[i], LOW);
//   }
//   analogWrite(pwmpin[motor], 0);
// }

// /* motorGo() will set a motor going in a specific direction
//  the motor will continue going in that direction, at that speed
//  until told to do otherwise.
 
//  motor: this should be either 0 or 1, will selet which of the two
//  motors to be controlled
 
//  direct: Should be between 0 and 3, with the following result
//  0: Brake to VCC
//  1: Clockwise
//  2: CounterClockwise
//  3: Brake to GND
 
//  pwm: should be a value between ? and 256, higher the number, the faster
//  it'll go
//  */
// void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
// {
//   if (motor <= 1)
//   {
//     if (direct <=4)
//     {
//       // Set inA[motor]
//       if (direct <=1)
//         digitalWrite(inApin[motor], HIGH);
//       else
//         digitalWrite(inApin[motor], LOW);

//       // Set inB[motor]
//       if ((direct==0)||(direct==2))
//         digitalWrite(inBpin[motor], HIGH);
//       else
//         digitalWrite(inBpin[motor], LOW);

//       analogWrite(pwmpin[motor], pwm);
//     }
//   }
// }
