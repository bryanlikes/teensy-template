// #include "WProgram.h"

// #define BRAKEVCC 0
// #define CW   1
// #define CCW  2
// #define BRAKEGND 3
// #define CS_THRESHOLD 100

// void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm);
// void motorOff(int motor);

// /*  VNH2SP30 pin definitions
//  xxx[0] controls '1' outputs
//  xxx[1] controls '2' outputs */
// int inApin[2] = {11};  // INA: Clockwise input
// int inBpin[2] = {12}; // INB: Counter-clockwise input
// int pwmpin[2] = {10}; // PWM input
// bool motor1on = true;

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
// }

// void loop()
// {
//   if (motor1on) {
//     motorGo(0, CW, 1023);
//     //motorGo(1, CCW, 1023);
//     delay(500);

//     motorGo(0, CCW, 1023);
//     //motorGo(1, CW, 1023);
//     delay(500);
//   }
//     if (Serial.available()) {
//     Serial.read();
    
//     if (motor1on) {
//         motorOff(0);
//         motor1on = false;
//     }else{
//         motor1on = true;
//     }
//     Serial.print("toggle motor:");
//     Serial.println(motor1on);
//   }
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
 
//  pwm: should be a value between ? and 1023, higher the number, the faster
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
