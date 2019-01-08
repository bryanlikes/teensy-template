// #include "WProgram.h"
// #include <Encoder.h>

// int led = 13;

// Encoder motor1(5, 6);
// long positionMotor1  = -999;

// void blink() {
//   digitalWrite(led, LOW);    // turn the LED off
//   delay(100);               // wait
//   digitalWrite(led, HIGH);   // turn the LED on
//   delay(1);               // wait
// }

// void setup() {                
//   // initialize the digital pin as an output.
//   pinMode(led, OUTPUT);    
//   Serial.begin(9600); // USB is always 12 Mbit/sec 
// }

// // the loop routine runs over and over again forever:
// void loop() {
//   long newMotor1;
//   newMotor1 = motor1.read();
//   if (newMotor1 != positionMotor1) {
//     Serial.print("Motor1 = ");
//     Serial.print(newMotor1);
//     Serial.println();
//     positionMotor1 = newMotor1;
//     blink();
//   }
//   // if a character is sent from the serial monitor,
//   // reset both back to zero.
//   if (Serial.available()) {
//     Serial.read();
//     Serial.println("Reset encoders to zero");
//     motor1.write(0);
//   }
// }

