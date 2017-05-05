#include <Servo.h> //define the servo library
Servo ssm; //define that ssm (steering servo motor) is a servo
Servo esc; //define that esc is a servo

void setup() {
ssm.attach(5); //define that ssm is connected at pin 12
esc.attach(6); //define that esc is connected at pin 13

//*********************Vehicle Calibration*********************//
esc.write(90);  delay(5000); 
esc.write(180); delay(5000); 
esc.write(0); delay(5000); 
esc.write(90); delay(5000); 
}

void loop() {}
