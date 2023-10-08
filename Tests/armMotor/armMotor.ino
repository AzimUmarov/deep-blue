#include <Servo.h>

//Servo
Servo fMotor, hMotor, vMotor1, vMotor2; //Create servo object to control the motors
int motorPin[]={2,3,4,5}; //Pins that will control the On/Off of the four motors


void setup(){
  fMotor.attach(8);
  hMotor.attach(9);
  vMotor1.attach(10);
  vMotor2.attach(11);
  
  //Set On/Off pins for motors as outputs
  for(int i=0;i<=sizeof(motorPin);i++){
  pinMode(i, OUTPUT);
  }
}

void loop() {
  armMotor(0,HIGH);
  delay(5000);

  /*For debugging
    //Going down
  for(int i=1500;i>600;i--) {
      fMotor.writeMicroseconds(i);
      delay(10);
      Serial.println(i);
  }
  delay(2000);

  //Going up
  for(int i=1500;i<2500;i++) {
      fMotor.writeMicroseconds(i);
      delay(10);
      Serial.println(i);
  }
  */
  
  armMotor(0,LOW);
  delay(5000);
}

void armMotor(int motorNum, bool state) {
    //If we want to arm the motor, first we send 1500 microseconds to the servo object
    if (state==HIGH){
        switch (motorNum) {
        case 0:
        digitalWrite(2, HIGH);
        fMotor.writeMicroseconds(1500);
        break;
        case 1:
        hMotor.writeMicroseconds(1500);
        break;
        case 2:
        vMotor1.writeMicroseconds(1500);
        case 3:
        vMotor2.writeMicroseconds(1500);
        break;
        }
    }
    //Then for both arm or disarm we will send the cosen state to the On/Off pin
    digitalWrite(motorPin[motorNum],state);
}
