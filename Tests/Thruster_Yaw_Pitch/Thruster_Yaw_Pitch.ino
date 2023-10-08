#include <Servo.h>

Servo rightmotor;  // create servo object to control a servo
Servo leftmotor;

void setup(){
  rightmotor.attach(11);  // attaches the servo on pin 9 to the servo object
  leftmotor.attach(9);
}

void yaw(bool direction){ //true == turn right, false == turn left
  if(direction == true)
    { 
      rightmotor.write(180);
      delay(2000);
      rightmotor.write(90);
    }
  if(direction == false)
    {
      leftmotor.write(180);
      delay(2000);
      leftmotor.write(90);
    }
}

void pitch(){
  
}



void loop(){
  yaw(true);
  delay(2000);
  yaw(false);
  delay(2000);
}
