/*This file is part of a code test for the serial communication function between
  an Arduino Mega and an Arduino Due. This is the code for the Arduino Due which
  will be the receiver. We are testing for two cases:
      a. Receiving a 1 or a 0
      b. Receiving a string of text
      
  Code uses Builtin LED and PINS 14(TX3) and 15(RX3). The two Arduinos are
  conected through a Bi-Directional Logic Level Converter.
*/

char mode = 'b'; //Variable to change between the two cases of the code
int state; //Data to be received in case (a)
char string[12]; //String to be received in case (b)

void setup() {
  Serial.begin(9600); //Start communication with the computer
  Serial3.begin(115200); //Start communication with the Arduino Mega
  pinMode(LED_BUILTIN, OUTPUT); //Set LED on pin 13 as output
}

void loop() {
  
    // Case (a): Receive a 1 or a 0 through Serial3 and light the LED accordingly
    if (mode=='a') {
        state = Serial3.read();
        digitalWrite(LED_BUILTIN, state);
    // Case (b): Receive a 12-byte chain of text through Serial3
    } else if (mode=='b'){
        Serial3.readBytes(string,12);
        Serial.println(string);
    }
}
