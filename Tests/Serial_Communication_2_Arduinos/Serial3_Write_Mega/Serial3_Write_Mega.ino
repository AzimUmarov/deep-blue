/*This file is part of a code test for the serial communication function between
  an Arduino Mega and an Arduino Due. This is the code for the Arduino Mega which
  will be the sender. We are testing for two cases:
     (a) Sending a 1 or a 0
     (b) Sending a string of text
      
  Code uses Builtin LED and PINS 14(TX3) and 15(RX3). The two Arduinos are
  conected through a Bi-Directional Logic Level Converter.
*/

char mode = 'b'; //Variable to change between the two cases of the code
int state = 1; //Data to be sent in case (a)
char string[12] = "Hola Hubert!"; //String to be sent in case (b)

void setup() {
  Serial.begin(115200); //Start communication with the Arduino Due
  pinMode(LED_BUILTIN, OUTPUT); //Set LED on pin 13 as output
}

void loop() {

    // Case (a): Sends alternatively a 1 and a 0 through Serial3 and lights the LED accordingly
    if (mode=='a') {
        state = !state;
        delay(500);
        Serial.write(state);
        digitalWrite(LED_BUILTIN, state);
        delay(500);
    // Case (b): Send a 12-byte chain of text through Serial3
    } else if (mode=='b'){
        Serial.write(string,12);
    }
    delay(10000);
}
