/*This file is part of a code test for the serial communication function between
  two Arduinos. This is the code for the sender. We are testing for sending a string of text
  with the status of the temperature and possible leaks.
      
  Code uses Serial3, PINS 14(TX3) and 15(RX3).
*/

bool tempExceeded=LOW;
bool leakDetected=HIGH;

void setup()
{
  Serial3.begin(115200); //Start communication with the other Arduino
}

void loop() {
    SerialWriteSafety(tempExceeded,leakDetected);
    //Minimum delay 200ms
    delay(200);
}

//Send Safety information through Serial3
void SerialWriteSafety(bool temp, bool leak) {
  //Create variable to store the message
  char serialMessage[14];
  //Load the message
  sprintf(serialMessage, "Temp:%d//Leak:%d", temp, leak);
  //Send the message
  Serial3.write(serialMessage,sizeof(serialMessage));
}
