/*This file is part of a code test for the serial communication function between two Arduinos.
  This is the code for the receiver. We are testing for sending a string of text
  with the status of the temperature and possible leaks and receiving a string of text, and turning it
  into the two float values for Latitude and Logitude of the GPS.
      
  Code uses Serial3, PINS 14(TX3) and 15(RX3). 
*/

float gpsLat=0;
float gpsLong=0;
bool tempExceeded=LOW;
bool leakDetected=HIGH;

void setup() {
  Serial.begin(9600); //Start communication with the computer
  Serial3.begin(115200); //Start communication with the other Arduino
}

void loop() {
     serialReadGPS();
     SerialWriteSafety(tempExceeded,leakDetected);
    //Minimum delay 200ms
    delay(200);
}

void serialReadGPS() {
        //Variable to store the string to be received
        char serialMessage[31];
        //Read message from the other Arduino
        Serial3.readBytes(serialMessage,34);
        //Turns the string into a float starting after "Lat:"
        int index=String(serialMessage).indexOf("Lat:")+4;
        gpsLat=String(serialMessage).substring(index).toFloat();
        //Does the same for "Long:"
        index=String(serialMessage).indexOf("Long:")+5;
        gpsLong=String(serialMessage).substring(index).toFloat();

        //For debugging
        Serial.println("-----Serial communication:-----");
        Serial.print("Message received:");
        Serial.println(serialMessage);
        Serial.print("gpsLat=");
        Serial.println(gpsLat,6);
        Serial.print("gpsLong=");
        Serial.println(gpsLong,6);
        Serial.println("-------------------------------");
        
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
