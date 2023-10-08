/*This file is part of a code test for the serial communication function between
  two Arduinos. This is the code for the sender. We are testing for sending a string of text
  with the Latitude and Logitude from the GPS.
      
  Code uses Serial3, PINS 14(TX3) and 15(RX3).
*/

//For debugging
float Latitude=-90.123456;
float Longitude=-180.123456;

void setup()
{
  Serial3.begin(115200); //Start communication with the other Arduino
}

void loop() {
    serialWriteGPS(Latitude,Longitude);
    //Minimum delay 200ms
    delay(200);
    
    //For debugging
    if (Latitude>90.5){Latitude=-90.123456;}
    else {Latitude+=10;}
}

//Send GPS coordinates through Serial3
void serialWriteGPS(float gpsLat, float gpsLong) {
  //Create variables to store the numbers as strings
  char gpsLatString[11];
  char gpsLongString[12];
  //Convert the numbers to strings
  dtostrf(gpsLat,0,6,gpsLatString);
  dtostrf(gpsLong,0,6,gpsLongString);
  //Create variable to store the message
  char serialMessage[sizeof(gpsLatString)+sizeof(gpsLongString)+11];
  //Load the message
  sprintf(serialMessage, "Lat:%s//Long:%s", gpsLatString, gpsLongString);
  //Send the message
  Serial3.write(serialMessage,sizeof(serialMessage));
}
