#include <TinyGPSPlus.h>

/*This file is part of a code test for the serial communication function between
  two Arduinos. This is the code for the sender. We are testing for sending a string of text
  with the Latitude and Logitude from the GPS.
      
  Code uses Serial3, PINS 14(TX3) and 15(RX3).
*/

float Latitude, Longitude;

TinyGPSPlus gps;


void setup()
{
  Serial3.begin(115200); //Start communication with the other Arduino
  Serial1.begin(9600);
  Serial.begin(9600);
}



void loop(){
  sendgps();
}


void sendgps() {
  while (Serial1.available() > 0){
    if (gps.encode(Serial1.read())){
      if (gps.location.isValid())
      {
      Latitude = gps.location.lat();
      Longitude = gps.location.lng();
      } else {
      Latitude = 0;
      Longitude = 0;
      }
      serialWriteGPS(Latitude,Longitude);
      delay(100);
    }}
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

