/*This file is part of a code test for the serial communication function between
  two Arduinos. This is the code for the sender. We are testing for sending a string of text
  with the Latitude and Logitude from the GPS.
      
  Code uses Serial3, PINS 14(TX3) and 15(RX3).

  This one includes TinyGPSPlus
*/


#include <TinyGPSPlus.h>

TinyGPSPlus gps;

float Latitude;
float Longitude;

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial3.begin(115200); //Start communication with the other Arduino

}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial1.available() > 0)
    if (gps.encode(Serial1.read()))
      displayInfo();
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Latitude=gps.location.lat();
    Serial.print("(");
    Serial.print(latitud,6);
    Serial.print(")");
    Serial.print(gps.location.lng(), 6);
    Longitude=gps.location.lng();
    Serial.print("(");
    Serial.print(longitud,6);
    Serial.print(")");
    
    Serial.println();
  }
  else
  {
    Serial.print(F("INVALID"));
  }
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
