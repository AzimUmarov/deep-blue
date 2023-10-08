/*This file is part of a code test for the serial communication function between
  two Arduinos. This is the code for the sender. We are testing for sending a string of text
  with the Latitude and Logitude from the GPS, and receiving a string with with the status of
  the temperature and possible leaks.
      
  Code uses Serial3, PINS 14(TX3) and 15(RX3).
*/

//Variables to store latitude and longitude
float Latitude=-90.123456;
float Longitude=-180.123456;
//Variables to stor possible errors

bool tempExceeded;
bool leakDetected;

void setup()
{
  Serial3.begin(115200); //Start communication with the other Arduino
  Serial.begin(9600); //Start communication with the computer
}

void loop() {
    serialWriteGPS(Latitude,Longitude);
    //Minimum delay 200ms
    delay(200);
    SerialReadSafety();
    
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



void SerialReadSafety() {
        char serialMessage[]= {0};
        //Variable to store the string to be received
        serialMessage[15];
        //Read message from the other Arduino
        Serial3.readBytes(serialMessage,14);
        //Turns the string into a float starting after "Temp:"
        int index=String(serialMessage).indexOf("Temp:")+5;
        tempExceeded=String(serialMessage).substring(index).toInt();
        //Does the same for "Leak:"
        index=String(serialMessage).indexOf("Leak:")+5;
        leakDetected=String(serialMessage).substring(index).toInt();

         //For debugging
         Serial.println("-----Serial communication:-----");
         Serial.print("Message received:");
         Serial.println(serialMessage);
         Serial.print("tempExceeded=");
         Serial.print(tempExceeded);
         Serial.print(" leakDetected=");
         Serial.println(leakDetected);
         //Serial.println("-------------------------------");
    
         if (tempExceeded==1) {
             digitalWrite(LED_BUILTIN, HIGH);
         }else {
             digitalWrite(LED_BUILTIN, LOW);
         } 
}
