/*This file is part of a code test for the serial communication function between
  two Arduinos. This is the code for the receiver. We are testing for sending a string of text
  with the status of the temperature and possible leaks.
      
  Code uses Serial3, PINS 14(TX3) and 15(RX3).
*/
bool tempExceeded;
bool leakDetected;

void setup() {
  Serial.begin(9600); //Start communication with the computer
  Serial3.begin(115200); //Start communication with the other Arduino
  pinMode(LED_BUILTIN,OUTPUT);
}

void loop() {
     SerialReadSafety();
}

void SerialReadSafety() {
        //Variable to store the string to be received
        char serialMessage[14];
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
         Serial.println(tempExceeded);
         Serial.print("leakDetected=");
         Serial.println(leakDetected);
         Serial.println("-------------------------------");
    
         if (tempExceeded==1) {
             digitalWrite(LED_BUILTIN, HIGH);
         }else {
             digitalWrite(LED_BUILTIN, LOW);
         }
         
}
