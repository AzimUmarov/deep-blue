/*Safety system module
                               +-----+
        +----[PWR]-------------| USB |--+
        |    GND/RST2 [ ][ ]   +-----+  |
        |  MOSI2/SCK2 [ ][ ]     SCL[ ] |
        |    5V/MISO2 [ ][ ]     SDA[ ] |
        |                       AREF[ ] |
        |                        GND[ ] |
        | [ ]N/C                  13[ ]~|
        | [ ]IOREF                12[ ]~|
        | [ ]RST                  11[ ]~|
        | [ ]3V3   +----------+   10[ ]~|
        | [ ]5v    | Arduino  |    9[ ]~|
        | [ ]GND   |    DUE   |    8[ ]~|
        | [ ]GND   +----------+         |
        | [ ]Vin                   7[ ]~|
        |                          6[ ]~|
        | [ ]A0                    5[ ]~|
        | [ ]A1                    4[ ]~|
        | [ ]A2               INT5/3[ ]~|
        | [ ]A3               INT4/2[ ]~|
        | [ ]A4                 TX>1[ ]~|
        | [ ]A5                 RX<0[ ]~|
        | [ ]A6                         |
        | [ ]A7               TX3/14[ ] |
        |                     RX3/15[ ] |
        | [ ]A8               TX2/16[ ] |
        | [ ]A9               RX2/17[ ] |
        | [ ]A10         TX1/INT3/18[ ] |
        | [ ]A11         RX1/INT2/19[ ] |
        | [ ]A12     I2C-SDA/INT1/20[ ] |
        | [ ]A13     I2C-SCL/INT0/21[ ] |
        | [ ]A14                        |
        | [ ]A15                        |
        |          RST SCK MISO         |
        |     ICSP [ ] [ ] [ ]          |
        |          [ ] [ ] [ ]          |
        |          GND MOSI 5V          |
        |                               |
        |     2560          ____________/
         \_________________/
*/
//===================================================================================================================================================
// Libraries for navigation system below

#include <Wire.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <Servo.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_Sensor.h>

//===================================================================================================================================================
// Libraries for navigation and safety system below

#include "cc1000.h" //Library for the Radio commmunications
#include <DHT.h> //Library for the temperature sensor
#include <TinyGPSPlus.h>

//===================================================================================================================================================
// global variables and other items used by navigation system below

//float lat, lon;
int year;
byte month, day, hour, minute, second, hundredths;
//TinyGPS gps;

//Pressure sensor
const int pressureInput = A0; //select the analog input pin for the pressure transducer
const int pressureZero = 97; //analog reading of pressure transducer at 0psi
const int pressureMax = 921.6; //analog reading of pressure transducer at 100psi
const int pressuretransducermaxPSI = 145.04; //psi value of transducer being used
const int baudRate = 9600; //constant integer to set the baud rate for serial monitor
const int sensorreadDelay = 250; //constant integer to set the sensor read delay in milliseconds
float pressureValue = 0; //variable to store the value coming from the pressure transducer
float depth;


//Servo
Servo fMotor, hMotor, vMotor1, vMotor2;

// Object avoidance distances (in inches)
#define echo 4
#define trig 3
#define SAFE_DISTANCE 4000
#define MID_DISTANCE 2500
#define STOP_DISTANCE 1000
#define TURN_CW 1
#define TURN_CCW 2
#define TURN_STRAIGHT 0
long time, sonarDistance;

//Magnetometer
int check = 1;
Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);
int targetHeading;              // where we want to go to reach current waypoint
int currentHeading;             // where we are actually facing now
int headingError;               // signed (+/-) difference between targetHeading and currentHeading
float rotationAngle;
float headingDegrees;
String direction;
#define HEADING_TOLERANCE 10     // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading
#define TURN_CW 1
#define TURN_CCW 2
#define TURN_STRAIGHT 0
enum directions {cw = TURN_CW, ccw = TURN_CCW, straight = TURN_STRAIGHT} ;
directions turnDirection = straight;

//GPS
float currentLat,
      currentLong,
      targetLat,
      targetLong;
int distanceToTarget,            // current distance to target (current waypoint)
    originalDistanceToTarget;    // distance to original waypoing when we started navigating to it

//Motors
#define FAST_SPEED 1900
#define NORMAL_SPEED 1750
#define TURN_SPEED 1675
#define DIVE_SPEED 1650
#define SLOW_SPEED 1600
#define STOP 1500
int speed = NORMAL_SPEED;


//===================================================================================================================================================
// global variables and other items used by communication and safety system below

float firetemp = 70; //Maximum allowed temperature in the system
TinyGPSPlus gps;
Cc1000 trx;

const int leakdigital = 12; // leak pin
const int leakVCC = 13; // leak pin

#define DHTPIN 7     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

//===================================================================================================================================================
// Main function and loop
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  fMotor.attach(8);
  hMotor.attach(9);
  vMotor1.attach(10);
  vMotor2.attach(11);
  txsetup();
  leaksetup();
  dhtsetup();
  pinMode (trig, OUTPUT);
  pinMode (echo, INPUT);
  if (!mag.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }

  fMotor.writeMicroseconds(1500);
  hMotor.writeMicroseconds(1500);
  vMotor1.writeMicroseconds(1500);
  vMotor2.writeMicroseconds(1500); // send "stop" signal to ESC.
  delay(7000); // delay to allow the ESC to recognize the stopped signal
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      TXgps();
      distanceToWaypoint();
      courseToWaypoint();
    }
    currentHeading = readCompass();    // get our current heading
    calcDesiredTurn();                // calculate how we would optimatally turn, without regard to obstacles
    if (check == 1) {
      initialOrientation();
    }
    // distance in front of us, move, and avoid obstacles as necessary
    checkSonar();
    moveAndAvoid();
  }
}

//===================================================================================================================================================
// Functions for navigation system below

void checkSonar(void)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  time = pulseIn(echo, HIGH);
  sonarDistance = (time / 2) / 29.1;
} // checkSonar()

int distanceToWaypoint()
{
  float delta = radians(currentLong - targetLong);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  float lat1 = radians(currentLat);
  float lat2 = radians(targetLat);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  distanceToTarget =  delta * 6372795;

  return distanceToTarget;
}

int courseToWaypoint()
{
  float dlon = radians(targetLong - currentLong);
  float cLat = radians(currentLat);
  float tLat = radians(targetLat);
  float a1 = sin(dlon) * cos(tLat);
  float a2 = sin(cLat) * cos(tLat) * cos(dlon);
  a2 = cos(cLat) * sin(tLat) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  targetHeading = degrees(a2);
  return targetHeading;
}
void readDepth(){
  pressureValue = analogRead(pressureInput); //reads value from input pin and assigns to variable
  pressureValue = ((pressureValue-pressureZero)*pressuretransducermaxPSI)/(pressureMax-pressureZero); //conversion equation to convert analog reading to psi
  delay(sensorreadDelay); //delay in milliseconds between read values
  float pascals = pressureValue*6894.7572932;
  depth = pascals/10045;
}
int readCompass(void)
{
  sensors_event_t event;
  mag.getEvent(&event);

  float Pi = 3.14159;
  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;

#define DEC_ANGLE 0.069
  heading += DEC_ANGLE;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;

  return ((int)headingDegrees);
}

void calcDesiredTurn(void)
{
  // calculate where we need to turn to head to destination
  if (abs(currentHeading - targetHeading) > 180) {
    if ((currentHeading - targetHeading) > 0) {
      rotationAngle = 360 - (currentHeading - targetHeading);
      rotationAngle = -rotationAngle;
    }
    else {
      rotationAngle = 360 - abs(currentHeading - targetHeading);
    }
  }
  else {
    rotationAngle = currentHeading - targetHeading;
  }
  if (abs(rotationAngle) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
    turnDirection = straight;
  else if (rotationAngle < 0)
    turnDirection = cw;
  else if (rotationAngle > 0)
    turnDirection = ccw;
  else
    turnDirection = straight;
}
void moveAndAvoid(void)
{
  if (sonarDistance >= SAFE_DISTANCE)       // no close objects in front of car
  {
    if (turnDirection == straight)
      fMotor.writeMicroseconds(FAST_SPEED);
    else {
      fMotor.writeMicroseconds(TURN_SPEED);
      if (turnDirection == cw) {
        hMotor.writeMicroseconds(map(abs(rotationAngle), 0, 180, 1500, 1900));
      }
      else if (turnDirection == ccw) {
        hMotor.writeMicroseconds(map(abs(rotationAngle), 0, 180, 1500, 1100));
      }
    }
    return;
  }

  if (sonarDistance > MID_DISTANCE && sonarDistance < SAFE_DISTANCE)    // not yet time to turn, but slow down
  {
    if (turnDirection == straight)
      fMotor.writeMicroseconds(NORMAL_SPEED);
    else {
      fMotor.writeMicroseconds(TURN_SPEED);
      if (turnDirection == cw) {
        hMotor.writeMicroseconds(map(abs(rotationAngle), 0, 180, 1500, 1900));
      }
      else if (turnDirection == ccw) {
        hMotor.writeMicroseconds(map(abs(rotationAngle), 0, 180, 1500, 1100));
      }
    }
    return;
  }

  if (sonarDistance <  MID_DISTANCE && sonarDistance > STOP_DISTANCE)  // getting close, time to turn to avoid object
  {
    if (turnDirection == straight)
      fMotor.writeMicroseconds(SLOW_SPEED);
    else {
      fMotor.writeMicroseconds(TURN_SPEED);
      if (turnDirection == cw) {
        hMotor.writeMicroseconds(map(abs(rotationAngle), 0, 180, 1500, 1900));
      }
      else if (turnDirection == ccw) {
        hMotor.writeMicroseconds(map(abs(rotationAngle), 0, 180, 1500, 1100));
      }
    }

    return;
  }


  if (sonarDistance <  STOP_DISTANCE)          // too close, stop and back up
  {
    fMotor.writeMicroseconds(STOP);
    hMotor.writeMicroseconds(STOP);
    while((sonarDistance <  STOP_DISTANCE)&&(depth<10)){
      readDepth();
      checkSonar();
      vMotor1.writeMicroseconds(1900);
      vMotor2.writeMicroseconds(1900);
    }
    if(
    vMotor1.writeMicroseconds(1500);
    vMotor2.writeMicroseconds(1500);
    underWater();
    underwater = true;
    return;
  }

}

void underWater(){
  if(sonarDistance > STOP_DISTANCE){
    fMotor.writeMicroseconds(1900);
  }
  else if(sonarDistance < STOP_DISTANCE){
    while(sonarDistance < STOP_DISTANCE{
      hMotor.writeMicroseconds(1900);
    }
  }
}

void initialOrientation(void) {
  while (abs(rotationAngle) <= HEADING_TOLERANCE) {
    if (turnDirection == cw) {
      hMotor.writeMicroseconds(map(abs(rotationAngle), 0, 180, 1500, 1900));
    }
    else if (turnDirection == ccw) {
      hMotor.writeMicroseconds(map(abs(rotationAngle), 0, 180, 1500, 1100));
    }
  }
  check = 0;
}

//============================================================================================================================================
// Functions for communication and safety system below

void leaksetup() {
  pinMode(leakVCC, OUTPUT);
  digitalWrite(leakVCC, HIGH);
  pinMode(leakdigital, INPUT);
}

void dhtsetup() {
  dht.begin();
}

// the setup function runs once when you press reset or power the board
void txsetup() {
  // initialize digital pin 13 (LED diode) as an output.
  pinMode(13, OUTPUT);

  //inicjalization cc1000 module
  trx.init();
  trx.set_modem_mode(RTTY_MODE);
  trx.set_power(PA_VALUE_0DBM);
  trx.set_deviation(600);
  trx.set_bitrate(300);
  trx.set_frequency(432920000);
  //trx.set_frequency(432920000, VCO_AB, true);
  trx.set_trx_mode(TX_MODE);

}

//////////////// FUNCTIONS OF SENSORS BELOW

bool checkleak()
{
  if (digitalRead(leakdigital) == LOW) {
    Serial.println("Digital value : wet");
    return true;
    delay (10);
  } else {
    Serial.println("Digital value : dry");
    return false;
  }
}

bool checkfire()
{
  //Read data and store it to variables hum and temp
  int hum = dht.readHumidity();
  int temp = dht.readTemperature();
  //Print temp and humidity values to serial monitor
  Serial.print("Humidity: ");
  Serial.print(hum);
  Serial.print(" %, Temp: ");
  Serial.print(temp);
  Serial.println(" Celsius");

  if (temp >= firetemp) {
    Serial.println("Fire on board");
    return true;
    delay (10);
  } else {
    Serial.println("No fire on board");
    return false;
  }
}



/////////////////////////RADIO + GPS FUNCTION
void TXgps()
{
  checkfire();
  checkleak();
  String str = " ";
  if (gps.location.isValid())
  {
    currentLat = gps.location.lat();
    currentLong = gps.location.lng();
    str = str + "$$$$$$$$ @@@@@@@@ ######## $$$$$$$$" + "Latitude: " + String(currentLat, 6) + "Longitude: " + String(currentLong, 6);
  } else
  {
    str = str + "%$%$%$%$ Gps invalid ##@#@#@#";
  }
  if (checkleak() == true) {
    str = str + "Leak on board %#$$#$#$#$##$";
  }
  if (checkfire() == true) {
    str = str + "Fire on board %#$$#$#$#$##$  ";
  }
  Serial.write((char*)str.c_str());
  trx.send_data(str);
  trx.set_trx_mode(TX_MODE);
  Serial.println();
  delay(1000);
}
