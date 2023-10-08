#include <Wire.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <Servo.h>
#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_Sensor.h>

//GPS
float lat, lon;
int year;
byte month, day, hour, minute, second, hundredths;
TinyGPS gps;
float currentLat,
      currentLong,
      targetLat,
      targetLong;
int distanceToTarget,            // current distance to target (current waypoint)
    originalDistanceToTarget;    // distance to original waypoing when we started navigating to it

//Servo
Servo fMotor, hMotor, vMotor1, vMotor2;

// Object avoidance distances (in inches)
#define SAFE_DISTANCE 4000
#define MID_DISTANCE 2500
#define STOP_DISTANCE 1000
#define TURN_CW 1
#define TURN_CCW 2
#define TURN_STRAIGHT 0
long sonarDistance, time;
enum directions {cw = TURN_CW, ccw = TURN_CCW, straight = TURN_STRAIGHT} ;
directions turnDirection = straight;

//Magnetometer
int check = 1;
Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);
int targetHeading;              // where we want to go to reach current waypoint
int currentHeading;             // where we are actually facing now
int headingError;               // signed (+/-) difference between targetHeading and currentHeading
float rotationAngle;
#define HEADING_TOLERANCE 5     // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading

//Motors
#define FAST_SPEED 1900
#define NORMAL_SPEED 1750
#define TURN_SPEED 1675
#define SLOW_SPEED 1600
#define STOP 1500
int speed = NORMAL_SPEED;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  fMotor.attach(8);
  hMotor.attach(9);
  vMotor1.attach(10);
  vMotor2.attach(11);

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
      gps.f_get_position(&lat, &lon);
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

void checkSonar(void)
{
  time = pulseIn(4, HIGH);
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
    return;
  }

}

void initialOrientation(void) {
  while (abs(rotationAngle) >= HEADING_TOLERANCE) {
    if (turnDirection = = cw) {
      hMotor.writeMicroseconds(map(abs(rotationAngle), 0, 180, 1500, 1900));
    }
    else if (turnDirection == ccw) {
      hMotor.writeMicroseconds(map(abs(rotationAngle), 0, 180, 1500, 1100));
    }
  }
  check = 0;
}
