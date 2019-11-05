#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <Servo.h>

Servo pitchservo;  // create servo object to control a servo
Servo pinchservo;  // create servo object to control a servo
Servo yawservo;  // create servo object to control a servo
Servo rollservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int rollpos = 0;    // variable to store the servo position
int pinchpos = 45; //starts pinch in open position
int pitchpos = 90; //starts pitch in neutral position
int yawpos = 90; //starts yaw in neutral position

/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

// button variables
int buttonPin = 2;
int buttonPressed = 0;

// orientation for servos
int maxPitchSensor = 180;
int minPitchSensor = 90;


void setup(void)
{
  Serial.begin(115200);
  //  Serial.println(F("Adafruit 9 DOF Pitch/Roll/Heading /Example")); Serial.println("");
  pinMode(buttonPin, INPUT);
  /* Initialise the sensors */
  initSensors();
  pitchservo.attach(11);  // attaches the pitch servo on pin 11 to the servo object
  pinchservo.attach(9);  // attaches the pinch servo on pin 9 to the servo object
  yawservo.attach(6);  // attaches the yaw servo on pin 6 to the servo object
  rollservo.attach(10);  // attaches the roll servo on pin 10 to the servo object
}

/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop(void)
{
  // SENSE
  // get orientation data
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  if (!dof.accelGetOrientation(&accel_event, &orientation))
  {
    Serial.println("Calculating orientation failed");
  }

  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    //    Serial.print(F("Roll: "));
    //    Serial.print(orientation.roll);
    //    Serial.print(F("; "));
    //    Serial.print(F("Pitch: "));
    //    Serial.print(orientation.pitch);
    //    Serial.print(F("; "));
    //    Serial.print(F("Heading: "));
    //    Serial.print(orientation.heading);
    //    Serial.println(F("; "));
  }

  buttonPressed = digitalRead(buttonPin);
  // Serial.println(buttonPressed);

  // THINK
  int x = 100;
  int y = map(x, 180, 90, 90, 180);

  //Serial.println(y);
  // PITCH THINKING
  int handpitch = orientation.pitch; //making float into an int
  //Serial.println(handpitch);
  if (handpitch > -150 && handpitch < 0) {
    //Serial.println("less than 150");
    handpitch = -150;
  }
  else if (handpitch > 180) {
    //Serial.println("more than 180");
    handpitch = 180;
  }
  else if (0 < handpitch && handpitch < 90) {
    //Serial.println("between 0 and 90");
    handpitch = 90;
  }
  int clawpitch;
  if (-180 <= handpitch && handpitch <= -150) {
    //Serial.println("-180--150");
    clawpitch = map(handpitch, -150, -180, 60, 90);
  }
  else if (90 <= handpitch && handpitch <= 180) {
    //Serial.println("90-180");
    clawpitch = map(handpitch, 180, 90, 90, 180);
  }
  pitchpos = clawpitch;
  //Serial.println(clawpitch);
  // ROLL THINKING
  int handroll = orientation.roll; //making float into an int
  //Serial.println(handroll);
  if (handroll > 90) {
    //Serial.println("less than 150");
    handroll = 90;}
  else if (handroll < -90) {
    //Serial.println("less than 150");
    handroll = -90;}
  int clawroll;
    //Serial.println("-180--150");
    clawroll = map(handroll, 90, -90, 0, 180);
  rollpos = clawroll;
  // YAW THINKING
  int handyaw = orientation.heading; //making float into an int
  // Serial.println(handyaw);
  if (handyaw > 230) {
    //Serial.println("less than 150");
    handyaw = 230;}
  else if (handyaw <50) {
    //Serial.println("less than 150");
    handyaw = 50;}
  int clawyaw;
    //Serial.println("-180--150");
    clawyaw = map(handyaw, 230, 50, 45, 135);
  yawpos = clawyaw;
  yawpos=90;
  // ACT
  closeclaw(buttonPressed);
  pitchservo.write(pitchpos);
  rollservo.write(rollpos);
  yawservo.write(yawpos);

  Serial.print("Roll: "); Serial.print(handroll); Serial.print(" Pitch: "); Serial.println(handpitch);
}
// HELPFUL FUNCTION

void initSensors()
{
  if (!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }
  if (!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
}
void closeclaw(int buttonpress) {
  if (buttonpress == 1) {
    pinchpos = 0;
    pinchservo.write(pinchpos);
  }
  if (buttonpress == 0) {
    pinchpos = 45;
    pinchservo.write(pinchpos);
  }
}
