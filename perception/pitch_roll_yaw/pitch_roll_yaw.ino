#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

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
    Serial.print(F("Roll: "));
    Serial.print(orientation.roll);
    Serial.print(F("; "));
    Serial.print(F("Pitch: "));
    Serial.print(orientation.pitch);
    Serial.print(F("; "));
    Serial.print(F("Heading: "));
    Serial.print(orientation.heading);
    Serial.println(F("; "));
  }

  buttonPressed = digitalRead(buttonPin);
  Serial.println(buttonPressed);

  // THINK
  int x = 100;
  int y = map(x, 180,90,90,180);
  Serial.println(y);
  

  // ACT

  
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
