#include <Arduino.h>
#include "HumanHand.h"

HumanHand::HumanHand()
{
}

void HumanHand::init(int button_pin)
{
    // get IMU ready to roll
    /* Assign a unique ID to the sensors */
    Adafruit_9DOF                dof   = Adafruit_9DOF();
    Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
    Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

    /* Update this with the correct SLP for accurate altitude measurements */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

    // necessary for IMU
    if (!accel.begin())
    {
        /* There was a problem detecting the LSM303 ... check your connections */
        // Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
        while (1);
    }
    if (!mag.begin())
    {
        /* There was a problem detecting the LSM303 ... check your connections */
        // Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while (1);
    }
    // set up our button
    buttonPin = button_pin;
    pinMode(buttonPin, INPUT);


}

int HumanHand::getFingerStatus()
{
    return fingerStatus;
}

sensors_vec_t HumanHand::getHandOrientation()
{
    return handOrientation;
}

void HumanHand::updateSensors()
{
    /*
    UPDATE FINGER STATUS
     */
    fingerStatus = digitalRead(buttonPin);

    /*
    UPDATE HAND ORIENTATION
     */

    // TODO: unclear why these are here - do we need to define these each time?


    /* Calculate pitch and roll from the raw accelerometer data */
    accel.getEvent(&accel_event);
    if (!dof.accelGetOrientation(&accel_event, &orientation))
    {
        // TODO: figure out how to raise an exception
        int error;
    }

    /* Calculate the heading using the magnetometer */
    mag.getEvent(&mag_event);
    if (!dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
    {
        int error;
    }

    // offsetting orientation to make values go from 0 to 360 instead of -180 to 180
    orientation.pitch = orientation.pitch + 180.0;
    orientation.roll = orientation.roll + 180.0;
    orientation.heading = orientation.heading + 180.0;

    handOrientation = orientation;

}
