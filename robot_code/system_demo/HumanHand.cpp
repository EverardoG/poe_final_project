#include <Arduino.h>
#include "HumanHand.h"

// using namespace HUMAND_HAND;
// {

HumanHand::HumanHand()
{
}

void HumanHand::init(int button_pin, int flex_pin)
{
    // get IMU ready to roll
    /* Assign a unique ID to the sensors */
    Adafruit_9DOF                dof   = Adafruit_9DOF();
    Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
    Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

    // necessary for IMU
    if (!accel.begin())
    {
        Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
        while (1);
    }
    if (!mag.begin())
    {
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while (1);
    }
    // set up our button
    buttonPin = button_pin;
    pinMode(buttonPin, INPUT);

    // set up Filter
    FilterOnePole rollXLowpassFilter( LOWPASS, roll_filter_frequency);
    FilterOnePole rollYLowpassFilter( LOWPASS, roll_filter_frequency);
    FilterOnePole pitchXLowpassFilter( LOWPASS, pitch_filter_frequency);
    FilterOnePole pitchYLowpassFilter( LOWPASS, pitch_filter_frequency);
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

    // @Jamie change this to make sense for the current glove
    int button_status = digitalRead(buttonPin);
    int flex_states = analogRead(flexPin);
    fingerStatus = button_status;

    /*
    UPDATE HAND ORIENTATION
     */

    /* Calculate pitch and roll from the raw accelerometer data */
    accel.getEvent(&accel_event);
    if (!dof.accelGetOrientation(&accel_event, &orientation))
    {
        Serial.println("ERROR: dof.accelGetOrientation in HumanHand failed");
    }

    /* Calculate the heading using the magnetometer */
    mag.getEvent(&mag_event);
    if (!dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
    {
        Serial.println("ERROR: dof.magGetOrientation in HumanHand failed");
    }


    // split each angle into x, y componenets, filter on those, then
    // recombine the two values into an angle
    // This is necessary because of the wrapping that occurs at 0,360

    orientation.pitch += 180.0;
    orientation.pitch *= 3.14/180.0;
    float pitch_x = cos(orientation.pitch);
    float pitch_y = sin(orientation.pitch);

    float pitch_x_filtered = pitchXLowpassFilter.input(pitch_x);
    float pitch_y_filtered = pitchYLowpassFilter.input(pitch_y);

    orientation.pitch = atan2(pitch_y_filtered, pitch_x_filtered) * 180.0/3.14;

    orientation.roll += 180.0;
    orientation.roll *= 3.14/180.0;
    float roll_x = cos(orientation.roll);
    float roll_y = sin(orientation.roll);

    float roll_x_filtered = rollXLowpassFilter.input(roll_x);
    float roll_y_filtered = rollYLowpassFilter.input(roll_y);

    orientation.roll = atan2(roll_y_filtered, roll_x_filtered) * 180.0/3.14;
    // Serial.print("Pitch: "); Serial.print(orientation.pitch); Serial.print(" | Roll: "); Serial.println(orientation.roll);

    handOrientation = orientation;
}