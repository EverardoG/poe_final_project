// so that there aren't problems if we import this library twice
#ifndef HumanHand_h
#define HumanHand_h

// test- may fix gyroscope name conflicts
// namespace HUMAND_HAND
// {
// for standard arudino things
#include <Arduino.h>

// all the necessary libraries for our gyro
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

// necessary for filtering
#include "Filters.h"
#include <math.h>

struct trips
{
  int thumb;
  int pointer;
  int middle;
};

class HumanHand
{
  public:
    HumanHand();
    void init(int button_pin, int thumb_pin, int pointer_pin, int middle_pin);
    void updateSensors();
    trips getFingerStatus();
    sensors_vec_t getHandOrientation();

  private:
    int buttonPin;
    int thumbPin;
    int pointerPin;
    int middlePin;
    int pointerStatus;
    int thumbStatus;
    int middleStatus;
    int button_status;
    int pointer_status;
    int thumb_status;
    int middle_status;
    sensors_vec_t handOrientation;

    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_vec_t   orientation;

    Adafruit_9DOF dof;
    Adafruit_LSM303_Accel_Unified accel;
    Adafruit_LSM303_Mag_Unified mag;

    FilterOnePole rollXLowpassFilter;
    FilterOnePole rollYLowpassFilter;
    FilterOnePole pitchXLowpassFilter;
    FilterOnePole pitchYLowpassFilter;

    int roll_filter_frequency = 0.5;
    int pitch_filter_frequency = 100000.0;

    int lastPitch;
    int lastRoll;

    boolean firstRun;
  };
// }
#endif
