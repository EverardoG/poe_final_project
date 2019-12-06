// so that there aren't problems if we import this library twice
#ifndef RobotHand_h
#define RobotHand_h

// for standard arudino library and servos
#include <Arduino.h>
#include <Servo.h>
#include <FlexyStepper.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_9DOF.h>

#define ST_LSM303DLHC_L3GD20        (0)
#define ST_LSM9DS1                  (1)
#define NXP_FXOS8700_FXAS21002      (2)
#define AHRS_VARIANT   NXP_FXOS8700_FXAS21002

#if AHRS_VARIANT == ST_LSM303DLHC_L3GD20
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#elif AHRS_VARIANT == ST_LSM9DS1
// ToDo!
#elif AHRS_VARIANT == NXP_FXOS8700_FXAS21002
//#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#else
#error "AHRS_VARIANT undefined! Please select a target sensor combination!"
#endif

class RobotHand
{
  public:
    RobotHand();
    void init(int left_step_pin, int left_direction_pin, int right_step_pin, int right_direction_pin, int pinch_pin_1, int pinch_pin_2);
    int remapAngle(float old_angle, float old_min, float old_max, float new_min, float new_max);
    int remap(float old_val, float old_min, float old_max, float new_min, float new_max);
    void setOrientation(int pitch_angle, int roll_angle);
    void setClaw(int pointerStatus, int thumbStatus);
    void updateSensors();
    void updateActuators();
    sensors_vec_t getRobotOrientation();
    void resetSteppers();
    int mode = 0;

  private:
    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_vec_t   orientation;

    sensors_vec_t   robotOrientation;

    int pitchAngle;
    int rollAngle;

    int thumbAngle;
    int pointerAngle;

    int leftStepPin;
    int leftDirectionPin;
    int rightStepPin;
    int rightDirectionPin;
    int pinchPin1;
    int pinchPin2;

    int leftStepperPos;
    int rightStepperPos;

    Servo PinchServo1;
    Servo PinchServo2;

    FlexyStepper LeftStepper;
    FlexyStepper RightStepper;

    bool isClosed;
};
#endif
