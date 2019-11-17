// so that there aren't problems if we import this library twice
#ifndef RobotHand_h
#define RobotHand_h

// for standard arudino library and servos
#include <Arduino.h>
#include <Servo.h>
#include <FlexyStepper.h>
#include <Adafruit_Sensor.h>
#include <Mahony.h>


// ------ this is all for the 9dof imu on the RobotHand
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
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#else
#error "AHRS_VARIANT undefined! Please select a target sensor combination!"
#endif

// Create sensor instances.
#if AHRS_VARIANT == ST_LSM303DLHC_L3GD20
Adafruit_L3GD20_Unified       gyro(20);
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
#elif AHRS_VARIANT == ST_LSM9DS1
// ToDo!
#elif AHRS_VARIANT == NXP_FXOS8700_FXAS21002
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
#endif

// ------ end all the gyro stuff

class RobotHand
{
  public:
    RobotHand();
    void init(int left_step_pin, int left_direction_pin, int right_step_pin, int right_direction_pin, int pinch_pin);
    float remapAngle(float old_angle, float old_min, float old_max, float new_min, float new_max);
    float remap(float old_val, float old_min, float old_max, float new_min, float new_max);
    void setOrientation(int pitch_angle, int roll_angle);
    void setClaw(int button_press);
    void updateSensors();
    void updateActuators();
    Mahony filter;

  private:
    int pitchAngle;
    int rollAngle;
    int pinchAngle;

    int leftStepPin;
    int leftDirectionPin;
    int rightStepPin;
    int rightDirectionPin;
    int pinchPin;

    int leftStepperPos;
    int rightStepperPos;

    FlexyStepper LeftStepper;
    FlexyStepper RightStepper;
    Servo PinchServo;

    bool isClosed;

    // these variables are all for the gyro
    // Offsets applied to raw x/y/z mag values
    float mag_offsets[3]            = { 20.09F, 76.32F, 123.17F };

    // Soft iron error compensation matrix
    float mag_softiron_matrix[3][3] = { {  0.987, -0.013, -0.019 },
      { -0.013,  1.051, -0.020 },
      { -0.019, -0.020,  0.967 }
    };

    float mag_field_strength        = 57.53F;

    // Offsets applied to compensate for gyro zero-drift error for x/y/z
    float gyro_zero_offsets[3]      = { 2.0F, 0.0F, 0.0F };
    };

#endif
