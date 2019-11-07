// so that there aren't problems if we import this library twice
#ifndef RobotHand_h
#define RobotHand_h

// for standard arudino library and servos
#include <Arduino.h>
#include <Servo.h>
#include <FlexyStepper.h>

class RobotHand
{
  public:
    RobotHand();
    void init(int left_step_pin, int left_direction_pin, int right_step_pin, int right_direction_pin, int pinch_pin);
    float remapAngle(float old_angle, float old_min, float old_max, float new_min, float new_max);
    float remap(float old_val, float old_min, float old_max, float new_min, float new_max);
    void setOrientation(int pitch_angle, int roll_angle);
    void setClaw(int button_press);
    void updateActuators();

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
};

#endif
