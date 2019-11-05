// so that there aren't problems if we import this library twice
#ifndef RobotHand_h
#define RobotHand_h

// for standard arudino library and servos
#include <Arduino.h>
#include <Servo.h>

class RobotHand
{
  public:
    RobotHand();
    void init(int pitch_pin, int yaw_pin, int roll_pin, int pinch_pin);
    float remapAngle(float old_angle, float old_min, float old_max, float new_min, float new_max);
    void setOrientation(int pitch_angle, int yaw_angle, int roll_angle);
    void setClaw(int button_press);
    void updateActuators();

  private:
    float remap(float old_val, float old_min, float old_max, float new_min, float new_max);
    void writeToServo(Servo servo, int servoPin,int newAngle);

    int pitchAngle;
    int yawAngle;
    int rollAngle;
    int pinchAngle;

    int rollPin;
    int pitchPin;
    int yawPin;
    int pinchPin;

    Servo pitchservo;
    Servo yawservo;
    Servo rollservo;
    Servo pinchservo;

    bool isClosed;
};

#endif
