#include <Arduino.h>
#include "RobotHand.h"

RobotHand::RobotHand()
{
}

void RobotHand::init(int pitch_pin, int yaw_pin, int roll_pin, int pinch_pin)
{
    pitchservo.attach(pitch_pin);
    yawservo.attach(yaw_pin);
    rollservo.attach(roll_pin);
    pinchservo.attach(pinch_pin);

}

void RobotHand::setOrientation(int pitch_angle, int yaw_angle, int roll_angle)
{
    int pitchAngle = pitch_angle;
    int yawAngle = yaw_angle;
    int rollAngle = roll_angle;
}

void RobotHand::setClaw(int button_press)
{
    if (button_press == 1) {
        pinchAngle = 0;
    }
    else {
        pinchAngle = 45;
    }
}

void RobotHand::updateActuators()
{
    pitchservo.write(pitchAngle);
    yawservo.write(yawAngle);
    rollservo.write(rollAngle);
    pinchservo.write(pinchAngle);
}

float RobotHand::remap(float old_val, float old_min, float old_max, float new_min, float new_max)
{
    return (old_val - old_min)/(old_max - old_min)*(new_max - new_min) + new_min;
}

float RobotHand::remapAngle(float old_angle, float old_min, float old_max, float new_min, float new_max)
{
    float offset_old;
    if ((old_max - old_min) > 0 ) {
        offset_old = old_min;
    }
    else {
        offset_old = old_min;
    }

    float offset_old_max = old_max - offset_old;
    float offset_old_angle = old_angle - offset_old;

    float offset_midpoint = 360.0 - offset_old_max;
    if (offset_old_angle > offset_old_max && offset_old_angle < offset_midpoint) {
        offset_old_angle = offset_old_max;
    }
    else if (offset_old_angle > offset_midpoint && offset_old_angle < 360.0) {
        offset_old_angle = 0;
    }

    float offset_new;
    if ((new_max - new_min) > 0 ) {
        offset_new = new_min;
    }
    else {
        offset_new = - (360 - new_min);
    }

    float offset_new_max = new_max - offset_new;
    float offset_new_angle = remap(offset_old_angle, 0.0, offset_old_max, 0.0, offset_new_max);
    float new_angle = offset_new_angle + offset_new;

    if (new_angle < 0) {
        new_angle = new_angle + 360;
    }

    return new_angle;
}