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
    pitchAngle = pitch_angle;
    yawAngle = yaw_angle;
    rollAngle = roll_angle;
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
    Serial.print("old angle: "); Serial.println(old_angle);
    float offset_old;
    if ((old_max - old_min) > 0 ) {
        offset_old = old_min;
    }
    else {
        offset_old = 360 - old_min;
    }
    Serial.print("offset old: "); Serial.println(offset_old);

    float offset_old_max = old_max + offset_old;
    if (offset_old_max < 0) offset_old_max += 360;

    float offset_old_angle = old_angle + offset_old;
    if (offset_old_angle > 360) offset_old_angle -= 360;

    Serial.print("offset_old_angle (og):"); Serial.println(offset_old_angle);

    // float offset_midpoint = 360.0 - offset_old_max;
    // if (offset_old_angle > offset_old_max && offset_old_angle < offset_midpoint) {
    //     offset_old_angle = offset_old_max;
    // }
    // else if (offset_old_angle > offset_midpoint && offset_old_angle < 360.0) {
    //     offset_old_angle = 0;
    // }

    float offset_new;
    if ((new_max - new_min) > 0 ) {
        offset_new = new_min;
    }
    else {
        offset_new = - (360 - new_min);
    }

    float offset_new_max = new_max - offset_new;
    Serial.print("offset old angle: "); Serial.println(offset_old_angle);
    Serial.print("offset new max: "); Serial.println(offset_new_max);
    Serial.print("offset old max: "); Serial.println(offset_old_max);
    float offset_new_angle = remap(offset_old_angle, 0.0, offset_old_max, 0.0, offset_new_max);
    Serial.print("offset new angle: "); Serial.println(offset_new_angle);

    Serial.print("offset new: "); Serial.println(offset_new);
    float new_angle = offset_new_angle - offset_new;

    Serial.print("new angle: "); Serial.println(new_angle);
    if (new_angle < 0) {
        new_angle = new_angle + 360;
    }

    // Serial.print("new angle: "); Serial.println(new_angle);

    // Serial.println("++++++++++");

    return new_angle;
}
