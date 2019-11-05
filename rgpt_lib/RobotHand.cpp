#include <Arduino.h>
#include "RobotHand.h"

RobotHand::RobotHand()
{
}

void RobotHand::init(int pitch_pin, int yaw_pin, int roll_pin, int pinch_pin)
{
    pitchPin = pitch_pin;
    yawPin = yaw_pin;
    rollPin = roll_pin;
    pinchPin = pinch_pin;


    pitchservo.attach(pitch_pin);
    // yawservo.attach(yaw_pin);
    rollservo.attach(roll_pin);
    pinchservo.attach(pinch_pin);

}

void RobotHand::setOrientation(int pitch_angle, int yaw_angle, int roll_angle)
{
    // prev_pitchAngle = pitchAngle;
    // prev_rollAngle = rollAngle;

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
    // Serial.println(pinchAngle);
    writeToServo(pitchservo, pitchPin, pitchAngle);
    writeToServo(rollservo, rollPin, rollAngle);
    writeToServo(pinchservo, pinchPin, pinchAngle);
}

float RobotHand::remap(float old_val, float old_min, float old_max, float new_min, float new_max)
{
    return (old_val - old_min)/(old_max - old_min)*(new_max - new_min) + new_min;
}

float RobotHand::remapAngle(float old_angle, float old_min, float old_max, float new_min, float new_max)
{
    // adding an offset if the min is actually greater than the max
    // this handles the case where we cross over the 0,360 point
    if (old_min > old_max) {
        if (old_angle <= old_max + 0.5*(360 - (old_min - old_max)) && old_angle > 0) {
            old_angle += 360;
        }
        old_max += 360;
    }

    // this rams the old angle up or down if it's out of bounds
    if (old_min < old_max) {
        if (old_angle > old_max) {
            old_angle = old_max;
        }
        else if (old_angle < old_min) {
            old_angle = old_min;
        }
    }
    else {
        if (old_angle < old_max) {
            old_angle = old_max;
        }
        else if (old_angle > old_min) {
            old_angle = old_min;
        }
    }

    // this adds an offset to the new values if they cross
    // the 0,360 point
    if (new_min > new_max) {
        new_max += 360;
        new_min += 360;
    }

    // do the actual remapping
    int new_angle = remap(old_angle, old_min, old_max, new_min, new_max);

    // take off any 360 offset by wrapping around 360
    new_angle %= 360;

    return new_angle;
}

void RobotHand::writeToServo(Servo servo, int servoPin,int newAngle) {
    // Serial.print(newAngle); Serial.print(" "); Serial.println(servo.read());
    if (newAngle != servo.read()) {
         if (!servo.attached()) {
             servo.attach(servoPin);
         }
        servo.write(newAngle);
        Serial.println(newAngle);
        // Serial.println(newAngle);
    }
     else if (servo.attached()) {
         servo.detach();
     }
}
