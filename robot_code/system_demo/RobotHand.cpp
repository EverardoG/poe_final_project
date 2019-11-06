#include <Arduino.h>
#include "RobotHand.h"

RobotHand::RobotHand()
{
}

void RobotHand::init(int left_step_pin, int left_direction_pin, int right_step_pin, int right_direction_pin)
{

    int leftStepPin = left_step_pin;
    int leftDirectionPin = left_direction_pin;
    int rightStepPin = right_step_pin;
    int rightDirectionPin = right_direction_pin;

    LeftStepper.connectToPins(leftStepPin, leftDirectionPin);
    RightStepper.connectToPins(rightStepPin, rightDirectionPin);

    LeftStepper.setCurrentPositionInSteps(0);
    LeftStepper.setSpeedInStepsPerSecond(10000);
    LeftStepper.setAccelerationInStepsPerSecondPerSecond(10000);

    RightStepper.setCurrentPositionInSteps(0);
    RightStepper.setSpeedInStepsPerSecond(10000);
    RightStepper.setAccelerationInStepsPerSecondPerSecond(10000);

}

void RobotHand::setOrientation(int pitch_angle, int roll_angle)
{

    pitchAngle = pitch_angle;
    rollAngle = roll_angle;

    int leftStepperPos = (- (float) roll_angle / 2.0 - (float) pitch_angle) * 8.89;
    int rightStepperPos = (- (float) roll_angle / 2.0 + (float) pitch_angle) * 8.89;

    // Serial.print("Left: "); Serial.print(leftStepperPos); Serial.print(" | Right: "); Serial.println(rightStepperPos);

    LeftStepper.setTargetPositionInSteps(leftStepperPos);
    RightStepper.setTargetPositionInSteps(rightStepperPos);

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
    long startTime = millis();
    long endInterval = 10;

    // run steppers for the specified endInterval in milliseconds
    while (!LeftStepper.motionComplete() && !RightStepper.motionComplete() && (millis() - startTime) < endInterval){
        boolean isTrue = (millis() - startTime) < endInterval;
        // Serial.print(millis()); Serial.print(" | ");
        // Serial.print(startTime); Serial.print(" | ");
        // Serial.println(isTrue);
        LeftStepper.processMovement();
        RightStepper.processMovement();
    }

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

// void RobotHand::writeToServo(Servo servo, int servoPin,int newAngle) {
//     // Serial.print(newAngle); Serial.print(" "); Serial.println(servo.read());
//     if (newAngle != servo.read()) {
//          if (!servo.attached()) {
//              servo.attach(servoPin);
//          }
//         servo.write(newAngle);
//         Serial.println(newAngle);
//         // Serial.println(newAngle);
//     }
//      else if (servo.attached()) {
//          servo.detach();
//      }
// }
