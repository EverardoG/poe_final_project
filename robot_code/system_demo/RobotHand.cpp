#include <Arduino.h>
#include "RobotHand.h"

//using FX::Adafruit_FXAS21002C;
using FX::Adafruit_FXOS8700;
// Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);


RobotHand::RobotHand()
{
}

void RobotHand::init(int left_step_pin, int left_direction_pin, int right_step_pin, int right_direction_pin, int pinch_pin_1, int pinch_pin_2, int pinch_pin_3)
{
    leftStepPin = left_step_pin;
    leftDirectionPin = left_direction_pin;
    rightStepPin = right_step_pin;
    rightDirectionPin = right_direction_pin;

    pinchPin1 = pinch_pin_1;
    pinchPin2 = pinch_pin_2;
    pinchPin3 = pinch_pin_3;

    LeftStepper.connectToPins(leftStepPin, leftDirectionPin);
    RightStepper.connectToPins(rightStepPin, rightDirectionPin);

    LeftStepper.setCurrentPositionInSteps(0);
    LeftStepper.setSpeedInStepsPerSecond(10000);
    LeftStepper.setAccelerationInStepsPerSecondPerSecond(10000);

    RightStepper.setCurrentPositionInSteps(0);
    RightStepper.setSpeedInStepsPerSecond(10000);
    RightStepper.setAccelerationInStepsPerSecondPerSecond(10000);

    PinchServo1.attach(pinchPin1);
    PinchServo2.attach(pinchPin2);
    PinchServo3.attach(pinchPin3);

    using FX::ACCEL_RANGE_4G;
    if (!accelmag.begin(ACCEL_RANGE_4G))
    {
        Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
        while (1);
    }


}

void RobotHand::setOrientation(int pitch_angle, int roll_angle)
{

    pitchAngle = pitch_angle;
    rollAngle = roll_angle;

    int leftStepperPos = (- (int) roll_angle / 2.0 - (int) pitch_angle) * 8.89;
    int rightStepperPos = (- (int) roll_angle / 2.0 + (int) pitch_angle) * 8.89;

    // Serial.print("Left: "); Serial.print(leftStepperPos); Serial.print(" | Right: "); Serial.println(rightStepperPos);

    LeftStepper.setTargetPositionInSteps(leftStepperPos);
    RightStepper.setTargetPositionInSteps(rightStepperPos);

}

void RobotHand::setClaw(int pointerStatus, int thumbStatus, int middleStatus)
{
    double p_int = 1, p_factor = 2, m_int =1, m_factor = 1.2, t_int = 1, t_factor = 2.5;

    // double p_fraction = p_int - ((double(pointerStatus)/700) * p_factor);
    double p_fraction = -(p_int - ((double(pointerStatus)/700) * p_factor));
    double m_fraction = m_int - ((double(middleStatus)/950) * m_factor);
    double t_fraction = t_int - ((double(thumbStatus)/700) * t_factor);

    pointerAngle = int(p_fraction * 180);
    thumbAngle = int(t_fraction * 180);
    middleAngle = int(m_fraction * 180);
//    Serial.print("Thumb "); Serial.println(thumbAngle);
//    Serial.print("Pointer "); Serial.println(pointerAngle);
}
void RobotHand::updateSensors()
{
    /* Calculate pitch and roll from the raw accelerometer data */
    accelmag.getEvent(&accel_event, &mag_event);
    if (!dof.accelGetOrientation(&accel_event, &orientation))
    {
        Serial.println("ERROR: dof.accelGetOrientation in RobotHand failed");
    }

    /* Calculate the heading using the magnetometer */
    if (!dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
    {
        Serial.println("ERROR: dof.magGetOrientation in RobotHand failed");
    }

    robotOrientation = orientation;
}

void RobotHand::updateActuators()
{

    if (pointerAngle < 0) {
        pointerAngle = 0;
    }
    else if (pointerAngle > 180) {
        pointerAngle = 180;
    }

    if (thumbAngle < 0) {
        thumbAngle = 0;
    }
    else if (thumbAngle > 180) {
        thumbAngle = 180;
    }

    if (middleAngle < 0) {
        middleAngle = 0;
    }
    else if (middleAngle > 180) {
        middleAngle = 180;
    }



    PinchServo1.write(pointerAngle);
    delay(20);
    PinchServo2.write(thumbAngle);
    delay(20);
    PinchServo3.write(middleAngle);
    delay(20);

    Serial.print("pointer: ");Serial.print(pointerAngle);
    Serial.print(" | middle: ");Serial.print(middleAngle);
    Serial.print(" | thumb: ");Serial.println(thumbAngle);

    long startTime = millis();
    long endInterval = 10;

    // run steppers for the specified endInterval in milliseconds
    while (!LeftStepper.motionComplete() && !RightStepper.motionComplete() && (millis() - startTime) < endInterval){
        LeftStepper.processMovement();
        RightStepper.processMovement();
    }

}

void RobotHand::resetSteppers()
{
    LeftStepper.setCurrentPositionInSteps(0);
    RightStepper.setCurrentPositionInSteps(0);
}

int RobotHand::remap(float old_val, float old_min, float old_max, float new_min, float new_max)
{
    return (old_val - old_min)/(old_max - old_min)*(new_max - new_min) + new_min;
}

int RobotHand::remapAngle(float old_angle, float old_min, float old_max, float new_min, float new_max)
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

sensors_vec_t RobotHand::getRobotOrientation()
{
    return orientation;
}
