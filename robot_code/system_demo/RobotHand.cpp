#include <Arduino.h>
#include "RobotHand.h"

// using namespace FX;
// using namespace ROBOT_HAND;
// using FX::Adafruit_FXAS21002C;
// using FX::Adafruit_FXOS8700;
using FX::Adafruit_FXAS21002C;
using FX::Adafruit_FXOS8700;
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);


RobotHand::RobotHand()
{
}

void RobotHand::init(int left_step_pin, int left_direction_pin, int right_step_pin, int right_direction_pin, int pinch_pin)
{

    int leftStepPin = left_step_pin;
    int leftDirectionPin = left_direction_pin;
    int rightStepPin = right_step_pin;
    int rightDirectionPin = right_direction_pin;

    pinchPin = pinch_pin;

    LeftStepper.connectToPins(leftStepPin, leftDirectionPin);
    RightStepper.connectToPins(rightStepPin, rightDirectionPin);

    LeftStepper.setCurrentPositionInSteps(0);
    LeftStepper.setSpeedInStepsPerSecond(10000);
    LeftStepper.setAccelerationInStepsPerSecondPerSecond(10000);

    RightStepper.setCurrentPositionInSteps(0);
    RightStepper.setSpeedInStepsPerSecond(10000);
    RightStepper.setAccelerationInStepsPerSecondPerSecond(10000);

    PinchServo.attach(pinchPin);

    // using namespace FX;
    // this code sets up the imu
    // using FX::AHRS_VARIANT;
    // using FX::NXP_FXOS8700_FXAS21002;

    if (!gyro.begin())
    {
        /* There was a problem detecting the gyro ... check your connections */
        Serial.println("Ooops, no gyro detected ... Check your wiring!");
        while (1);
    }
    #if AHRS_VARIANT == NXP_FXOS8700_FXAS21002
    using FX::ACCEL_RANGE_4G;
    if (!accelmag.begin(ACCEL_RANGE_4G))
    {
        Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
        while (1);
    }
    #else
    if (!accel.begin())
    {
        /* There was a problem detecting the accel ... check your connections */
        Serial.println("Ooops, no accel detected ... Check your wiring!");
        while (1);
    }

    if (!mag.begin())
    {
        /* There was a problem detecting the mag ... check your connections */
        Serial.println("Ooops, no mag detected ... Check your wiring!");
        while (1);
    }
    #endif

    // filter expect as an input the sampling rate of the sensor
    // in our case, that's 10 hz because that's how fast our code runs
    filter.begin(10);
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
        pinchAngle = 50;
    }
    else {
        pinchAngle = 160;
    }
    Serial.println(pinchAngle);
}
void RobotHand::updateSensors()
{
    // sensor_t senseboi;
    // gyro.getSensor(&senseboi);// throws error with getSensor
    // using namespace FX;
    // using FX::sensors_event_t;
    sensors_event_t gyro_event;
    sensors_event_t accel_event;
    sensors_event_t mag_event;
    // gyro.begin(mag_event);// doesnt throw an error with the begin method

    // Get new data samples
    // using FX::Adafruit_FXAS21002C;
    gyro.getEvent(&gyro_event); // throws error with getEvent
    // #if AHRS_VARIANT == NXP_FXOS8700_FXAS21002
    // accelmag.getEvent(&accel_event, &mag_event);
    // #else
    // accel.getEvent(&accel_event);
    // mag.getEvent(&mag_event);
    // #endif

    // Apply mag offset compensation (base values in uTesla)
    float x = mag_event.magnetic.x - mag_offsets[0];
    float y = mag_event.magnetic.y - mag_offsets[1];
    float z = mag_event.magnetic.z - mag_offsets[2];

    // Apply mag soft iron error compensation
    float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
    float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
    float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

    // Apply gyro zero-rate error compensation
    float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
    float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
    float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

    // The filter library expects gyro data in degrees/s, but adafruit sensor
    // uses rad/s so we need to convert them first (or adapt the filter lib
    // where they are being converted)
    gx *= 57.2958F;
    gy *= 57.2958F;
    gz *= 57.2958F;

    // Update the filter
    //You can download the requested file from the pool / main / libp / libpng / subdirectory at:    security.ubuntu.com / ubuntu
    filter.update(gx, gy, gz,
                    accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                    mx, my, mz);

    // Print the orientation filter output
    // Note: To avoid gimbal lock you should read quaternions not Euler
    // angles, but Euler angles are used here since they are easier to
    // understand looking at the raw values. See the ble fusion sketch for
    // and example of working with quaternion data.
    // float roll = filter.getRoll();
    // float pitch = filter.getPitch();
    // float heading = filter.getYaw();
}

void RobotHand::updateActuators()
{
    PinchServo.write(pinchAngle);

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
