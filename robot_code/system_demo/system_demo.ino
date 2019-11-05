// #include "HumanHand.h"
#include "rgpt_lib.h"
#include <Servo.h>

HumanHand humanhand;
RobotHand robothand;

int buttonPressed;
sensors_vec_t handOrientation;

int pitch_angle;
int yaw_angle;
int roll_angle;

void setup(void)
{
  Serial.begin(115200);
  humanhand.init(2); // initializes hand with button pin at pin 2
  robothand.init(11, 6, 5, 9); // pitch, yaw, roll, pinch
}

void loop(void)
{
  long startTime = millis();
  // SENSE
  humanhand.updateSensors();
  buttonPressed = humanhand.getFingerStatus();
  sensors_vec_t handOrientation = humanhand.getHandOrientation();

  // THINK

  pitch_angle = robothand.remapAngle(handOrientation.pitch, 260, 30, 180, 60);
  yaw_angle = robothand.remapAngle(handOrientation.heading, 50, 230, 45, 135);
  roll_angle = robothand.remapAngle(handOrientation.roll, 120, 260, 180, 0);

  robothand.setOrientation(pitch_angle, yaw_angle, roll_angle); //pitch, yaw, roll
  robothand.setClaw(buttonPressed);

  // ACT
  robothand.updateActuators();

  long endTime = millis();
  long delTime = endTime - startTime;

  // Serial.println(delTime);
}