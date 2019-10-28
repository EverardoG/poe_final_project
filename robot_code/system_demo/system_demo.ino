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
  robothand.init(11, 6, 10, 9); // pitch, yaw, roll, pinch
}

void loop(void)
{
  Serial.println("-----------");
  // SENSE
  humanhand.updateSensors();
  buttonPressed = humanhand.getFingerStatus();
  sensors_vec_t handOrientation = humanhand.getHandOrientation();

  // THINK
  // Serial.println(handOrientation.pitch);
  pitch_angle = robothand.remapAngle(handOrientation.pitch, 250, 50, 180, 60);
  Serial.print("pitch angle: "); Serial.println(pitch_angle);
  Serial.println("+++++++++");
  yaw_angle = robothand.remapAngle(handOrientation.heading, 50, 230, 45, 135);
  roll_angle = robothand.remapAngle(handOrientation.roll, 270, 90, 0, 180);
  // Serial.println(handOrientation.pitch);
  // Serial.println(pitch_angle);

  robothand.setOrientation(pitch_angle, yaw_angle, roll_angle); //pitch, yaw, roll
  robothand.setClaw(buttonPressed);
  // pitch_angle = robothand.remapAngle(handOrientation.pitch, 330, 90, 60, 180);
  // yaw_angle = robothand.remapAngle(handOrientation.heading, 230, 50, 45, 135);
  // roll_angle = robothand.remapAngle(handOrientation.roll, 90, 270, 0, 180);

  // Serial.println("------");
  // Serial.print(pitch_angle); Serial.print(" : ");
  // Serial.print(yaw_angle); Serial.print(" : ");
  // Serial.println(roll_angle);

  // ACT
  robothand.updateActuators();
}