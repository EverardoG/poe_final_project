// #include "HumanHand.h"
#include "rgpt_lib.h"

HumanHand humanhand;
RobotHand robothand;

int buttonPressed;
sensors_vec_t handOrientation;

float pitch_angle;
float roll_angle;

void setup(void)
{
  Serial.begin(115200);
  humanhand.init(2); // initializes hand with button pin at pin 2
  robothand.init(9, 8, 3, 2); // left_step_pin, left_direction_pin, right_step_pin, right_direction_pin
}

void loop(void)
{
  long startTime = millis();

  // SENSE
  humanhand.updateSensors();
  buttonPressed = humanhand.getFingerStatus();
  sensors_vec_t handOrientation = humanhand.getHandOrientation();

  // THINK
  if (handOrientation.pitch >= 0) { pitch_angle = robothand.remap(handOrientation.pitch, 0.0, -180.0, 0.0, 180.0); }
  else { pitch_angle = robothand.remap(handOrientation.pitch, 0.0, 180.0, 0.0, -180.0); }

  if (handOrientation.roll >= 0) { roll_angle = robothand.remap(handOrientation.roll, 180.0, 0.0, 0.0, 180.0); }
  else { roll_angle = robothand.remap(handOrientation.roll, -180.0, 0.0, 0.0, -180.0); }

  Serial.print("Pitch: "); Serial.print(pitch_angle); Serial.print(" | Roll: "); Serial.println(roll_angle);

  robothand.setOrientation(pitch_angle, roll_angle); //pitch, roll
  robothand.setClaw(buttonPressed);

  // ACT
  robothand.updateActuators();

  long endTime = millis();
  long delTime = endTime - startTime;

  // Serial.println(delTime);
}