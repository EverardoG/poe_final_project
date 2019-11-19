// #include "HumanHand.h"
#include "rgpt_lib.h"

HumanHand humanhand;
RobotHand robothand;

int buttonPressed;
// HUMAND_HAND::sensors_vec_t handOrientation;

float pitch_angle;
float roll_angle;

float robot_pitch;
float robot_roll;

long prev_time = 0;
long cur_time = 0;
long loop_time = 20; // this is how fast our real time loop runs in milliseconds

void setup(void)
{
  Serial.begin(115200);
  Serial.println("start");
  humanhand.init(4); // initializes hand with button pin at pin 2
  Serial.println("human hand ready");
  robothand.init(9, 8, 3, 2, 11); // left_step_pin, left_direction_pin, right_step_pin, right_direction_pin, pinch_pin
  Serial.println("robot hand ready");
}

void loop(void)
{
//  Serial.println("main loop running");
//  Serial.println(cur_time);
//  Serial.println(prev_time);
//  Serial.println(loop_time);

  cur_time = millis();
//  Serial.println(cur/_time);
  if ((cur_time - prev_time) >= loop_time) {
//    Serial.println("/real time loop running");
    prev_time = cur_time;
    // SENSE
    humanhand.updateSensors();
//    Serial.println("human /hand updated");
    robothand.updateSensors();
//    Serial.println("robot han/d updated");
    buttonPressed = humanhand.getFingerStatus();
    sensors_vec_t handOrientation = humanhand.getHandOrientation();

    // THINK
    if (handOrientation.pitch >= 0) { pitch_angle = robothand.remap(handOrientation.pitch, 0.0, -180.0, 0.0, 180.0); }
    else { pitch_angle = robothand.remap(handOrientation.pitch, 0.0, 180.0, 0.0, -180.0); }

    if (handOrientation.roll >= 0) { roll_angle = robothand.remap(handOrientation.roll, 180.0, 0.0, 0.0, 180.0); }
    else { roll_angle = robothand.remap(handOrientation.roll, -180.0, 0.0, 0.0, -180.0); }

//    Serial.print("/Human Pitch: "); Serial.print(pitch_angle); Serial.print(" | Human Roll: "); Serial.println(roll_angle);

    robothand.setOrientation(pitch_angle, roll_angle); //pitch, roll
//    Serial.println(buttonPresse/d);
    robothand.setClaw(buttonPressed);

    robot_pitch = robothand.filter.getPitch();
    robot_roll = robothand.filter.getRoll();

    Serial.println(robot_pitch); //Serial.print(" | "); Serial.println(robot_roll);

    // ACT
    robothand.updateActuators();
    // Serial.println(delTime);
  }
}
