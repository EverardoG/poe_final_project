// #include "HumanHand.h"
#include "rgpt_lib.h"

HumanHand humanhand;
RobotHand robothand;

doubs fings;
int thumbStatus;
int pointerStatus;
int calButtonPressed;
sensors_vec_t handOrientation;
sensors_vec_t robotOrientation;

int pitch_angle;
int roll_angle;

int robot_pitch;
int robot_roll;

long prev_time = 0;
long cur_time = 0;
long loop_time = 20; // this is how fast our real time loop runs in milliseconds


int calButtonPin = 13;
bool calibration_done = false;

bool robot_hand_within_certain_bounds;

void setup(void)
{
  Serial.begin(115200);
  Serial.println("start");
  humanhand.init(12, A2, A1); // button_pin, thumb_pin, pointer_pin
  Serial.println("human hand ready");
  robothand.init(9, 8, 3, 2, 6, 5); // left_step_pin, left_direction_pin, right_step_pin, right_direction_pin, pinch_pin_1, pinch_pin_2
  Serial.println("robot hand ready");
  pinMode(calButtonPin, INPUT);
}

void loop(void)
{
//  Serial.println("running soft time loop");
  cur_time = millis();
  if ((cur_time - prev_time) >= loop_time) {
    prev_time = cur_time;
//    Serial.println("running real time loop");

    // SENSE
    humanhand.updateSensors();
    robothand.updateSensors();

    fings = humanhand.getFingerStatus();
    thumbStatus = fings.thumb;
    pointerStatus = fings.pointer;

    calButtonPressed = digitalRead(calButtonPin);

    handOrientation = humanhand.getHandOrientation();
    robotOrientation = robothand.getRobotOrientation();

    //Serial.print(handOrientation.pitch); Serial.print(" | "); Serial.println(handOrientation.roll);

    // THINK
    // @jasmine get rid of robot_hand_within_certain_bounds - this is just a placeholder to get this to compile
    robot_hand_within_certain_bounds = true;

    // @jasmine - This is basically what puts the robothand in calibration mode
    //Serial.println(calButtonPressed);
    if (calButtonPressed == 1) {
      // robothand.mode == 1 is calibration mode, 0 is mimic the human hand
      robothand.mode = 1;
    }

    // if the robohand is in calibration mode, continue calibrating

    if (robothand.mode == 1){
        Serial.println("calibrating");
        // @jasmine this is where the actual calibration code should go
        // set the pitch_angle to something based on robotOrientation
        // set the roll_angle to something based on robotOrientation
        pitch_angle = 0;
        roll_angle = 0;

        // @jasmine set some end condition here that'll break the robot hand out of calibration mode
        // it'll probably be something like once the robothand is within bounds of some angles, then stop
        // this is the if statement that'll break the robot hand out of calibration mode
        if (robot_hand_within_certain_bounds) {
          // this resets the robot hand's absolute position
          robothand.resetSteppers();

          // set the robothand back to mimic mode
          robothand.mode = 0;
        }

      }
      Serial.print(robot_pitch_angle);
      Serial.print(",");
      Serial.print(new_robot_pitch_angle);
      Serial.print(",");
      Serial.print(robotOrientation.pitch);
      Serial.print(",");
      if (robotOrientation.pitch > 90)
      {
        robot_roll_angle = 180 - robotOrientation.pitch;
      }
      else if (robotOrientation.pitch < -90)
      {
        robot_roll_angle = -180 - robotOrientation.pitch;
      }
      else
      {
        robot_roll_angle = robotOrientation.pitch;
      }


      Serial.println(robot_roll_angle);
      // @jasmine this is where the actual calibration code should go
      // set the pitch_angle to something based on robotOrientation
      // set the roll_angle to something based on robotOrientation
      //pitch_angle = 0;
      //roll_angle = 0;

      // @jasmine set some end condition here that'll break the robot hand out of calibration mode
      // it'll probably be something like once the robothand is within bounds of some angles, then stop
      // this is the if statement that'll break the robot hand out of calibration mode
      if (robot_hand_within_certain_bounds) {
        // this resets the robot hand's absolute position
        robothand.resetSteppers();

        // set the robothand back to mimic mode
        robothand.mode = 0;
      }
    }
    else {
      //Serial.println("mimicing human hand");
      //Serial.print(handOrientation.pitch); Serial.print(" | "); Serial.println(handOrientation.roll);
//      if (handOrientation.pitch >= 0) { pitch_angle = robothand.remap(handOrientation.pitch, 0.0, -180.0, 0.0, 180.0); }
//      else { pitch_angle = robothand.remap(handOrientation.pitch, 0.0, 180.0, 0.0, -180.0); }

      pitch_angle = handOrientation.pitch;

      if (handOrientation.roll >= 0) { roll_angle = robothand.remap(handOrientation.roll, 180.0, 0.0, 0.0, 180.0); }
      else { roll_angle = robothand.remap(handOrientation.roll, -180.0, 0.0, 0.0, -180.0); }
      Serial.print(pitch_angle); Serial.print(" | "); Serial.println(roll_angle);
    }

    //Serial.print(pitch_angle); Serial.print(" | "); Serial.println(roll_angle);

    robothand.setOrientation(pitch_angle, roll_angle); //pitch, roll
//    Serial.print("Thumb"); Serial.println(thumbStatus);
//    Serial.print("Pointer"); Serial.println(pointerStatus);
    robothand.setClaw(pointerStatus, thumbStatus);

    // ACT
    robothand.updateActuators();
  }
}
