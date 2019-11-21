// #include "HumanHand.h"
#include "rgpt_lib.h"

HumanHand humanhand;
RobotHand robothand;

int fingerStatus;
int calButtonPressed;
sensors_vec_t handOrientation;
sensors_vec_t robotOrientation;

float pitch_angle;
float roll_angle;

float robot_pitch;
float robot_roll;

long prev_time = 0;
long cur_time = 0;
long loop_time = 20; // this is how fast our real time loop runs in milliseconds

int calButtonPin = 7;
bool calibration_done = false;

void setup(void)
{
  Serial.begin(115200);
  Serial.println("start");
  humanhand.init(4, A2, A2); // button_pin, thumb_pin, pointer_pin
  Serial.println("human hand ready");
  robothand.init(9, 8, 3, 2, 10, 11); // left_step_pin, left_direction_pin, right_step_pin, right_direction_pin, pinch_pin_1, pinch_pin_2
  Serial.println("robot hand ready");
  pinMode(calButtonPin, INPUT);
}

void loop(void)
{
  cur_time = millis();
  if ((cur_time - prev_time) >= loop_time) {
    prev_time = cur_time;
    // SENSE
    humanhand.updateSensors();
    robothand.updateSensors();

    pointerStatus, thumbStatus = humanhand.getFingerStatus();
    calButtonPressed = digitalRead(calButtonPin);

    handOrientation = humanhand.getHandOrientation();
    robotOrientation = robothand.getRobotOrientation();

    // THINK
    // @jasmine get rid of robot_hand_within_certain_bounds - this is just a placeholder to get this to compile
    bool robot_hand_within_certain_bounds = true;

    // @jasmine - This is basically what puts the robothand in calibration mode
    if (calButtonPressed == 1){
      // robothand.mode == 1 is calibration mode, 0 is mimic the human hand
      robothand.mode = 1;
    }

    // if the robohand is in calibration mode, continue calibrating
    if (robothand.mode == 1){
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
    else {
      if (handOrientation.pitch >= 0) { pitch_angle = robothand.remap(handOrientation.pitch, 0.0, -180.0, 0.0, 180.0); }
      else { pitch_angle = robothand.remap(handOrientation.pitch, 0.0, 180.0, 0.0, -180.0); }

      if (handOrientation.roll >= 0) { roll_angle = robothand.remap(handOrientation.roll, 180.0, 0.0, 0.0, 180.0); }
      else { roll_angle = robothand.remap(handOrientation.roll, -180.0, 0.0, 0.0, -180.0); }
    }

    robothand.setOrientation(pitch_angle, roll_angle); //pitch, roll
    robothand.setClaw(pointerStatus, thumbStatus);

    // ACT
    robothand.updateActuators();
  }
}
