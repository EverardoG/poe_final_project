// Servo - Version: Latest 
#include <Servo.h>

/*

*/

int thumb_pin = A3;
int pointer_pin = A5;
int thumb_input;
int pointer_input;
double desired;
int _time;
boolean _on = true;
Servo fing_test;

void blink();
int convert(int input, int finger);

void setup() {
  Serial.begin(9600);   
  fing_test.attach(9);
}

void loop() {
  thumb_input = analogRead(thumb_pin);
  Serial.print("Thumb "); Serial.println(thumb_input);
 
  pointer_input = analogRead(pointer_pin);
  //Serial.print("Pointer "); Serial.println(pointer_input);
  
  desired = convert(thumb_input, 0);
  Serial.print("Desired "); Serial.println(desired);
  fing_test.write(desired);
  
  blink();
}

void blink(){
  if(_on = true){
    digitalWrite(13, HIGH);
    _time = millis();
    _on = false;
  } 
  if(millis() > _time + 50){
    _on = true;
  }
}

int convert(int input, int finger = 1){
  double y_int;
  int factor;
  if(finger == 1){
    y_int = .75;
    factor = 1;
  } else{
    y_int = .5;
    factor = 2;
  }
  double fraction = y_int - (double(input)/700) * factor;
  Serial.println(fraction);
  int angle = int(fraction * 180);
  return angle;
}
