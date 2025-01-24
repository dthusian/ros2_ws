
#include"mc.h"

const int PIN_ENC_L = A1;
const int PIN_ENC_R = A0;
const int PIN_PWM_FL = 7;
const int PIN_PWM_BL = 6;
const int PIN_PWM_FR = 5;
const int PIN_PWM_BR = 4;

// default values
const float kP = 0.0;
const float kI = 0.0;
const float kD = 0.0;

MC left = MC(PIN_ENC_L, PIN_PWM_FL, PIN_PWM_BL, kP, kI, kD, 'l', false);
MC right = MC(PIN_ENC_R, PIN_PWM_FR, PIN_PWM_BR, kP, kI, kD, 'r', true);

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  while(!Serial);
}

void loop() {
  while(Serial.available() > 0) {
    int cmd = Serial.read();
    if(cmd == 'B') {
      float left_vel = Serial.parseFloat();
      Serial.read();
      float right_vel = Serial.parseFloat();
      while(Serial.read() != '\n');
      left.set_target_vel(left_vel);
      right.set_target_vel(right_vel);
    } else if(cmd == 'C') {
      float kP = Serial.parseFloat();
      Serial.read();
      float kI = Serial.parseFloat();
      Serial.read();
      float kD = Serial.parseFloat();
      while(Serial.read() != '\n');
      left.set_pid(kP, kI, kD);
      right.set_pid(kP, kI, kD);
    } else if(cmd == 'D') {
      mc_debug = 1;
      while(Serial.read() != '\n');
    }
  }
  
  // emergency-stop if it ever gets disconnected
  while(!Serial) {
    left.reset();
    right.reset();
  }

  // update and send encoder data
  int left_pos = left.update();
  int right_pos = right.update();
  Serial.print("A");
  Serial.print(left_pos);
  Serial.print(",");
  Serial.println(right_pos);
  delay(50);
}


