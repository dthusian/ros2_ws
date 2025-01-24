#ifndef MC_H
#define MC_H

#include"avg.h"

#define DEADZONE 100

int mc_debug = 0;
const float M_PER_ENC_UNIT = 0.00015339807878856412; // 0.05*PI/1024

class MC {
  // Configuration
  int pin_enc;
  int pin_pwmf;
  int pin_pwmb;
  bool rev_enc;
  char name;

  // PID Parameters
  float kP;
  float kI;
  float kD;

  // State
  float target_vel;
  float current_vel;
  Avg avg_vel;
  float output;
  int last_enc_pos;
  unsigned long last_time;
  float last_err;
  Avg integral;

public:
  MC(int p_enc, int p_pwmf, int p_pwmb, float n_kP, float n_kI, float n_kD, char name, bool rev_enc)
  : pin_enc{p_enc}, pin_pwmf{p_pwmf}, pin_pwmb{p_pwmb}, rev_enc{rev_enc}, name{name},
    kP{n_kP}, kI{n_kI}, kD{n_kD},
    target_vel{0}, current_vel{0}, avg_vel{}, output{0}, last_enc_pos{0}, last_time{0}, last_err{0}, integral{}
  {
    pinMode(pin_enc, INPUT);
    pinMode(pin_pwmf, OUTPUT);
    pinMode(pin_pwmb, OUTPUT);
    reset();
  }

  void reset() {
    analogWrite(pin_pwmf, 0);
    analogWrite(pin_pwmb, 0);
    avg_vel.reset();
    current_vel = 0;
    integral.reset();
    target_vel = 0;
    last_err = 0;
    output = 0;
  }

  int update() {
    // compute encoder offset since last update
    int enc_pos = analogRead(pin_enc);
    int offset = enc_pos - last_enc_pos;
    if(offset > 512) { // handle rollover
      offset = offset - 1024;
    } else if(offset < -512) {
      offset = offset + 1024;
    }
    last_enc_pos = enc_pos;
    float offset_m = offset * M_PER_ENC_UNIT;
    if(rev_enc) {
      offset_m = -offset_m;
    }
    
    // compute velocity from encoder offset
    unsigned long time = millis();
    float time_offset = (float)(time - last_time);
    last_time = time;
    float vel_sample = 1000 * offset_m / time_offset;
    float err = target_vel - current_vel;
    current_vel = vel_sample;
    avg_vel.push(vel_sample);

    // compute PID
    float oP = err * kP;
    integral.push(err * time_offset);
    float oI = integral.sum() * kI;
    float oD = (last_err - err) * kD / time_offset;
    last_err = err;
    float output_correction = oP + oI + oD;

    // add to output
    if(isnan(output)) {
      output = 0.0;
    }
    output += output_correction;
    // output clamp
    float clamp = 250 - DEADZONE;
    if(output > clamp) {
      output = clamp;
    } else if(output < -clamp) {
      output = -clamp;
    }

    // debug
    if(mc_debug) {
      Serial.print("D");
      Serial.print(name);
      Serial.print("\tS");
      Serial.print(target_vel);
      Serial.print("\tE");
      Serial.print(err);
      Serial.print("\tV");
      Serial.print(current_vel);
      Serial.print("\tA");
      Serial.print(avg_vel.avg());
      Serial.print("\tC");
      Serial.print(output_correction);
      Serial.print("\tO");
      Serial.println(output);
    }
    
    // set output power
    if(output < 0) {
      analogWrite(pin_pwmf, 0);
      analogWrite(pin_pwmb, DEADZONE - (int)output);
    } else {
      analogWrite(pin_pwmf, DEADZONE + (int)output);
      analogWrite(pin_pwmb, 0);
    }

    return enc_pos;
  }

  void set_target_vel(float vel) {
    target_vel = vel;
  }

  void set_pid(float n_kP, float n_kI, float n_kD) {
    kP = n_kP;
    kI = n_kI;
    kD = n_kD;
  }
};

#endif