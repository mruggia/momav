#include <Arduino.h>
#include "watchdog.h"
#include "ServoMS.h"

// constructor, initialize pins for encoder and motor h-bridge
ServoMS::ServoMS(byte _pin_out1, byte _pin_out2, TwoWire* _i2c) :
pin_out1(_pin_out1), pin_out2(_pin_out2), i2c(_i2c) {
  analogWriteResolution(12);
  pinMode(pin_out1, OUTPUT); analogWrite(pin_out1, 4095);
  pinMode(pin_out2, OUTPUT); analogWrite(pin_out2, 4095);
}

// set PID tuning parameters
void ServoMS::init_pid(unsigned long _pid_freq, float _pid_kp, float _pid_ki, float _pid_kd, float _pid_lim, bool _pid_pom) {
  
  pid_pom = _pid_pom;
  pid_lim = _pid_lim;
  pid_freq = _pid_freq;
  
  float pid_freq_sec = ((float)_pid_freq)/1e6;
  pid_kp = _pid_kp;
  pid_ki = _pid_ki * pid_freq_sec;
  pid_kd = _pid_kd / pid_freq_sec;
  
}

// set encoder parameters
void ServoMS::init_enc(long _enc_freq, long _enc_filt) {

  enc_freq = _enc_freq;
  enc_filt = _enc_filt;
  
  //i2c->begin();
  //i2c->setClock(1000000);

}

// ######################################################################################################


// run servo control (must be executed regularly)
void ServoMS::run() {
  time_now = micros();
  if(!time_last_pid) { time_last_pid = time_now; }
  if(!time_last_enc) { time_last_enc = time_now; }

  run_enc();

  if (time_now - time_last_pid >= pid_freq) {
    
    pid_in = ((float)enc_tot)/65536.0*TWO_PI;
    run_pid();
    
    int pid_pwm = 4095 - int(abs(pid_out*4095.0));
    pid_pwm = constrain(pid_pwm, 0, 4095);
    if (pid_out > 0.0) {
      analogWrite(pin_out1, pid_pwm);
      analogWrite(pin_out2, 4095);
    } else {
      analogWrite(pin_out1, 4095);
      analogWrite(pin_out2, pid_pwm);
    }
    
    time_last_pid += pid_freq;
  }

  //check for delays in execution
  watchdog_clean_timer(time_now, time_last_pid, pid_freq);   
}


//compute encoder position
void ServoMS::run_enc() {
  const long ENC_RES = 65536L;  //resolution of one encoder revolution
  long enc_rot_new, enc_prt_new, enc_tot_new, enc_diff; //new measurement data

  // check if encoder data is available
  if (i2c->available() == 2) {
  
    // read raw encoder data
    enc_prt_new = ( ( i2c->read() << 8 ) | i2c->read() ) << 4;
  
    // find full rotation count that brings measurement closest to last
    enc_rot_new = enc_rot;
    enc_diff = enc_prt_new - enc_prt;
    if (enc_diff >  ENC_RES/2) { enc_rot_new -= 1; }
    if (enc_diff < -ENC_RES/2) { enc_rot_new += 1; }
  
    // calculate new rotation and filter it
    enc_tot_new = enc_rot_new*ENC_RES + enc_prt_new;
    enc_tot_new = ( enc_tot_new*(enc_filt) + enc_tot*(100-enc_filt) )/100;
  
    // save results
    enc_rot = enc_rot_new;
    enc_prt = enc_prt_new;
    enc_tot = enc_tot_new;

    // set flag that encoder responded
    enc_resp = true;
  }

  // deal with broken states
  if (i2c->available() > 2) { watchdog_throw(); while( i2c->available() ) { i2c->read(); } }

  //send new data request to encoder
  if (time_now - time_last_enc >= enc_freq) {

    if (!enc_resp) { watchdog_throw(); }
    
    i2c->beginTransmission(0x36);
    i2c->write(0x0c);
    i2c->endTransmission();
    i2c->requestFrom(0x36, 2);
    
    time_last_enc += enc_freq;
    enc_resp = false;
    
  }

  //check for delays in execution
  watchdog_clean_timer(time_now, time_last_enc, enc_freq);
}


//compute pid control
void ServoMS::run_pid() {

  float pid_err = pid_setp - pid_in;       //input error
  float pid_din = (pid_in - pid_in_prev);  //input derivative

  // calculate output integra term
  pid_out_sum += (pid_ki * pid_err);                        // add current term
  if(pid_pom) pid_out_sum -= pid_kp * pid_din;              // add kp on measurement if specified
  pid_out_sum = constrain(pid_out_sum, -pid_lim, pid_lim);  // cap term to output limits

  // calculate rest of output term
  pid_out = pid_out_sum - pid_kd * pid_din;  // calculate ki and kd terms
  if(!pid_pom) pid_out = pid_kp * pid_err;   // add kp on error if specified
  pid_out = constrain(pid_out, -1.0, 1.0);   //cap term to output limits

  //remember for later
  pid_in_prev = pid_in;
  
}
