#include <Arduino.h>
#include "ServoMS.h"

// constructor, initialize pins for encoder and motor h-bridge
ServoMS::ServoMS(byte _pin_out1, byte _pin_out2, TwoWire* _i2c) :
pin_out1(_pin_out1), pin_out2(_pin_out2), i2c(_i2c) {
  analogWriteResolution(12);
  pinMode(pin_out1, OUTPUT); digitalWrite(pin_out1, HIGH);
  pinMode(pin_out2, OUTPUT); digitalWrite(pin_out2, HIGH);
}

// set PID tuning parameters
void ServoMS::init_pid(unsigned long _pid_freq, double _pid_kp, double _pid_ki, double _pid_kd, double _pid_lim, bool _pid_pom) {
  
  pid_pom = _pid_pom;
  pid_lim = _pid_lim;
  pid_freq = _pid_freq;
  
  double pid_freq_sec = ((double)_pid_freq)/1e6;
  pid_kp = _pid_kp;
  pid_ki = _pid_ki * pid_freq_sec;
  pid_kd = _pid_kd / pid_freq_sec;
  
}

// set encoder parameters
void ServoMS::init_enc(long _enc_filt, double _enc_offs) {
  
  enc_filt = _enc_filt;
  enc_offs = _enc_offs/TWO_PI*65536.0;
  
  i2c->begin();
  i2c->setClock(1000000);

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
      
      int pid_pwm = 4096 - int(abs(pid_out*4096.0));
      pid_pwm = constrain(pid_pwm, 0, 4096);
      if (pid_out > 0.0) {
        analogWrite(pin_out1, pid_pwm);
        analogWrite(pin_out2, 4096);
      } else {
        analogWrite(pin_out1, 4096);
        analogWrite(pin_out2, pid_pwm);
      }

      time_last_pid += pid_freq;
   }
}


//compute encoder position
void ServoMS::run_enc() {
  const long ENC_RES = 65536L;  //resolution of one encoder revolution
  bool send_request = false;    //flag if a new measurement request should be sent
  long enc_rot_new, enc_prt_new, enc_tot_new, enc_diff; //new measurement data


  // deal with broken states
  if (Wire.available() > 2) { while( Wire.available() ) { Wire.read(); } }
  if (time_now - time_last_enc >= 500) { send_request = true; }

  // check if encoder data is available
  if (Wire.available() == 2) {
  
    // read raw encoder data
    enc_prt_new = ( ( Wire.read() << 8 ) | Wire.read() ) << 4;
    enc_prt_new += enc_offs;
  
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
  
    // set request new data flag
    send_request = true;
  }

  //send new data request to encoder
  if (send_request) {
    Wire.beginTransmission(0x36);
    Wire.write(0x0c);
    Wire.endTransmission();
    Wire.requestFrom(0x36, 2);
    time_last_enc = time_now;
  }

}


//compute pid control
void ServoMS::run_pid() {

  double pid_err = pid_setp - pid_in;       //input error
  double pid_din = (pid_in - pid_in_prev);  //input derivative

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
