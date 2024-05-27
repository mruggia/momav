#ifndef ServoMS_h
#define ServoMS_h

#include <Wire.h>

// ###########################
// Library for external closed loop control of KST MS series servos.
// enables continuous rotation with position control!
// requires the following hardware modifications:
// * remove original control board, exposine magnetic encoder IC legs
// * connect I2C + power leads of encoder (AS5600) to MCU (for SAMD21 add 330Ohm pullups)
// * connect motor leads to some mosfet h-bridge and its input to 2 PWM pins
// ###########################


class ServoMS {
public:

  // constructor, initialize pins for encoder and motor h-bridge
  ServoMS(byte _pin_out1, byte _pin_out2, TwoWire* _i2c);

  // set PID tuning parameters
  void init_pid(unsigned long _pid_freq, double _pid_kp, double _pid_ki, double _pid_kd, double _pid_lim, bool _pid_pom = true);
  // set encoder parameters
  void init_enc(long _enc_filt, double _enc_offs);

  // run servo control (must be executed regularly)
  void run();

  // interface
  double get_pid_in()   { return -pid_in; }
  double get_pid_out()  { return -pid_out; }
  double get_pid_setp() { return -pid_setp; }
  void set_pid_setp(double _pid_setp) { pid_setp = -_pid_setp; }
  
private:

  void run_enc();    //compute encoder position
  void run_pid();    //compute pid control
  
  //com
  const byte pin_out1;   // pin for h-bridge output 1
  const byte pin_out2;   // pin for h-bridge output 2
  TwoWire* i2c;          //i2c object to communicate with encoder

  //pid parameters
  double pid_kp = 0.0;      // PID tuning parameters
  double pid_ki = 0.0;      //
  double pid_kd = 0.0;      //
  bool   pid_pom = true;    // true: proportional on measurement (PoM), false: proportional on error (PoE)
  double pid_lim = 1.0;     // integrator limit
  long   pid_freq = 1e3;    // pid loop frequency
  //pid data
  double pid_in = 0.0;      // input
  double pid_out = 0.0;     // output
  double pid_setp = 0.0;    // setpoint
  double pid_in_prev = 0.0; // input from previous iteration
  double pid_out_sum = 0.0; // sum of all outputs

  //encoder parameters
  long enc_offs = 0;        // encoder offset position from upright
  long enc_filt = 0;        // encoder filtering constant
  //encoder data
  long enc_rot = 0;         //full rotations of last measurement
  long enc_prt = 0;         //angle of last measurement (not including full rotations)
  long enc_tot = 0;         //angle of last measurement (including full rotations)
  
  //timing
  unsigned long time_now = micros(); //current time
  unsigned long time_last_pid = 0;   //last time the pid loop was run
  unsigned long time_last_enc = 0;   //last time the encoder provided a value
  
};

#endif
