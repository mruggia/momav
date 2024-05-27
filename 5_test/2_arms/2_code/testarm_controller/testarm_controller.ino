#include <Arduino.h>
#include "ServoMS.h"
#include "ESC32.h"
#include "COBS.h"

///////////////////////////////////////////////////////////////////////////////////////////////

//## servo control
ServoMS servo(9, 10, &Wire);

//## esc control
ESC32 esc(2, &Serial1);

///////////////////////////////////////////////////////////////////////////////////////////////

//## send packet
struct send_packet_struct {
  unsigned long com_ts;
  float srv_pos;
  float srv_setp;
  float srv_volt;
  float esc_setp;
  float esc_volt;
  float esc_curr;
  float esc_rpm;
} send_packet;
uint8_t send_packet_enc[sizeof(send_packet_struct)+2];
long send_freq;

//## receive packet
#define CMD_SRV_SETP 0x01
#define CMD_ESC_SETP 0x02
struct recv_packet_struct {
  uint8_t command;
  uint8_t data[4];
} recv_packet;
uint8_t recv_buffer_enc[sizeof(recv_packet_struct)+2];
int recv_packet_pos = 0;

///////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  /*//tuning config
  long   com_freq = 10e3;   //frequency with which data is reported to master (us)
  long   com_baud = 2e6;    //baud rate of master communication
  long   enc_filt = 20;     //servo encoder filter factor (%)
  double enc_offs = 5.48;   //servo encoder offset to zero position (0-2pi)
  long   pid_freq = 1e3;    //servo pid loop frequency (us)
  double pid_kp  = 12.0;    //servo proportional gain (on measurement, not on error!)
  double pid_ki  = 360.0;   //servo integral gain
  double pid_kd  = 0.08;    //servo derivative gain
  double pid_lim = 2.5;     //servo integrator limit (output capped to 1.0)
  long   esc_freq = 2e3;    //esc throttle update frequency (us)
  long   esc_telem = 10e3;  //esc telemetry update frequency (us)
  long   esc_poles = 14;    //esc motor nr of poles*/

  //tuning config
  long   com_freq = 10e3;   //frequency with which data is reported to master (us)
  long   com_baud = 2e6;    //baud rate of master communication
  long   enc_filt = 20;     //servo encoder filter factor (%)
  double enc_offs = 0.0;    //servo encoder offset to zero position (0-2pi)
  long   pid_freq = 1e3;    //servo pid loop frequency (us)
  double pid_kp  = 7.6;     //servo proportional gain (on measurement, not on error!)
  double pid_ki  = 220.0;   //servo integral gain
  double pid_kd  = 0.05;    //servo derivative gain
  double pid_lim = 1.8;     //servo integrator limit (output capped to 1.0)
  long   esc_freq = 2e3;    //esc throttle update frequency (us)
  long   esc_telem = 10e3;  //esc telemetry update frequency (us)
  long   esc_poles = 14;    //esc motor nr of poles


  //init serial connection
  Serial.begin(com_baud);
  send_freq = com_freq;
  
  //init servo
  servo.init_pid(pid_freq, pid_kp, pid_ki, pid_kd, pid_lim);
  servo.init_enc(enc_filt, enc_offs);

  //init esc
  esc.init(esc_freq, esc_telem, esc_poles);
  
}


void loop() {
  
  servo.run();
  esc.run();

  send_master();
  recv_master();
  
}

///////////////////////////////////////////////////////////////////////////////////////////////
// send packet to master
void send_master() {
  static unsigned long us_last = micros();
  unsigned long us_now = micros();

  if (us_now - us_last >= send_freq) {
    send_packet.com_ts   = us_now;
    send_packet.srv_pos  = servo.get_pid_in();
    send_packet.srv_setp = servo.get_pid_setp();
    send_packet.srv_volt = servo.get_pid_out();
    send_packet.esc_setp = esc.get_throt();
    send_packet.esc_volt = ((double)esc.get_volt())/100.0;
    send_packet.esc_curr = ((double)esc.get_curr())/100.0;
    send_packet.esc_rpm  = ((double)esc.get_rpm());
    
    cobs_encode((uint8_t*)&send_packet, sizeof(send_packet), send_packet_enc);
    Serial.write(send_packet_enc, sizeof(send_packet_enc));
    
    us_last += send_freq;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////
// receive packet from master
void recv_master() {

  while (Serial.available() > 0) {
    uint8_t c = Serial.read();
    recv_buffer_enc[recv_packet_pos] = c;
    recv_packet_pos += 1;

    if      (c == 0x00 && recv_packet_pos != sizeof(recv_buffer_enc)) { recv_packet_pos = 0; return; }
    else if (c != 0x00 && recv_packet_pos == sizeof(recv_buffer_enc)) { recv_packet_pos = 0; return; }
    else if (c == 0x00 && recv_packet_pos == sizeof(recv_buffer_enc)) { recv_packet_pos = 0;

      size_t ret = cobs_decode(recv_buffer_enc, sizeof(recv_buffer_enc), (uint8_t*)&recv_packet);
      if ( ret == 0 ) { return; }
      
      switch(recv_packet.command) {
        case CMD_SRV_SETP:
          servo.set_pid_setp(*((float*)recv_packet.data));
          break;
        case CMD_ESC_SETP:
          esc.set_throt(*((float*)recv_packet.data));
          break;
      }
      
    }
  }
}
