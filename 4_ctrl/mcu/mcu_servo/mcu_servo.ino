#include <Arduino.h>
#include <Wire.h>
#include "wiring_private.h"
#include "watchdog.h"
#include "ServoMS.h"

#define I2C_ADDR 0x10 //0x10, 0x11 or 0x12

TwoWire Wire1(&sercom2, 4, 3);
TwoWire Wire2(&sercom1, 11, 13);
void SERCOM2_Handler(void) { Wire1.onService(); }

ServoMS servo1(10, 6, &Wire);
ServoMS servo2( 8, 9, &Wire2);

void setup() {

  //tuning config
  long  enc_freq = 250;    //encoder update frequency (us)
  long  enc_filt = 20;     //servo encoder filter factor (%)
  long  pid_freq = 1e3;    //servo pid loop frequency (us)
  float pid_kp  = 7.6;     //servo proportional gain (on measurement, not on error!)
  float pid_ki  = 220.0;   //servo integral gain
  float pid_kd  = 0.05;    //servo derivative gain
  float pid_lim = 1.8;     //servo integrator limit (output capped to 1.0)

  //init usb serial connection
  SerialUSB.begin(2e6);
  
  //init watchdog for rt monitoring
  watchdog_init();
  
  //init i2c
  Wire.begin();
  Wire.setClock(1000000);
  Wire2.begin();
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);
  Wire2.setClock(1000000);
  Wire1.begin(I2C_ADDR);
  pinPeripheral(4, PIO_SERCOM_ALT);
  pinPeripheral(3, PIO_SERCOM_ALT);
  Wire1.onRequest(requestEvent);
  Wire1.onReceive(receiveEvent);

  //init servos
  servo1.init_pid(pid_freq, pid_kp, pid_ki, pid_kd, pid_lim);
  servo1.init_enc(enc_freq, enc_filt);
  servo2.init_pid(pid_freq, pid_kp, pid_ki, pid_kd, pid_lim);
  servo2.init_enc(enc_freq, enc_filt);
  
}

void loop() {

  servo1.run();
  servo2.run();
  watchdog_run();
    
}

//receive i2c data
void receiveEvent(int count) {
  if (count != 8) {
    for (int i=0; i<count; i++) { Wire.read(); } 
    return; 
  }
  
  static union {
    byte c[8];
    struct {
      float pid_setp1;
      float pid_setp2;
    } d;
  } rcv;
  
  for (int i=0; i<8; i++) { rcv.c[i] = Wire1.read(); }
  servo1.set_pid_setp(rcv.d.pid_setp1);
  servo2.set_pid_setp(rcv.d.pid_setp2);
  
}

//send i2c data
void requestEvent() {

  static union {
    byte c[20];
    struct {
      float pid_in1;
      float pid_out1;
      float pid_in2;
      float pid_out2;
      bool watchdog;
    } d;
  } snd;

  snd.d.pid_in1 = servo1.get_pid_in();
  snd.d.pid_out1 = servo1.get_pid_out();
  snd.d.pid_in2 = servo2.get_pid_in();
  snd.d.pid_out2 = servo2.get_pid_out();
  snd.d.watchdog = watchdog_status();
  Wire1.write(snd.c, 20);
  
}
