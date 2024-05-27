#include <Arduino.h>
#include <Wire.h>
#include "watchdog.h"
#include "ESC32.h"

#define I2C_ADDR 0x09
#define NUM_POLES 14

//new sercom uart
Uart Serial2(&sercom1, 12, 10, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM1_Handler() { Serial2.IrqHandler(); }

//escs group 1
ESC32 esc1(2, &Serial1, NUM_POLES);
ESC32 esc2(3, &Serial1, NUM_POLES);
ESC32 esc3(4, &Serial1, NUM_POLES);
ESC32 esc4(5, &Serial1, NUM_POLES);

//esc group 2
ESC32 esc5(6, &Serial2, NUM_POLES);
ESC32 esc6(7, &Serial2, NUM_POLES);
ESC32 esc7(8, &Serial2, NUM_POLES);
ESC32 esc8(9, &Serial2, NUM_POLES);

//esc to motor mapping
ESC32* motors[] = { &esc1, &esc2, &esc3, &esc5, &esc6, &esc7};
int timeouts[] = { 0, 0, 0, 0, 0, 0 };

//parameters
unsigned long esc_freq = 2e3;     //esc throttle update frequency (us)
unsigned long i2c_freq = 250e3;   //i2c minimum update frequency (us)
unsigned long time_last_i2c = 0;  //timestamp of last i2c update

void setup() {
 
  //init serial connection
  SerialUSB.begin(2e6);
  Serial1.begin(115200);
  Serial2.begin(115200);
  
  //init watchdog for rt monitoring
  watchdog_init();

  //init i2c
  Wire.begin(I2C_ADDR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  
}


void loop() {
  unsigned long time_now = micros();
  static unsigned long time_last_esc = micros();
  static int telem_group1 = 0, telem_group2 = 3;
  
  if (time_now - time_last_esc >= esc_freq) {
    time_last_esc += esc_freq;
    
    //read telem from last request
    motors[telem_group1]->read_telem();
    motors[telem_group2]->read_telem();
    
    //update to new telem request
    telem_group1++; if (telem_group1==3) { telem_group1=0; }
    telem_group2++; if (telem_group2==6) { telem_group2=3; }

    //send throttle and request telem (if last i2c command is not to old)
    if (time_now - (time_last_i2c-1e3) < i2c_freq) {
      for(int i=0; i<6; i++) { 
        if (i==telem_group1 || i==telem_group2) {
          motors[i]->send_throt(true);
        } else {
          motors[i]->send_throt(false); 
        }
      }
    }

  }

  watchdog_clean_timer(time_now, time_last_esc, esc_freq);
  watchdog_run();
}

//receive i2c data
void receiveEvent(int count) {
  if (count != 24) { 
    for (int i=0; i<count; i++) { Wire.read(); } 
    return; 
  }
  
  time_last_i2c = micros();
  
  static union {
    byte c[24];
    float d[6];
  } rcv;
  
  for (int i=0; i<24; i++) { rcv.c[i] = Wire.read(); }
  for ( int i=0; i<6; i++ ) { motors[i]->throt = rcv.d[i]; }
  
}

//send i2c data
void requestEvent() {

  static union {
    byte c[56];
    struct {
      float volt;
      float curr[6];
      float rpm[6];
      bool watchdog;
    } d;
  } snd;

  snd.d.volt = 0.0;
  for ( int i=0; i<6; i++ ) { 
    snd.d.volt += ((float)motors[i]->volt)/100.0;
    snd.d.curr[i] = ((float)motors[i]->curr)/100.0;
    snd.d.rpm[i] = ((float)motors[i]->rpm);
  }
  snd.d.volt = snd.d.volt / 6.0;
  snd.d.watchdog = watchdog_status();
  Wire.write(snd.c, 56);
  
}
