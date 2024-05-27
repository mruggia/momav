#include <Arduino.h>
#include "ESC32.h"

// DSHOT timing targets: https://www.speedgoat.com/products/dshot
// further reading: http://reefwingrobotics.blogspot.com/2020/05/arduino-esc-tester-adding-dshot.html
// PX4 DSHOT implementation: https://github.com/PX4/PX4-Autopilot/tree/98a9748bb8939c450d4beb0eecc67c79b6bbed6d/src/drivers/dshot

// delay macro for waiting N number of clock cycles
#define DELAY(N) __asm__ __volatile__ (NOP ## N)
#define NOP1 "nop\n"
#define NOP2  NOP1  NOP1
#define NOP3  NOP2  NOP1
#define NOP4  NOP3  NOP1
#define NOP5  NOP4  NOP1
#define NOP6  NOP5  NOP1
#define NOP7  NOP6  NOP1
#define NOP8  NOP7  NOP1
#define NOP9  NOP8  NOP1
#define NOP10 NOP9  NOP1
#define NOP11 NOP10 NOP1
#define NOP12 NOP11 NOP1
#define NOP13 NOP12 NOP1
#define NOP14 NOP13 NOP1
#define NOP15 NOP14 NOP1
#define NOP16 NOP15 NOP1
#define NOP17 NOP16 NOP1
#define NOP18 NOP17 NOP1
#define NOP19 NOP18 NOP1
#define NOP20 NOP19 NOP1
#define NOP21 NOP20 NOP1
#define NOP22 NOP21 NOP1
#define NOP23 NOP22 NOP1
#define NOP24 NOP23 NOP1
#define NOP25 NOP24 NOP1
#define NOP26 NOP25 NOP1
#define NOP27 NOP26 NOP1
#define NOP28 NOP27 NOP1
#define NOP29 NOP28 NOP1
#define NOP30 NOP29 NOP1
#define NOP31 NOP30 NOP1
#define NOP32 NOP31 NOP1
#define NOP33 NOP32 NOP1
#define NOP34 NOP33 NOP1
#define NOP35 NOP34 NOP1
#define NOP36 NOP35 NOP1
#define NOP37 NOP36 NOP1
#define NOP38 NOP37 NOP1
#define NOP39 NOP38 NOP1
#define NOP40 NOP39 NOP1

//macros with hardcoded delays for DSHOT600 + SAMD21 @48MHz
#define DELAY_T0H DELAY(12)
#define DELAY_T0L DELAY(21)
#define DELAY_T1H DELAY(38)
#define DELAY_T1L DELAY(1)

//macro to send one DShot bit
#define DSHOT0() *setDShot = maskDShot; DELAY_T0H; *clrDShot = maskDShot; DELAY_T0L;
#define DSHOT1() *setDShot = maskDShot; DELAY_T1H; *clrDShot = maskDShot; DELAY_T1L;

// ######################################################################################################

// constructor, configure dshot pin
ESC32::ESC32(byte _pinDShot, Uart* _telem, int _poles) :
pinDShot(_pinDShot), telem(_telem), poles(_poles),
setDShot(&PORT->Group[g_APinDescription[_pinDShot].ulPort].OUTSET.reg), 
clrDShot(&PORT->Group[g_APinDescription[_pinDShot].ulPort].OUTCLR.reg),
maskDShot(1ul << g_APinDescription[_pinDShot].ulPin)
{
  pinMode(_pinDShot, OUTPUT);
}

// ######################################################################################################

// send dshot throttle message (0-1)
void ESC32::send_throt(bool _req_telem) {
  
  int throt_int = ((int)(throt*2000))+48;     // get throttle as integer for dshot (range 48-2047)
  throt_int = constrain(throt_int, 48, 2047); // clip to valid range 48-2047
  if(throt_int==48 && counter%(10*60*500)==0) { throt_int=49; } // if throtle at 0 and 10min have passed, set throtle to min to avoid timeout
  uint16_t data = throt_int << 1;             // add throttle to dshot message
  if (_req_telem) { data |= 1; }              // add telemetry request flag if wanted
  
  send_dshot( data );  //send dshot message
  counter++;
}

//send a generic dshot message
void ESC32::send_dshot(uint16_t data) {
  
  //calculate checksum
  uint16_t csum, data_cpy = data;
  for (byte i=0; i<3; i++){
    csum ^= data_cpy;
    data_cpy >>= 4;
  }
  csum &= 0xF;
  
  //combine data and checksum to message
  uint16_t msg = (data<<4)|csum;

  //send message to esc
  noInterrupts();
  for (int i = 15; i >= 0; i--) {
    if (msg & (1<<i)) { DSHOT1(); }
    else              { DSHOT0(); }
  }
  interrupts();
  
}

// ######################################################################################################

// read esc telemetry
bool ESC32::read_telem() { 
  if ( telem->available() != 10 ) { //check if message size is correct
    while( telem->available() ) { telem->read(); }
    return false; 
  }

  //read message
  uint8_t data[10];
  for (int i=0; i<10; i++) {
    data[i] = telem->read();
  }
  
  //decode message
  temp = data[0];
  volt = (data[1] << 8) | data[2];
  curr = (data[3] << 8) | data[4];
  cons = (data[5]) << 8 | data[6];
  rpm = ((data[7] << 8) | data[8])*100*2/poles;
  
  return true;
}
