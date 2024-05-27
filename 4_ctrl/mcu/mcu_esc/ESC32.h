#ifndef ESC32_h
#define ESC32_h

// ###########################
// Library for control of BLHeli_32 using DShot600 ESCs
// DShot600 implementation uses bit-banging with hardcoded timings. it only works on SAMD21 @48MHz
// ###########################

class ESC32 {
public:

  // constructor, configure dshot pin
  ESC32(byte _pinDShot, Uart* _telem, int _poles);

  // read telemetry
  bool read_telem();
  // send dshot throttle message (0-1)
  void send_throt(bool _req_telem);

  //esc telemetry
  int8_t  temp; // [deg C]
  int16_t volt; // [0.01V]
  int16_t curr; // [0.01A]
  int16_t cons; // [mAh]
  int16_t rpm;  // [rpm]

  //esc input
  float throt = 0.0;

  //dshot
  const byte pinDShot;          //dshot pin nr
  volatile uint32_t *setDShot;  //dshot pin set register
  volatile uint32_t *clrDShot;  //dshot pin clear register
  const uint32_t  maskDShot;    //dshot pin mask
  int poles;                    //nr of poles for rpm calculation
  unsigned long counter=0;      //counter to add throttle jitter that prevents timeouts
  
  //telemetry
  Uart* telem;
  
  //send a generic dshot message
  void send_dshot(uint16_t data);
  
};



#endif
