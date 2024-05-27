#ifndef ESC32_h
#define ESC32_h

// ###########################
// Library for control of BLHeli_32 using DShot600 ESCs
// DShot600 implementation uses bit-banging with hardcoded timings. it only works on SAMD21 @48MHz
// ###########################

class ESC32 {
public:

  // constructor, configure dshot pin
  ESC32(byte _pinDShot, Uart* _telem);
  // init function, start serial & wait for esc to boot
  void init(long _throttle_freq, long _telem_freq, int _poles);

  // main loop that sets throttle and reads telemetry
  void run();

  //interface
  int8_t  get_temp() { return temp; }
  int16_t get_volt() { return volt; }
  int16_t get_curr() { return curr; }
  int16_t get_cons() { return cons; }
  int16_t get_rpm()  { return rpm; }
  double  get_throt(){ return throt; }
  void set_throt(double _throt) { throt = _throt; }

private:

  //esc telemetry
  int8_t  temp; // [deg C]
  int16_t volt; // [0.01V]
  int16_t curr; // [0.01A]
  int16_t cons; // [mAh]
  int16_t rpm;  // [rpm]

  //esc input
  double throt = 0.0;

  //timing
  long throt_freq; //update frequency of throttle (us)
  long telem_freq;    //update frequency of telemetry (us)
  unsigned long time_now = micros();  //current time
  unsigned long time_last_throt = 0;  //last time throttle command was sent
  unsigned long time_last_telem = 0;  //last time telem request was sent

  //dshot
  const byte pinDShot;          //dshot pin nr
  volatile uint32_t *setDShot;  //dshot pin set register
  volatile uint32_t *clrDShot;  //dshot pin clear register
  const uint32_t  maskDShot;    //dshot pin mask

  //telemetry
  Uart* telem;
  int poles;

  // send dshot throttle message (0-1)
  void send_throt(double _throt, bool _req_telem);
  
  //send a generic dshot message
  void send_dshot(uint16_t data);

  // read esc telemetry
  void read_telem();
  
};



#endif
