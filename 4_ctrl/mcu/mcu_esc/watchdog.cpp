
#include <Arduino.h>
#include "watchdog.h"

unsigned long watchdog_last;
unsigned long watchdog_delta = 200e3;
bool watchdog_state = false;

void watchdog_init() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_LED_RXL, OUTPUT);
  pinMode(PIN_LED_TXL, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(PIN_LED_RXL, HIGH);
  digitalWrite(PIN_LED_TXL, LOW);
  watchdog_last = micros();
}

void watchdog_throw() {
  digitalWrite(PIN_LED_RXL, LOW);
  digitalWrite(PIN_LED_TXL, HIGH);
  watchdog_last = micros();
  watchdog_state = true;
}

void watchdog_clean_timer(unsigned long now, unsigned long &last, unsigned long delta) {
  if (now - last >= delta * 10) {
    last += delta * 10;
    while( now - last >= delta ) { last += delta; }
    watchdog_throw();
  }
}

void watchdog_run() {
  if ( micros() - watchdog_last > watchdog_delta ) {
    digitalWrite(PIN_LED_RXL, HIGH);
    digitalWrite(PIN_LED_TXL, LOW);
    watchdog_state = false;
  }
}

bool watchdog_status() {
  return watchdog_state;
}
