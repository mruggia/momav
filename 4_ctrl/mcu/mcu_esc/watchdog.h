#ifndef WATCHDOG_h
#define WATCHDOG_h

void watchdog_throw();
void watchdog_init();
void watchdog_clean_timer(unsigned long now, unsigned long &last, unsigned long delta);
void watchdog_run();
bool watchdog_status();

#endif
