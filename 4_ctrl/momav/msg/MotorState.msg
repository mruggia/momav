Header header
float32[6] throttle		# motor trottle percentage of all 6 motors [0-1]
float32[6] current		# motor current of all 6 motors [A]
float32[6] rpm			# motor rpm of all 6 motors [rpm]
float32 voltage			# battery voltage [V]
bool watchdog			# watchdog flag if mcu_esc arduino detects a problem