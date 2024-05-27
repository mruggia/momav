Header header
float32[6] setpoint		# servo setpoint angle for all 6 arms [rad]
float32[6] position		# servo current angle for all 6 arms [rad]
float32[6] voltage		# servo control voltage for all 6 arms [-1 to 1]
bool watchdog			# watchdog flag if mcu_servo arduino detects a problem