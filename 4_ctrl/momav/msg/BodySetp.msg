Header header

geometry_msgs/Vector3 translation			# body setpoint xyz coordinate (inertial frame)
geometry_msgs/Vector3 linear_velocity		# body setpoint xyz velocity (inertial frame)
geometry_msgs/Vector3 linear_acceleration	# body setpoint xyz acceleration (inertial frame)
geometry_msgs/Vector3 force					# body force setpoint (inertial frame)

geometry_msgs/Quaternion orientation		# body setpoint orientation (transforms body to inertial frame)
geometry_msgs/Vector3 angular_velocity		# body setpoint angluar velocity (inertial frame)
geometry_msgs/Vector3 angular_acceleration	# body setpoint angular acceleration (inertial frame)
geometry_msgs/Vector3 torque				# body torque setpoint (inertial frame)

int32 setpoint_type							# type of setpoint (-1: manual, other: step in fixed setpoint trajectory sequence)