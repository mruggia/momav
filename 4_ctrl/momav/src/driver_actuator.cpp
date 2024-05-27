#include "ros/ros.h"

#include "momav/CommanderState.h"
#include "momav/MotorSetp.h"
#include "momav/MotorState.h"
#include "momav/ServoSetp.h"
#include "momav/ServoState.h"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
extern "C" {
	#include <linux/i2c.h>
	#include <linux/i2c-dev.h>
	#include <i2c/smbus.h>
}


// ################################################################################################

// callback prototypes
void callback_commander(const momav::CommanderState::ConstPtr& msg);
void callback_motor_setp(const momav::MotorSetp::ConstPtr& msg);
void callback_servo_setp(const momav::ServoSetp::ConstPtr& msg);

// utility prototypes
bool init_i2c(int& dev, int addr, const char* name);
void update_servo();
void update_motor();

// ros topics
ros::Subscriber sub_commander;   momav::CommanderState msg_commander;
ros::Subscriber sub_motor_setp;  momav::MotorSetp msg_motor_setp;
ros::Subscriber sub_servo_setp;  momav::ServoSetp msg_servo_setp;
ros::Publisher  pub_motor_state; momav::MotorState msg_motor_state;
ros::Publisher  pub_servo_state; momav::ServoState msg_servo_state;

// global variables
std::string _i2c_dev;	// i2c device path (e.g. "/dev/i2c-1")
int dev_servo1, dev_servo2, dev_servo3, dev_motor;	// handles to various i2c devices
bool READY = false;		// flag if node is ready to be run
bool ERROR = false;		// flag if node detected error and has stopped forwarding motor/servo setpoints
bool ARMED = false;		// flag of current arming status

// ################################################################################################

int main(int argc, char **argv) {

	// setup ros node
	ros::init(argc, argv, "driver_actuator");
	ros::NodeHandle n;
	ros::param::get("~device", _i2c_dev);

	// setup i2c
	if (!init_i2c(dev_motor,  0x09, "motor"))  { ros::spin(); return 0; }
	if (!init_i2c(dev_servo1, 0x10, "servo1")) { ros::spin(); return 0; }
	if (!init_i2c(dev_servo2, 0x11, "servo2")) { ros::spin(); return 0; }
	if (!init_i2c(dev_servo3, 0x12, "servo3")) { ros::spin(); return 0; }

	// seup ros topics
	pub_motor_state = n.advertise<momav::MotorState>("motor_state", 1);
	pub_servo_state = n.advertise<momav::ServoState>("servo_state", 1);
	sub_motor_setp = n.subscribe("motor_setp", 1, callback_motor_setp);
	sub_servo_setp = n.subscribe("servo_setp", 1, callback_servo_setp);
	sub_commander = n.subscribe("commander_state", 1, callback_commander);

	// loop untill the user exits
	ros::spin();

	// before quitting, turn of motors just to be sure
	for (int i=0; i<6; i++) { msg_motor_state.throttle[i] = 0.0; }
	update_motor();
	
	close(dev_servo1);
	close(dev_servo2);
	close(dev_servo3);
	close(dev_motor);
	return 0;
}


// ################################################################################################

// callback for commander topic
void callback_commander(const momav::CommanderState::ConstPtr& msg) { 
	msg_commander = *(msg.get());
	ARMED = msg_commander.armed;
	bool new_READY = ( msg_commander.motor_setp && msg_commander.servo_setp && !ERROR );

	//turn of motors if ready goes from true to false
	if( READY && !new_READY ) {
		for (int i=0; i<6; i++) { msg_motor_state.throttle[i] = 0.0; }
		update_motor();
	}

	READY = new_READY;
}

// forward motor setpoint messages to mcu
void callback_motor_setp(const momav::MotorSetp::ConstPtr& msg) {
	if(!READY) { return; }
	memcpy(&msg_motor_setp, msg.get(), sizeof(momav::MotorSetp));

	msg_motor_state.header.stamp = ros::Time::now();
	msg_motor_state.throttle = msg_motor_setp.throttle;
	if (!ARMED) { for (int i=0; i<6; i++) { msg_motor_state.throttle[i] = 0.0; } }
	update_motor();
	pub_motor_state.publish(msg_motor_state);
}

// forward arm setpoint messages to mcu
void callback_servo_setp(const momav::ServoSetp::ConstPtr& msg) {
	if(!READY) { return; }
	memcpy(&msg_servo_setp, msg.get(), sizeof(momav::ServoSetp));

	msg_servo_state.header.stamp = ros::Time::now();
	msg_servo_state.setpoint = msg_servo_setp.setpoint;
	if (!ARMED) { for (int i=0; i<6; i++) { msg_servo_state.setpoint[i] = 0.0; } }
	update_servo();
	pub_servo_state.publish(msg_servo_state);
}


// ################################################################################################

// initialize i2c connection to a device
bool init_i2c(int& dev, int addr, const char* name) {
	if ((dev = open(_i2c_dev.c_str(), O_RDWR)) < 0) { 
		ROS_ERROR("[driver_actuator] failed to open the i2c bus for %s", name); 
		return false; 
	}
	if (ioctl (dev, I2C_SLAVE, addr) < 0) { 
		ROS_ERROR("[driver_actuator] failed to open i2c slave %s", name); 
		return false; 
	}
	if (i2c_smbus_write_quick(dev, I2C_SMBUS_WRITE) < 0) { 
		ROS_ERROR("[driver_actuator] failed to talk to i2c slave %s", name); 
		return false; 
	}
	return true;
}

//send arm setpoint to mcu
void update_servo() {

	msg_servo_state.watchdog = false;

	int dev; float *pid_setp1, *pid_setp2, *pid_in1, *pid_in2, *pid_out1, *pid_out2;
	for(int id=0; id<3; id++) {
		
		if(id == 0) { 
			dev = dev_servo1; 
			pid_setp1 = &msg_servo_state.setpoint[5]; pid_setp2 = &msg_servo_state.setpoint[4];
			pid_in1   = &msg_servo_state.position[5]; pid_in2   = &msg_servo_state.position[4];
			pid_out1  = &msg_servo_state.voltage[5];  pid_out2  = &msg_servo_state.voltage[4];
		} else if(id == 1) {
			dev = dev_servo2; 
			pid_setp1 = &msg_servo_state.setpoint[0]; pid_setp2 = &msg_servo_state.setpoint[3];
			pid_in1   = &msg_servo_state.position[0]; pid_in2   = &msg_servo_state.position[3];
			pid_out1  = &msg_servo_state.voltage[0];  pid_out2  = &msg_servo_state.voltage[3];
		} else if (id == 2) {
			dev = dev_servo3; 
			pid_setp1 = &msg_servo_state.setpoint[1]; pid_setp2 = &msg_servo_state.setpoint[2];
			pid_in1   = &msg_servo_state.position[1]; pid_in2   = &msg_servo_state.position[2];
			pid_out1  = &msg_servo_state.voltage[1];  pid_out2  = &msg_servo_state.voltage[2];
		}

		union {
			unsigned char c[8];
			struct {
				float pid_setp1;
				float pid_setp2;
			} d;
		} snd;
		snd.d.pid_setp1 = *pid_setp1;
		snd.d.pid_setp2 = *pid_setp2;
		if (write(dev, snd.c, 8) != 8) {
			ROS_ERROR("[driver_actuator] failed i2c write to servo%d", id);
			ERROR = true;
		}

		union {
			unsigned char c[20];
			struct {
				float pid_in1;
				float pid_out1;
				float pid_in2;
				float pid_out2;
				bool watchdog;
			} d;
		} rcv;
		if (read(dev, rcv.c, 20) != 20) {
			ROS_ERROR("[driver_actuator] failed i2c read to servo%d", id);
			ERROR = true;
		}

		*pid_in1 = rcv.d.pid_in1;
		*pid_out1 = rcv.d.pid_out1;
		*pid_in2 = rcv.d.pid_in2;
		*pid_out2 = rcv.d.pid_out2;
		msg_servo_state.watchdog = (msg_servo_state.watchdog || rcv.d.watchdog);

	}

}

//send motor setpoint to mcu
void update_motor() {

	union {
		unsigned char c[24];
		float d[6];
	} snd;
	snd.d[1] = msg_motor_state.throttle[0];
	snd.d[0] = msg_motor_state.throttle[1];
	snd.d[5] = msg_motor_state.throttle[2];
	snd.d[4] = msg_motor_state.throttle[3];
	snd.d[3] = msg_motor_state.throttle[4];
	snd.d[2] = msg_motor_state.throttle[5];
	if (write(dev_motor, snd.c, 24) != 24) {
		ROS_ERROR("[driver_actuator] failed i2c write to motor");
		ERROR = true;
	}

	union {
		unsigned char c[56];
		struct {
			float volt;
			float curr[6];
			float rpm[6]; 
			bool watchdog;
		} d;
  	} rcv;
	if (read(dev_motor, rcv.c, 56) != 56) {
		ROS_ERROR("[driver_actuator] failed i2c read to motor");
		ERROR = true;
	}
	msg_motor_state.voltage = rcv.d.volt;
	msg_motor_state.current[0] = rcv.d.curr[1];
	msg_motor_state.current[1] = rcv.d.curr[0];
	msg_motor_state.current[2] = rcv.d.curr[5];
	msg_motor_state.current[3] = rcv.d.curr[4];
	msg_motor_state.current[4] = rcv.d.curr[3];
	msg_motor_state.current[5] = rcv.d.curr[2];
	msg_motor_state.rpm[0] = rcv.d.rpm[1];
	msg_motor_state.rpm[1] = rcv.d.rpm[0];
	msg_motor_state.rpm[2] = rcv.d.rpm[5];
	msg_motor_state.rpm[3] = rcv.d.rpm[4];
	msg_motor_state.rpm[4] = rcv.d.rpm[3];
	msg_motor_state.rpm[5] = rcv.d.rpm[2];
	msg_motor_state.watchdog = rcv.d.watchdog;

}