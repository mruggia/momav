#include "ros/ros.h"
#include "momav/MotorSetp.h"
#include "momav/ServoSetp.h"
#include "std_msgs/Header.h"

#include <cmath>
#include <thread>

// ################################################################################################

// prototypes
void calib_step();
void calib_cont();
void ramp_throt(int mot, double throt);

// ros publishers and subscribers
ros::Publisher   pub_motor_setp;  momav::MotorSetp msg_motor_setp;
ros::Publisher   pub_servo_setp;  momav::ServoSetp msg_servo_setp;
ros::Publisher   pub_calib_stamp; std_msgs::Header msg_calib_stamp;

// ros parameters
std::string _mode;
int _thr_beg, _thr_stp, _thr_end; double _arm_rev, _arm_stp, _arm_dly, _arm_spd;

// global variables
bool READY = false;		// flag if node is ready to be run

// ################################################################################################

// program entry point
int main(int argc, char **argv) {

	// setup ros node
	ros::init(argc, argv, "controller_calib");
	ros::NodeHandle n;

	// setup published and subscribed topics
	pub_motor_setp = n.advertise<momav::MotorSetp>("motor_setp", 1);
	pub_servo_setp = n.advertise<momav::ServoSetp>("servo_setp", 1);
	pub_calib_stamp = n.advertise<std_msgs::Header>("calib_stamp", 1);

	// load parameters
	ros::param::get("~mode", _mode);
	ros::param::get("~thr_beg", _thr_beg);
	ros::param::get("~thr_stp", _thr_stp);
	ros::param::get("~thr_end", _thr_end);
	ros::param::get("~arm_rev", _arm_rev);
	ros::param::get("~arm_stp", _arm_stp);
	ros::param::get("~arm_dly", _arm_dly);
	ros::param::get("~arm_spd", _arm_spd);

	// init data & start sender thread
	for(int i=0; i<6; i++) { msg_motor_setp.throttle[i] = 0.0; }
	for(int i=0; i<6; i++) { msg_servo_setp.setpoint[i] = 0.0; }
	READY = true;
	std::thread sender_thread;
	if (_mode == "step") {
		sender_thread = std::thread(calib_step);
	} else if (_mode == "cont") {
		sender_thread = std::thread(calib_cont);
	}
	
	// continuously send motor & servo setpoints
	ros::Rate rate(100);
	while (ros::ok() && READY) {

		msg_motor_setp.header.stamp = ros::Time::now();
		msg_servo_setp.header.stamp = ros::Time::now();
		pub_motor_setp.publish(msg_motor_setp);
		pub_servo_setp.publish(msg_servo_setp);

		ros::spinOnce();
		rate.sleep();
	}

	// stop thread
	READY = false;
	sender_thread.join();

	return 0;
}

// ################################################################################################

// mode="step": step trough all throttle&angle combinations for each arm
void calib_step() {
	ros::Duration(2.0).sleep();

	for(int mot=0; mot<6; mot++) {

		int rev_tot = 0;
		for (int thr=_thr_beg; thr<=_thr_end; thr+=_thr_stp) {

			ramp_throt(mot, ((double)thr)/100.0);
			msg_servo_setp.setpoint[mot] = ((double)rev_tot)*2.0*M_PI;

			ros::Duration(_arm_dly).sleep();

			for(int rev=0; rev<_arm_rev; rev++) {
				for(int ang=0; ang<360; ang+=_arm_stp) {

					msg_servo_setp.setpoint[mot] = ((double)(ang+rev_tot*360)) *(M_PI/180.0);

					ros::Duration(_arm_dly).sleep();
					msg_calib_stamp.stamp = ros::Time::now();
					pub_calib_stamp.publish(msg_calib_stamp);
					if(!READY) { return; }

				}
				rev_tot++;
			}
		}

		ramp_throt(mot, 0.0);
		msg_servo_setp.setpoint[mot] = ((double)rev_tot)*2.0*M_PI;
	}

	ros::Duration(2.0).sleep();
	for(int i=0; i<6; i++) { msg_servo_setp.setpoint[i] = 0.0; }
	READY = false;
}

// mode="cont": continuously rotate trough angles & step trough throttles
void calib_cont() {
	ros::Duration(2.0).sleep();

	ros::Rate rate(100);
	for(int mot=0; mot<6; mot++) {

		for (int thr=_thr_beg; thr<=_thr_end; thr+=_thr_stp) {

			ramp_throt(mot, ((double)thr)/100.0);
			ros::Duration(0.6).sleep();
			msg_calib_stamp.stamp = ros::Time::now();
			pub_calib_stamp.publish(msg_calib_stamp);

			double pos = 0.0;
			while(true) {
				msg_servo_setp.setpoint[mot] = pos;
				pos += _arm_spd*2.0*M_PI/100.0;
				if (pos >= _arm_rev*2.0*M_PI) { break; }
				rate.sleep();
				if(!READY) { return; }
			}
			msg_servo_setp.setpoint[mot] = _arm_rev*2.0*M_PI;
			ros::Duration(0.6).sleep();
			msg_calib_stamp.stamp = ros::Time::now();
			pub_calib_stamp.publish(msg_calib_stamp);

			while(true) {
				msg_servo_setp.setpoint[mot] = pos;
				pos -= _arm_spd*2.0*M_PI/100.0;
				if (pos <= 0.0) { break; }
				rate.sleep();
				if(!READY) { return; }
			}

			msg_servo_setp.setpoint[mot] = 0.0;
		}

		ramp_throt(mot, 0.0);
	}
	READY = false;
}


// ################################################################################################

// ramps throttle slowly to a target
void ramp_throt(int mot, double throt) {
	double thr_curr = msg_motor_setp.throttle[mot];
	double thr_end = throt;
	double thr_step = 0.005; if (thr_curr>thr_end) { thr_step = -thr_step; }

	ros::Rate rate(100);
	while( std::abs(thr_curr-thr_end) > std::abs(thr_step) ) {
		thr_curr += thr_step;
		msg_motor_setp.throttle[mot] = thr_curr;
		rate.sleep();
	}
	msg_motor_setp.throttle[mot] = thr_end;
}