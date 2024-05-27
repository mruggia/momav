
#include "ros/ros.h"
#include "momav/Remote.h"
#include "momav/BodyState.h"
#include "momav/BodySetp.h"
#include "momav/MotorSetp.h"
#include "momav/MotorState.h"
#include "momav/ServoSetp.h"
#include "momav/ServoState.h"
#include "momav/CommanderState.h"
#include "momav/CommanderStream.h"

#include <math.h>
#include <algorithm>
#include <thread>
#include<iostream>


// ################################################################################################

// callbacks
void callback_remote(const momav::Remote::ConstPtr& msg);
void callback_body_state_track(const momav::BodyState::ConstPtr& msg);
void callback_body_state_cam(const momav::BodyState::ConstPtr& msg);
void callback_body_state_sim(const momav::BodyState::ConstPtr& msg);
void callback_body_setp_pose(const momav::BodySetp::ConstPtr& msg);
void callback_body_setp_wrench(const momav::BodySetp::ConstPtr& msg);
void callback_motor_setp(const momav::MotorSetp::ConstPtr& msg);
void callback_servo_setp(const momav::ServoSetp::ConstPtr& msg);
void callback_motor_state(const momav::MotorState::ConstPtr& msg);
void callback_servo_state(const momav::ServoState::ConstPtr& msg);

// other prototypes
void commander_wait_all_topics();
void commander_wait_topic(int& timer);
void commander_wait_remote();
bool commander_check_remote();
bool commander_check_wrench();

// ros topics
ros::Subscriber sub_remote;				momav::Remote msg_remote;				int tout_remote;
ros::Subscriber sub_body_state_track;	momav::BodyState msg_body_state;		int tout_body_state;
ros::Subscriber sub_body_state_cam;
ros::Subscriber sub_body_state_sim;
ros::Subscriber sub_body_setp_pose;		momav::BodySetp msg_body_setp_pose;		int tout_body_setp_pose;
ros::Subscriber sub_body_setp_wrench;	momav::BodySetp msg_body_setp_wrench;	int tout_body_setp_wrench;
ros::Subscriber sub_motor_setp;			momav::MotorSetp msg_motor_setp; 		int tout_motor_setp;
ros::Subscriber sub_servo_setp;			momav::ServoSetp msg_servo_setp;		int tout_servo_setp;
ros::Subscriber sub_motor_state;		momav::MotorState msg_motor_state;		int tout_motor_state;
ros::Subscriber sub_servo_state;		momav::ServoState msg_servo_state;		int tout_servo_state;
ros::Publisher pub_commander;			momav::CommanderState msg_commander;
ros::Publisher pub_stream;				momav::CommanderStream msg_stream;

// ros parameters
std::string _body_state_topic;  	// topic used as body_state input (body_state_track, body_state_cam or body_state_sim)
int _rate;							// publish rate of commander_state and commander_stream topics
double _volt_min; 					// battery voltage that triggers a warning
int _arming_channel; 				// remote channel used for arming/disarming
double _timeout; int _timeout_int;	// time before a missing topic triggers a timeout [sec]

// global variables
bool READY = false;		// flag if drone is ready to fly (all needed topic present)

// ################################################################################################

// program entry point
int main(int argc, char **argv) {

	// init ros
	ros::init(argc, argv, "commander");
	ros::NodeHandle n;

	// get parameters
	ros::param::get("/momav/body_state", _body_state_topic);
	ros::param::get("~rate", _rate);
	ros::param::get("~volt_min", _volt_min);
	ros::param::get("~arm_chn", _arming_channel);
	ros::param::get("~timeout", _timeout); _timeout_int = std::max(2, (int)round(_timeout*((double)_rate)));

	// subscribe to all nodes and init timeout counter
	sub_remote           = n.subscribe("remote",           1, callback_remote, ros::TransportHints().udp());
	sub_body_state_track = n.subscribe("body_state_track", 1, callback_body_state_track, ros::TransportHints().udp());
	sub_body_state_cam   = n.subscribe("body_state_cam",   1, callback_body_state_cam);
	sub_body_state_sim   = n.subscribe("body_state_sim",   1, callback_body_state_sim);
	sub_body_setp_pose   = n.subscribe("body_setp_pose",   1, callback_body_setp_pose);
	sub_body_setp_wrench = n.subscribe("body_setp_wrench", 1, callback_body_setp_wrench);
	sub_motor_setp       = n.subscribe("motor_setp",       1, callback_motor_setp);
	sub_servo_setp       = n.subscribe("servo_setp",       1, callback_servo_setp);
	sub_motor_state      = n.subscribe("motor_state",      1, callback_motor_state);
	sub_servo_state      = n.subscribe("servo_state",      1, callback_servo_state);
	tout_remote = -1; 
	tout_body_state = -1; tout_body_setp_pose = -1; tout_body_setp_wrench = -1;
	tout_motor_setp = -1; tout_servo_setp = -1; tout_motor_state = -1; tout_servo_state = -1;


	// set up commander state publisher
	pub_commander = n.advertise<momav::CommanderState>("commander_state", 1);
	pub_stream = n.advertise<momav::CommanderStream>("commander_stream", 1);

	// initialize default messages
	msg_body_state.orientation.w = 1.0;
	msg_body_setp_pose.orientation.w = 1.0;
	msg_body_setp_wrench.orientation.w = 1.0;

	// wait for all needed topics in current mode
	std::thread thread_wait_topics(commander_wait_all_topics);

	// loop until user exits
	ros::Rate rate(_rate);
	while (ros::ok())  {

		// check if a topic timed out
		if (READY) {
			tout_remote++;           if( tout_remote == _timeout_int )           { msg_commander.remote = false;           ROS_ERROR("[commander] remote topic has timed out!"); }  
			tout_body_state++;       if( tout_body_state == _timeout_int )       { msg_commander.body_state = false;       ROS_ERROR("[commander] %s topic has timed out!", _body_state_topic.c_str()); }
			tout_body_setp_pose++;   if( tout_body_setp_pose == _timeout_int )   { msg_commander.body_setp_pose = false;   ROS_ERROR("[commander] body_setp_pose topic has timed out!"); }
			tout_body_setp_wrench++; if( tout_body_setp_wrench == _timeout_int ) { msg_commander.body_setp_wrench = false; ROS_ERROR("[commander] body_setp_wrench topic has timed out!"); }
			tout_motor_setp++;       if( tout_motor_setp == _timeout_int )       { msg_commander.motor_setp = false;       ROS_ERROR("[commander] motor_setp topic has timed out!"); }
			tout_servo_setp++;       if( tout_servo_setp == _timeout_int )       { msg_commander.servo_setp = false;       ROS_ERROR("[commander] servo_setp topic has timed out!"); }
			tout_motor_state++;      if( tout_motor_state == _timeout_int )      { msg_commander.motor_state = false;      ROS_ERROR("[commander] motor_state topic has timed out (hardware fault = %s)!", (msg_motor_state.watchdog)?"true":"false"); }
			tout_servo_state++;      if( tout_servo_state == _timeout_int )      { msg_commander.servo_state = false;      ROS_ERROR("[commander] servo_state topic has timed out (hardware fault = %s)!", (msg_servo_state.watchdog)?"true":"false"); }
		}

		// publish results
		msg_commander.header.stamp = ros::Time::now();
		pub_commander.publish(msg_commander);

		// publish low rate stream
		msg_stream.header.stamp = ros::Time::now();
		msg_stream.remote = msg_remote;
		msg_stream.body_state = msg_body_state;
		msg_stream.body_setp_pose = msg_body_setp_pose;
		msg_stream.body_setp_wrench = msg_body_setp_wrench;
		msg_stream.motor_setp = msg_motor_setp;
		msg_stream.motor_state = msg_motor_state;
		msg_stream.servo_setp = msg_servo_setp;
		msg_stream.servo_state = msg_servo_state;
		pub_stream.publish(msg_stream);

		// spin
		ros::spinOnce(); rate.sleep(); ros::spinOnce();
	}
	return 0;
}


// ################################################################################################

void callback_remote(const momav::Remote::ConstPtr& msg) {
	msg_remote = *(msg.get());
	msg_commander.remote = true;
	if (tout_remote > _timeout_int) { ROS_WARN("[commander] remote topic has resumed!"); }
	tout_remote = 0;

	if (!READY) {
		msg_commander.armed = false;
		return;
	}

	static bool reject_arm_flag = false;
	if (msg->channels[_arming_channel] > 0.5) {
		if ( msg_commander.armed == false ) {

			if (reject_arm_flag) { return; }

			if (!commander_check_remote()) {
				ROS_WARN("[commander] remote not in nominal state. REJECTED ARMING!");
				reject_arm_flag = true;
			} else if (!commander_check_wrench()) {
				ROS_WARN("[commander] wrench setpoint to high. REJECTED ARMING!");
				reject_arm_flag = true;
			} else {
				ROS_WARN("[commander] ARMED!");
				msg_commander.armed = true;
			}

		}
	} else {
		if ( msg_commander.armed == true ) { 
			ROS_WARN("[commander] DISARMED!");
			msg_commander.armed = false;
		}
		reject_arm_flag = false;
	}

}


void callback_body_state_track(const momav::BodyState::ConstPtr& msg) {
	if ( _body_state_topic != "body_state_track" ) { return; }
	msg_body_state = *(msg.get());
	msg_commander.body_state = true;
	if (tout_body_state > _timeout_int) { ROS_WARN("[commander] body_state_track topic has resumed!"); }
	tout_body_state = 0;
}
void callback_body_state_cam(const momav::BodyState::ConstPtr& msg) {
	if ( _body_state_topic != "body_state_cam" ) { return; }
	msg_body_state = *(msg.get());
	msg_commander.body_state = true;
	if (tout_body_state > _timeout_int) { ROS_WARN("[commander] body_state_cam topic has resumed!"); }
	tout_body_state = 0;
}
void callback_body_state_sim(const momav::BodyState::ConstPtr& msg) {
	if ( _body_state_topic != "body_state_sim" ) { return; }
	msg_body_state = *(msg.get());
	msg_commander.body_state = true;
	if (tout_body_state > _timeout_int) { ROS_WARN("[commander] body_state_sim topic has resumed!"); }
	tout_body_state = 0;
}


void callback_body_setp_pose(const momav::BodySetp::ConstPtr& msg) {
	msg_commander.body_setp_pose = true;
	msg_body_setp_pose = *(msg.get());
	if (tout_body_setp_pose > _timeout_int) { ROS_WARN("[commander] body_setp_pose topic has resumed!"); }
	tout_body_setp_pose = 0;
}
void callback_body_setp_wrench(const momav::BodySetp::ConstPtr& msg) {
	msg_commander.body_setp_wrench = true;
	msg_body_setp_wrench = *(msg.get());
	if (tout_body_setp_wrench > _timeout_int) { ROS_WARN("[commander] body_setp_wrench topic has resumed!"); }
	tout_body_setp_wrench = 0;
}


void callback_motor_setp(const momav::MotorSetp::ConstPtr& msg) { 
	msg_commander.motor_setp = true;
	msg_motor_setp = *(msg.get());
	if (tout_motor_setp > _timeout_int) { ROS_WARN("[commander] motor_setp topic has resumed!"); }
	tout_motor_setp = 0;
}
void callback_servo_setp(const momav::ServoSetp::ConstPtr& msg) { 
	msg_commander.servo_setp = true;
	msg_servo_setp = *(msg.get());
	if (tout_servo_setp > _timeout_int) { ROS_WARN("[commander] servo_setp topic has resumed!"); }
	tout_servo_setp = 0;
}
void callback_motor_state(const momav::MotorState::ConstPtr& msg) {
	static int loc_counter = 0; loc_counter++;
	if ( msg->voltage<_volt_min && msg->voltage>1.0 && loc_counter%10==0 ) { 
		ROS_WARN("[commander] battery low at %.1fV!", msg->voltage); 
	}
	msg_motor_state = *(msg.get());
	if (!msg->watchdog) {
		msg_commander.motor_state = true;
		if (tout_motor_state > _timeout_int) { ROS_WARN("[commander] motor_state topic has resumed!"); }
		tout_motor_state = 0;
	}
}
void callback_servo_state(const momav::ServoState::ConstPtr& msg) {
	msg_servo_state = *(msg.get());
	if (!msg->watchdog) {
		msg_commander.servo_state = true;
		if (tout_servo_state > _timeout_int) { ROS_WARN("[commander] servo_state topic has resumed!"); }
		tout_servo_state = 0;
	}
}


// ################################################################################################

// wait for all needed topics in current mode
void commander_wait_all_topics() {
	ros::Duration(0.5).sleep();

	ROS_WARN("[commander] synchronizing clock with ground pc...");
	system("(sudo ntpdate -b -p 4 ground > /dev/null 2>&1)");
	ROS_WARN("[commander] waiting for %s topic...", _body_state_topic.c_str());
	commander_wait_topic(tout_body_state);
	ROS_WARN("[commander] waiting for remote topic...");
	commander_wait_topic(tout_remote);
	commander_wait_remote();
	ROS_WARN("[commander] waiting for body_setp_pose topic...");
	commander_wait_topic(tout_body_setp_pose);
	ROS_WARN("[commander] waiting for body_setp_wrench topic...");
	commander_wait_topic(tout_body_setp_wrench);
	ROS_WARN("[commander] waiting for motor_setp topic...");
	commander_wait_topic(tout_motor_setp);
	ROS_WARN("[commander] waiting for servo_setp topic...");
	commander_wait_topic(tout_servo_setp);
	if (_body_state_topic != "body_state_sim") {
		ROS_WARN("[commander] waiting for motor_state topic...");
		commander_wait_topic(tout_motor_state);
		ROS_WARN("[commander] waiting for servo_state topic...");
		commander_wait_topic(tout_servo_state);
		ROS_WARN("[commander] READY TO FLY!");
	} else {
		ROS_WARN("[commander] READY TO SIMULATE!");
	}
	
	READY = true;
}

// wait for a topic to appear for the first time
void commander_wait_topic(int& timer) {
	ros::Rate rate(_rate);
	while(ros::ok() && (timer==-1)) {
		msg_commander.header.stamp = ros::Time::now();
		pub_commander.publish(msg_commander);
		ros::spinOnce(); rate.sleep(); ros::spinOnce();
	}
}

// wait for the remote to return to nominal state
void commander_wait_remote() {
	ros::Rate rate(_rate);

	if (msg_remote.channels[_arming_channel] > 0.5) {
		ROS_WARN("[commander] waiting for remote to be DISARMED...");
	}
	while(ros::ok() && (msg_remote.channels[_arming_channel] > 0.5)) {
		msg_commander.header.stamp = ros::Time::now();
		pub_commander.publish(msg_commander);
		ros::spinOnce(); rate.sleep(); ros::spinOnce();
	}

	if (!commander_check_remote()) {
		ROS_WARN("[commander] waiting for remote to return to nominal state...");
	}
	while(ros::ok() && !commander_check_remote()) {
		msg_commander.header.stamp = ros::Time::now();
		pub_commander.publish(msg_commander);
		ros::spinOnce(); rate.sleep(); ros::spinOnce();
	}
}

// check if the remote is in its nominal state
bool commander_check_remote() {

	static int _x_chn; ros::param::get("/momav/controller_navigation/x_chn", _x_chn);
	static int _y_chn; ros::param::get("/momav/controller_navigation/y_chn", _y_chn);
	static int _z_chn; ros::param::get("/momav/controller_navigation/z_chn", _z_chn);
	static int _yaw_chn; ros::param::get("/momav/controller_navigation/yaw_chn", _yaw_chn);
	static int _pitch_chn; ros::param::get("/momav/controller_navigation/pitch_chn", _pitch_chn);
	static int _roll_chn; ros::param::get("/momav/controller_navigation/roll_chn", _roll_chn);

	return (( msg_remote.channels[_x_chn] > 0.45 && msg_remote.channels[_x_chn] < 0.55 ) &&
		( msg_remote.channels[_y_chn] > 0.45 && msg_remote.channels[_y_chn] < 0.55 ) &&
		( msg_remote.channels[_z_chn] < 0.05 ) &&
		( msg_remote.channels[_yaw_chn] > 0.45 && msg_remote.channels[_yaw_chn] < 0.55 ) &&
		( msg_remote.channels[_pitch_chn] > 0.45 && msg_remote.channels[_pitch_chn] < 0.55 ) &&
		( msg_remote.channels[_roll_chn] > 0.45 && msg_remote.channels[_roll_chn] < 0.55 )
	);

}

// check if wrench setpoint is below treshold for arming
bool commander_check_wrench() {

	static double _arm_lim_f;  ros::param::get("~arm_lim_f", _arm_lim_f);
	static double _arm_lim_t;   ros::param::get("~arm_lim_t", _arm_lim_t);

	return (
		std::abs(msg_body_setp_wrench.force.x)  < _arm_lim_f &&
		std::abs(msg_body_setp_wrench.force.y)  < _arm_lim_f &&
		        (msg_body_setp_wrench.force.z)  < _arm_lim_f &&		/* ignore -Z axis in takeoff */
		std::abs(msg_body_setp_wrench.torque.x) < _arm_lim_t &&
		std::abs(msg_body_setp_wrench.torque.y) < _arm_lim_t &&
		std::abs(msg_body_setp_wrench.torque.z) < _arm_lim_t
	);

}