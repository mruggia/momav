#include "ros/ros.h"
#include "momav/CommanderState.h"
#include "momav/BodyState.h"
#include "momav/BodySetp.h"

#include <chrono>
#include <algorithm>
#include <math.h>
#include <Eigen/Dense>
using namespace Eigen;


// ################################################################################################

// callbacks
void callback_commander(const momav::CommanderState::ConstPtr& msg);
void callback_body_state(const momav::BodyState::ConstPtr& msg);
void callback_body_setp_pose(const momav::BodySetp::ConstPtr& msg);

// controllers
bool run_controller();

// ros publishers and subscribers
ros::Subscriber  sub_commander;        momav::CommanderState msg_commander;
ros::Subscriber  sub_body_state;       momav::BodyState msg_body_state, last_body_state;
ros::Subscriber  sub_body_setp_pose;   momav::BodySetp msg_body_setp_pose;
ros::Publisher   pub_body_setp_wrench; momav::BodySetp msg_body_setp_wrench;

// ros parameters
std::string _body_state_topic;	// topic used as body_state input (body_state_track, body_state_cam, or body_state_sim)
double _m, _Ix, _Iy, _Iz;		// mass and angular mass of momav
double _P_rot, _I_rot, _D_rot;	// PID gains of rotation controller
double _P_pos, _I_pos, _D_pos;	// PID gains of position controller
double _I_rot_lim, _I_pos_lim;	// I gain limit for rotation & position controllers
double _I_t_sat;				// time to saturate integrator
double _I_zmin, _I_zmin_corr;	// z height at which to turn on integral gain (_I_zmin_corr corrected for arming height)
double _A_rot, _A_pos;			// virtual impedance factor on rotation & translation
double _f_lim, _t_lim;			// limit force/torque to reject wrench setpoint

// global variables
bool ARMED = false;		// flag of current arming status
bool READY = false;		// flag if controller is ready to be run
bool ERROR = false;		// flag if an error has been detected
bool INTEGRATE = false;	// flag if integrator of pid should be running

// ################################################################################################

// program entry point
int main(int argc, char **argv) {

	// setup ros node
	ros::init(argc, argv, "controller_pose");
	ros::NodeHandle n;

	// load parameters
	ros::param::get("/momav/body_state", _body_state_topic);
	ros::param::get("/momav/m", _m); ros::param::get("/momav/Ix", _Ix); ros::param::get("/momav/Iy", _Iy); ros::param::get("/momav/Iz", _Iz);
	ros::param::get("~P_rot", _P_rot); ros::param::get("~I_rot", _I_rot); ros::param::get("~D_rot", _D_rot);
	ros::param::get("~P_pos", _P_pos); ros::param::get("~I_pos", _I_pos); ros::param::get("~D_pos", _D_pos);
	ros::param::get("~I_rot_lim", _I_rot_lim); ros::param::get("~I_pos_lim", _I_pos_lim);
	ros::param::get("~I_t_sat", _I_t_sat); 
	ros::param::get("~I_zmin", _I_zmin); _I_zmin_corr = _I_zmin;
	ros::param::get("~A_rot", _A_rot); ros::param::get("~A_pos", _A_pos);
	ros::param::get("~f_lim", _f_lim); ros::param::get("~t_lim", _t_lim);

	// setup published and subscribed topics
	sub_commander = n.subscribe("commander_state", 1, callback_commander);
	sub_body_state = n.subscribe(_body_state_topic, 1, callback_body_state, ros::TransportHints().udp());
	sub_body_setp_pose = n.subscribe("body_setp_pose", 1, callback_body_setp_pose);
	pub_body_setp_wrench = n.advertise<momav::BodySetp>("body_setp_wrench", 1);

	//loop until node is stopped
	ros::spin();
	return 0;
}


// ################################################################################################

// callback for commander topic
void callback_commander(const momav::CommanderState::ConstPtr& msg) { 
	msg_commander = *(msg.get());

	if (!ARMED && msg_commander.armed) {
		_I_zmin_corr = _I_zmin + msg_body_state.translation.z;
	}

	READY = ( msg_commander.body_state && msg_commander.body_setp_pose && !ERROR );
	ARMED = msg_commander.armed;
}

// callback for body state topic (body_state_track, body_state_cam, or body_state_sim)
void callback_body_state(const momav::BodyState::ConstPtr& msg) {
	msg_body_state = *(msg.get());
	if (!READY) { return; }

	if (run_controller()) {
		pub_body_setp_wrench.publish(msg_body_setp_wrench);
	} else {
		ROS_ERROR("[controller_pose] controller failed. invalid wrench setpoint!");
		ERROR = true;
		READY = false;
	}
}

// callback for body setpoint pose topic
void callback_body_setp_pose(const momav::BodySetp::ConstPtr& msg) { 
	msg_body_setp_pose = *(msg.get());
	if (msg_body_state.translation.z > _I_zmin_corr && READY && ARMED) { INTEGRATE = true; }
	if (!READY || !ARMED) { INTEGRATE = false; }
}

// ################################################################################################

bool run_controller() {

	// get controller update rate
	static double h = 0.001;
	static auto t_old = std::chrono::high_resolution_clock::now();
	auto t_new = std::chrono::high_resolution_clock::now();
	double dt = ((double) std::chrono::duration_cast<std::chrono::microseconds>(t_new-t_old).count()) / 1.0e6;
	h = 0.95*h + 0.05*dt;
	t_old = t_new;

	// copy over setpoint
	msg_body_setp_wrench = msg_body_setp_pose;

	// position control
	{

		// calculate position error
		Vector3d p_c = Vector3d(msg_body_state.translation.x,      msg_body_state.translation.y,      msg_body_state.translation.z);
		Vector3d p_s = Vector3d(msg_body_setp_pose.translation.x,  msg_body_setp_pose.translation.y,  msg_body_setp_pose.translation.z);
		Vector3d p_e = p_c - p_s;

		// calculate position derivative error
		Vector3d dp_c = Vector3d(msg_body_state.linear_velocity.x,     msg_body_state.linear_velocity.y,     msg_body_state.linear_velocity.z);
		Vector3d dp_s = Vector3d(msg_body_setp_pose.linear_velocity.x, msg_body_setp_pose.linear_velocity.y, msg_body_setp_pose.linear_velocity.z);
		Vector3d dp_e = dp_c - dp_s;

		// calculate position integral error
		static Vector3d ip_e = Vector3d(0.0, 0.0, 0.0);
		if (INTEGRATE) {
			ip_e += p_e.cwiseMax(-_I_pos_lim/_I_t_sat/_I_pos).cwiseMin(+_I_pos_lim/_I_t_sat/_I_pos) * h;
			ip_e = ip_e.cwiseMax(-_I_pos_lim/_I_pos).cwiseMin(+_I_pos_lim/_I_pos);
		} else {
			ip_e = Vector3d(0.0, 0.0, 0.0);
		}

		// set force setpoints based on PID control & zero setpoint
		Vector3d force_req = -_P_pos*p_e -_I_pos*ip_e -_D_pos*dp_e;
		msg_body_setp_wrench.force.x = force_req(0) + msg_body_setp_pose.force.x;
		msg_body_setp_wrench.force.y = force_req(1) + msg_body_setp_pose.force.y;
		msg_body_setp_wrench.force.z = force_req(2) + msg_body_setp_pose.force.z;

	}

	// orientation control
	{

		// calculate quaternion error
		Quaterniond q_c(msg_body_state.orientation.w,     msg_body_state.orientation.x,     msg_body_state.orientation.y,     msg_body_state.orientation.z);
		Quaterniond q_s(msg_body_setp_pose.orientation.w, msg_body_setp_pose.orientation.x, msg_body_setp_pose.orientation.y, msg_body_setp_pose.orientation.z);
		Quaterniond q_e = q_s.inverse() * q_c;

		// calculate rotation error
		double a_e = 2.0*acos(std::clamp(q_e.w(), -1.0, 1.0));
		Vector3d r_e = q_c * ( (a_e/sin(a_e/2.0)) * q_e.vec() );
		if (a_e > M_PI) { r_e = (a_e-2.0*M_PI)/a_e * r_e; }
		if (a_e < 1e-9) { r_e = Vector3d(0.0, 0.0, 0.0); }

		// calculate rotation derivative error
		Vector3d dr_c = Vector3d(msg_body_state.angular_velocity.x,     msg_body_state.angular_velocity.y,     msg_body_state.angular_velocity.z);
		Vector3d dr_s = Vector3d(msg_body_setp_pose.angular_velocity.x, msg_body_setp_pose.angular_velocity.y, msg_body_setp_pose.angular_velocity.z);
		Vector3d dr_e = dr_c - dr_s;

		// calculate integrated rotation error
		static Vector3d ir_e = Vector3d(0.0, 0.0, 0.0);
		if (INTEGRATE) {
			ir_e += r_e.cwiseMax(-_I_rot_lim/_I_t_sat/_I_rot).cwiseMin(+_I_rot_lim/_I_t_sat/_I_rot) * h;
			ir_e = ir_e.cwiseMax(-_I_rot_lim/_I_rot).cwiseMin(+_I_rot_lim/_I_rot);
		} else {
			ir_e = Vector3d(0.0, 0.0, 0.0);
		}

		// set torque setpoints based on PID control & zero setpoint
		Vector3d torqe_req = -_P_rot*r_e -_I_rot*ir_e -_D_rot*dr_e;
		msg_body_setp_wrench.torque.x = torqe_req(0) + msg_body_setp_pose.torque.x;
		msg_body_setp_wrench.torque.y = torqe_req(1) + msg_body_setp_pose.torque.y;
		msg_body_setp_wrench.torque.z = torqe_req(2) + msg_body_setp_pose.torque.z;

	}

	// set virtual impedance
	Quaterniond q_c(msg_body_state.orientation.w, msg_body_state.orientation.x, msg_body_state.orientation.y, msg_body_state.orientation.z);
	Vector3d _I = q_c * Vector3d(_Ix, _Iy, _Iz);
	msg_body_setp_wrench.force.x  -= _A_pos*_m    * msg_body_state.linear_acceleration.x;
	msg_body_setp_wrench.force.y  -= _A_pos*_m    * msg_body_state.linear_acceleration.y;
	msg_body_setp_wrench.force.z  -= _A_pos*_m    * msg_body_state.linear_acceleration.z;
	msg_body_setp_wrench.torque.x -= _A_rot*_I(0) * msg_body_state.angular_acceleration.x;
	msg_body_setp_wrench.torque.y -= _A_rot*_I(1) * msg_body_state.angular_acceleration.y;
	msg_body_setp_wrench.torque.z -= _A_rot*_I(2) * msg_body_state.angular_acceleration.z;

	// finalize message
	msg_body_setp_wrench.header.stamp = ros::Time::now();

	// sanity check
	if ( ARMED && (
		 abs(msg_body_setp_wrench.force.x) > _f_lim || std::isnan(msg_body_setp_wrench.force.x) ||
		 abs(msg_body_setp_wrench.force.y) > _f_lim || std::isnan(msg_body_setp_wrench.force.y) ||
//		    (msg_body_setp_wrench.force.z) > _f_lim || std::isnan(msg_body_setp_wrench.force.z) || 		/* ignore -Z in takeoff */
		 abs(msg_body_setp_wrench.torque.x) > _t_lim || std::isnan(msg_body_setp_wrench.torque.x) ||
		 abs(msg_body_setp_wrench.torque.y) > _t_lim || std::isnan(msg_body_setp_wrench.torque.y) ||
		 abs(msg_body_setp_wrench.torque.z) > _t_lim || std::isnan(msg_body_setp_wrench.torque.z) )) {
		return false;
	}

	return true;
}
