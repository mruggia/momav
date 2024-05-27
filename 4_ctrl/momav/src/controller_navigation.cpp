#include "ros/ros.h"
#include "momav/CommanderState.h"
#include "momav/Remote.h"
#include "momav/BodySetp.h"
#include "momav/BodyState.h"

#include <algorithm>
#include <math.h>
#include <Eigen/Dense>
using namespace Eigen;


// ################################################################################################

// callbacks
void callback_commander(const momav::CommanderState::ConstPtr& msg);
void callback_remote(const momav::Remote::ConstPtr& msg);
void callback_body_state(const momav::BodyState::ConstPtr& msg);

// navigator
bool run_navigator();

// utility
void _enforce_vel_acc(double& curr, double& vel, double& acc, double setp, double lim_vel, double lim_acc, double h);

// ros publishers and subscribers
ros::Subscriber  sub_commander;      momav::CommanderState msg_commander;
ros::Subscriber  sub_remote;         momav::Remote msg_remote;
ros::Subscriber  sub_body_state;     momav::BodyState msg_body_state;
ros::Publisher   pub_body_setp_pose; momav::BodySetp msg_body_setp_pose;

// ros parameters
std::string _body_state_topic;					// topic used as body_state input (body_state_track, body_state_cam, or body_state_sim)
int _traj_run_chn, _traj_sel_chn; 				// remote channels for running & selecting fixed setpoint trajectories
double _traj_pos_dist, _traj_rot_dist;			// fixed trajectory position & orientation changes from 0
double _traj_pos_time, _traj_rot_time;			// fixed trajectory position & orientation time for one sequence step
double _vel_pos, _acc_pos, _vel_rot, _acc_rot;	// velocity and acceleration limits on position and 
double _jerk_flt;								// moving average filter on feed-forward force/torque
int _x_chn;     double _x_min, _x_max;			// x coord min/max setpoint at full remote deflection
int _y_chn;     double _y_min, _y_max;			// y coord min/max setpoint at full remote deflection
int _z_chn;     double _z_min, _z_max;			// z coord min/max setpoint at full remote deflection
int _yaw_chn;   double _yaw_min, _yaw_max;		// yaw min/max setpoint at full remote deflection
int _pitch_chn; double _pitch_min, _pitch_max;	// yaw min/max setpoint at full remote deflection
int _roll_chn;  double _roll_min, _roll_max;	// yaw min/max setpoint at full remote deflection
double _m, _Ix, _Iy, _Iz;						// mass and angular mass of momav

// global variables
bool READY = false;						// flag if controller is ready to be run
const Vector3d _traj_sequence[] = {		// steps sequence in fixed trajectory
	Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 0.0, 0.0),
	Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0),
	Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 1.0),
	Vector3d(0.0, 0.0, 0.0), Vector3d(-1.0, 0.0, 0.0),
	Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, -1.0, 0.0),
	Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, -1.0),
	Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0)
};
const int _traj_sequence_len = sizeof(_traj_sequence)/sizeof(Vector3d);

// ################################################################################################

// program entry point
int main(int argc, char **argv) {

	// setup ros node
	ros::init(argc, argv, "controller_navigation");
	ros::NodeHandle n;

	// load parameters
	ros::param::get("/momav/body_state", _body_state_topic);
	ros::param::get("~traj_run_chn", _traj_run_chn); ros::param::get("~traj_sel_chn", _traj_sel_chn);
	ros::param::get("~traj_pos_dist", _traj_pos_dist); ros::param::get("~traj_rot_dist", _traj_rot_dist); 
	ros::param::get("~traj_pos_time", _traj_pos_time); ros::param::get("~traj_rot_time", _traj_rot_time); 
	ros::param::get("~vel_pos", _vel_pos); ros::param::get("~acc_pos", _acc_pos);
	ros::param::get("~vel_rot", _vel_rot); ros::param::get("~acc_rot", _acc_rot);
	ros::param::get("~jerk_flt", _jerk_flt);
	ros::param::get("~x_chn", _x_chn); ros::param::get("~x_min", _x_min); ros::param::get("~x_max", _x_max);
	ros::param::get("~y_chn", _y_chn); ros::param::get("~y_min", _y_min); ros::param::get("~y_max", _y_max);
	ros::param::get("~z_chn", _z_chn); ros::param::get("~z_min", _z_min); ros::param::get("~z_max", _z_max);
	ros::param::get("~yaw_chn", _yaw_chn);     ros::param::get("~yaw_min", _yaw_min);     ros::param::get("~yaw_max", _yaw_max);
	ros::param::get("~pitch_chn", _pitch_chn); ros::param::get("~pitch_min", _pitch_min); ros::param::get("~pitch_max", _pitch_max);
	ros::param::get("~roll_chn", _roll_chn);   ros::param::get("~roll_min", _roll_min);   ros::param::get("~roll_max", _roll_max);
	ros::param::get("/momav/m", _m); ros::param::get("/momav/Ix", _Ix); ros::param::get("/momav/Iy", _Iy); ros::param::get("/momav/Iz", _Iz);

	// setup published and subscribed topics
	sub_commander = n.subscribe("commander_state", 1, callback_commander);
	sub_remote = n.subscribe("remote", 1, callback_remote, ros::TransportHints().udp());
	sub_body_state = n.subscribe(_body_state_topic, 1, callback_body_state, ros::TransportHints().udp());
	pub_body_setp_pose = n.advertise<momav::BodySetp>("body_setp_pose", 1);

	//loop until node is stopped
	ros::spin();
	return 0;
}


// ################################################################################################


// callback for commander topic
void callback_commander(const momav::CommanderState::ConstPtr& msg) { 
	msg_commander = *(msg.get()); 
	READY = ( msg_commander.remote );
}

// callback for remote topic
void callback_remote(const momav::Remote::ConstPtr& msg) {
	msg_remote = *(msg.get());
}

// callback for body state topic (body_state_track, body_state_cam, or body_state_sim)
void callback_body_state(const momav::BodyState::ConstPtr& msg) {
	msg_body_state = *(msg.get());
	if (!READY) { return; }

	run_navigator();
	pub_body_setp_pose.publish(msg_body_setp_pose);
}


// ################################################################################################

bool run_navigator() {

	// track current position / orientation & their velocities
	static Vector3d r_curr = Vector3d(0.0, 0.0, 0.0);
	static Vector3d v_curr = Vector3d(0.0, 0.0, 0.0);
	static Vector3d a_curr = Vector3d(0.0, 0.0, 0.0);
	static Vector3d e_curr = Vector3d(0.0, 0.0, 0.0);
	static Vector3d de_curr = Vector3d(0.0, 0.0, 0.0);
	static Vector3d dde_curr = Vector3d(0.0, 0.0, 0.0);
	// hold current position / orientation setpoints that disregards max vel/accel
	Vector3d r_setp = Vector3d(0.0, 0.0, 0.0);
	Vector3d e_setp = Vector3d(0.0, 0.0, 0.0);
	// position offset for start of fixed trajectory
	static Vector3d traj_offset = Vector3d(0.0, 0.0, 0.0);
	// id along trajectory steps sequence (_traj_sequence)
	static int traj_step = -1;

	// get time since last run
	static double _h = 0.001;
	static auto t_old = std::chrono::high_resolution_clock::now();
	auto t_new = std::chrono::high_resolution_clock::now();
	double dt = ((double) std::chrono::duration_cast<std::chrono::microseconds>(t_new-t_old).count()) / 1.0e6;
	_h = 0.95*_h + 0.05*dt;
	t_old = t_new;

	// if fixed setpoint trajectory is active
	bool traj_active = (msg_remote.channels[_traj_run_chn] > 0.5);
	if (traj_active) {

		// save offset at start of fixed trajectory
		if (traj_step == -1) { traj_offset = r_curr; traj_step = 0; }

		// if position fixed trajectory is active
		if (msg_remote.channels[_traj_sel_chn] < 0.25) {
			r_setp = _traj_sequence[traj_step] * _traj_pos_dist + traj_offset;
			e_setp = Vector3d(0.0, 0.0, 0.0);
		}
		// if orientation fixed trajectory is active
		else if (msg_remote.channels[_traj_sel_chn] < 0.75) {
			r_setp = traj_offset;
			e_setp = _traj_sequence[traj_step] * _traj_rot_dist;
		}
		// if continuous rotation trajectory is active
		else {
			if (traj_step == 0) {
				r_setp = traj_offset;
				e_setp = Vector3d(0.0, 0.0, 10*2*M_PI);
			} else {
				r_setp = traj_offset;
				e_setp = Vector3d(0.0, 0.0, 0.0);
			}
		}

		// advance trajectory if setpoint reached
		if ( (r_setp-r_curr).norm() < 1e-6 && (e_setp-e_curr).norm() < 1e-6 ) {	traj_step++; }
		// stop trajectory at end
		if ( traj_step >= _traj_sequence_len ) { traj_step = _traj_sequence_len-1; }

	// if remote control is active
	} else {
		traj_step = -1;

		// get position setpoints from remote
		r_setp[0] = msg_remote.channels[_x_chn]*(_x_max-_x_min)+_x_min;
		r_setp[1] = msg_remote.channels[_y_chn]*(_y_max-_y_min)+_y_min;
		r_setp[2] = msg_remote.channels[_z_chn]*(_z_max-_z_min)+_z_min;

		// get orientation setpoints from remote
		e_setp[0] = (msg_remote.channels[_yaw_chn]*(_yaw_max-_yaw_min)+_yaw_min)*M_PI/180.0;
		e_setp[1] = (msg_remote.channels[_pitch_chn]*(_pitch_max-_pitch_min)+_pitch_min)*M_PI/180.0;
		e_setp[2] = (msg_remote.channels[_roll_chn]*(_roll_max-_roll_min)+_roll_min)*M_PI/180.0;
	}

	// set current position / orientation to setpoint on first run
	static bool first = true;
	if (first) { r_curr = r_setp; e_curr = e_setp; }
	first = false;

	// enforce max vel/accel
	if (traj_active) {
		for (int i=0; i<3; i++) { _enforce_vel_acc(r_curr[i], v_curr[i],  a_curr[i],   r_setp[i], _traj_pos_dist/_traj_pos_time, _traj_pos_dist/_traj_pos_time/2.0, _h); }
		for (int i=0; i<3; i++) { _enforce_vel_acc(e_curr[i], de_curr[i], dde_curr[i], e_setp[i], _traj_rot_dist/_traj_rot_time, _traj_rot_dist/_traj_rot_time/2.0, _h); }
	} else {
		for (int i=0; i<3; i++) { _enforce_vel_acc(r_curr[i], v_curr[i],  a_curr[i],   r_setp[i], _vel_pos, _acc_pos, _h); }
		for (int i=0; i<3; i++) { _enforce_vel_acc(e_curr[i], de_curr[i], dde_curr[i], e_setp[i], _vel_rot, _acc_rot, _h); }
	}

	// calculate rotation from rpy
	Quaterniond q_curr = AngleAxisd(e_curr[0], Vector3d::UnitZ()) * AngleAxisd(e_curr[1], Vector3d::UnitY()) * AngleAxisd(e_curr[2], Vector3d::UnitX());
	Matrix3d C = q_curr.toRotationMatrix();
	Matrix3d E; E << 0, -sin(e_curr[0]), cos(e_curr[1])*cos(e_curr[0]),
					 0, +cos(e_curr[0]), cos(e_curr[1])*sin(e_curr[0]),
					 1,               0,               -sin(e_curr[1]);
	Matrix3d Ib = DiagonalMatrix<double, 3>(_Ix, _Iy, _Iz);
	Matrix3d I  = C * Ib * C.transpose();
	Vector3d w_curr = E*de_curr;
	Vector3d y_curr = E*dde_curr;

	// calculate feedforward force/torque
	Vector3d f_prev = Vector3d(msg_body_setp_pose.force.x,msg_body_setp_pose.force.y,msg_body_setp_pose.force.z);
	Vector3d t_prev = Vector3d(msg_body_setp_pose.torque.x,msg_body_setp_pose.torque.y,msg_body_setp_pose.torque.z);
	Vector3d f_curr = _jerk_flt*( _m*a_curr ) + (1.0-_jerk_flt)*f_prev;
	Vector3d t_curr = _jerk_flt*( I*y_curr + w_curr.cross(I*w_curr) ) + (1.0-_jerk_flt)*t_prev;

	// submit new position / orientation setpoints
	msg_body_setp_pose.translation.x          = r_curr[0]; msg_body_setp_pose.translation.y          = r_curr[1]; msg_body_setp_pose.translation.z          = r_curr[2];
	msg_body_setp_pose.linear_velocity.x      = v_curr[0]; msg_body_setp_pose.linear_velocity.y      = v_curr[1]; msg_body_setp_pose.linear_velocity.z      = v_curr[2];
	msg_body_setp_pose.linear_acceleration.x  = a_curr[0]; msg_body_setp_pose.linear_acceleration.y  = a_curr[1]; msg_body_setp_pose.linear_acceleration.z  = a_curr[2];
	msg_body_setp_pose.force.x                = f_curr[0]; msg_body_setp_pose.force.y                = f_curr[1]; msg_body_setp_pose.force.z                = f_curr[2];
	msg_body_setp_pose.orientation.w = q_curr.w(); msg_body_setp_pose.orientation.x = q_curr.x(); msg_body_setp_pose.orientation.y = q_curr.y(); msg_body_setp_pose.orientation.z = q_curr.z();
	msg_body_setp_pose.angular_velocity.x     = w_curr[0]; msg_body_setp_pose.angular_velocity.y     = w_curr[1]; msg_body_setp_pose.angular_velocity.z     = w_curr[2];
	msg_body_setp_pose.angular_acceleration.x = y_curr[0]; msg_body_setp_pose.angular_acceleration.y = y_curr[1]; msg_body_setp_pose.angular_acceleration.z = y_curr[2];
	msg_body_setp_pose.torque.x               = t_curr[0]; msg_body_setp_pose.torque.y               = t_curr[1]; msg_body_setp_pose.torque.z               = t_curr[2];
	msg_body_setp_pose.setpoint_type = traj_step;
	msg_body_setp_pose.header.stamp = ros::Time::now();

	return true;
}


void _enforce_vel_acc(double& curr, double& vel, double& acc, double setp, double lim_vel, double lim_acc, double h) {

	double time_target = -(curr-setp) / (0.5*vel);
	double acc_target = -vel / time_target;
	double acc_max = std::copysign(lim_acc, setp-curr);

	// calculate desired acc
	if (std::isinf(time_target)) {						// velocity zero, position error non-zero
		acc = acc_max;
	} else if (std::isnan(time_target)) {				// velocity zero, position error zero
		acc = 0.0;
	} else if (time_target == 0.0) {					// velocity non-zero, position error zero
		acc = 0.0;
	} else if (time_target < 0.0) {						// just overshot target
		acc = acc_max;
	} else if (std::abs(acc_target) < 1.00*lim_acc) {	// will undershoot target with 100% decelleration
		if (std::abs(acc-acc_max) > 1.5*lim_acc) {
			acc = 0.0;
		} else {
			acc = acc_max;
		}
	} else if (std::abs(acc_target) < 1.01*lim_acc) {	// target reachable with 100-101% decelleration
		acc = acc_target;
	} else {											// will overshoot target with 101% decelleration
		acc = -acc_max;
	}

	// cap vel to limit
	if (std::abs(vel+acc*h) > lim_vel) {
		acc = (std::copysign(lim_vel, vel+acc*h) - vel)/h;
	}

	// update pos & vel
	curr += vel*h + 0.5*acc*h*h;
	vel += acc*h;

	// stop if setpoint reached and vel~=0
	if ( std::abs(vel) < 1.01*lim_acc*h && std::abs(curr-setp)<1e-4 ) {
		acc = 0.0;
		vel = 0.0;
		curr = setp;
	}

}
