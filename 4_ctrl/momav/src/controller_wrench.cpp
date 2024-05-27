
#include "ros/ros.h"
#include "momav/CommanderState.h"
#include "momav/BodyState.h"
#include "momav/BodySetp.h"
#include "momav/MotorSetp.h"
#include "momav/ServoSetp.h"

#include <chrono>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <math.h>
#include <Eigen/Dense>
using namespace Eigen;
typedef Matrix<double,18,18> Matrix18d;
typedef Matrix<double,18,1> Vector18d;
typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,6,1> Vector6d;

// ################################################################################################

// general constants
std::string _body_state_topic;  // topic used as body_state input (body_state_track, body_state_cam, or body_state_sim)
double _m;						// body mass
// geometry constants
Vector3d _r[6];					// position of arms [m]
Vector3d _x[6];					// rotation axes of arms
Vector3d _z[6];					// zero rotation direction of arms
Vector3d _y[6];					// 90° rotation direction of arms
// motor constants
double _cf[6];					// _cf: motor force constant [N]
double _cm[6];					// _cm: motor torque constant [Nm]
// objective & residual constants
double _uw, _upw, _uph, _upl;	// _uw: throttle weight, _upw: throttle limit penalty; _uph&_upl: higher&lower bounds for penalty
double _aw, _apw, _aph;			// _aw: arm angle speed weight, _apw: arm angle speed penalty, _aph: arm angle speed higher bound
// solver constants
double _du_max, _da_max;		// _du_max & _da_max: max u & a update for 1 iteration
double _tresh_G, _tresh_O;		// break treshold for constraint G norm and objective change |O-O_prev|/O_prev
int _max_iter;					// max # of iterations before giving up
int _max_fail;					// max # of times convergence can fail before shutting down node

// ################################################################################################

//callbacks
void callback_commander(const momav::CommanderState::ConstPtr& msg);
void callback_body_state(const momav::BodyState::ConstPtr& msg);
void callback_body_setp(const momav::BodySetp::ConstPtr& msg);

//ros publishers and subscribers
ros::Subscriber  sub_commander;  momav::CommanderState msg_commander;
ros::Subscriber  sub_body_state; momav::BodyState msg_body_state;
ros::Subscriber  sub_body_setp;  momav::BodySetp msg_body_setp;
ros::Publisher   pub_motor_setp; momav::MotorSetp msg_motor_setp;
ros::Publisher   pub_servo_setp; momav::ServoSetp msg_servo_setp;

// global variables
bool READY = false;		// flag if controller is ready to be run

// return codes
enum class RETURN_CODE {
    SUCCESS,
    ERROR_NAN,
    ERROR_U_RANGE,
    ERROR_MAX_ITER
};
// run wrench controler to generate msg_motor_setp and msg_servo_setp
bool run_controller();
// precise controller solving min O(u,a) subj.to: G(u,a)=0
RETURN_CODE precise_controller(Vector18d &ual, Vector18d ual_last, Vector3d F_b, Vector3d M_b);
// simple controller solving min u*u subj.to: G(u,a)=0 (linear program)
RETURN_CODE simple_controller(Vector18d &ual, Vector18d ual_last, Vector3d F_b, Vector3d M_b);

// ################################################################################################

// program entry point
int main(int argc, char **argv) {

	// setup ros node
	ros::init(argc, argv, "controller_wrench");
	ros::NodeHandle n;

	// load parameters
	ros::param::get("/momav/body_state", _body_state_topic);
	ros::param::get("/momav/m", _m);
	ros::param::get("~uw", _uw); ros::param::get("~upw", _upw); 
	ros::param::get("~uph", _uph); ros::param::get("~upl", _upl);
	ros::param::get("~aw", _aw); ros::param::get("~apw", _apw); 
	ros::param::get("~aph", _aph);
	ros::param::get("~du_max", _du_max); ros::param::get("~da_max", _da_max); 
	ros::param::get("~tresh_G", _tresh_G); ros::param::get("~tresh_O", _tresh_O);
	ros::param::get("~max_iter", _max_iter); ros::param::get("~max_fail", _max_fail);
	std::string yaml_str; YAML::Node yaml_obj;
	ros::param::get("~r", yaml_str); yaml_obj = YAML::Load(yaml_str);
	for (int i=0; i<6; i++) { _r[i] = Vector3d(yaml_obj[i][0].as<double>(), yaml_obj[i][1].as<double>(), yaml_obj[i][2].as<double>()); }
	ros::param::get("~x", yaml_str); yaml_obj = YAML::Load(yaml_str);
	for (int i=0; i<6; i++) { _x[i] = Vector3d(yaml_obj[i][0].as<double>(), yaml_obj[i][1].as<double>(), yaml_obj[i][2].as<double>()).normalized(); }
	ros::param::get("~z", yaml_str); yaml_obj = YAML::Load(yaml_str);
	for (int i=0; i<6; i++) { _z[i] = Vector3d(yaml_obj[i][0].as<double>(), yaml_obj[i][1].as<double>(), yaml_obj[i][2].as<double>()).normalized(); }
	ros::param::get("~cf", yaml_str);  yaml_obj = YAML::Load(yaml_str);
	for (int i=0; i<6; i++) { _cf[i] = yaml_obj[i].as<double>(); }
	ros::param::get("~cm", yaml_str); yaml_obj = YAML::Load(yaml_str);
	for (int i=0; i<6; i++) { _cm[i] = yaml_obj[i].as<double>(); }
	for (int i=0; i<6; i++) { _y[i] = _x[i].cross(_z[i]); }

	// setup published and subscribed topics
	sub_commander = n.subscribe("commander_state", 1, callback_commander);
	sub_body_state = n.subscribe(_body_state_topic, 1, callback_body_state, ros::TransportHints().udp());
	sub_body_setp = n.subscribe("body_setp_wrench", 1, callback_body_setp);
	pub_motor_setp = n.advertise<momav::MotorSetp>("motor_setp", 1);
	pub_servo_setp = n.advertise<momav::ServoSetp>("servo_setp", 1);

	// init data
	for(int i=0; i<6; i++) { msg_motor_setp.throttle[i] = 0.0; }
	for(int i=0; i<6; i++) { msg_servo_setp.setpoint[i] = 0.0; }

	// loop until node is stopped
	ros::spin();
	return 0;
}


// ################################################################################################

// callback for commander topic
void callback_commander(const momav::CommanderState::ConstPtr& msg) { 
	msg_commander = *(msg.get()); 
	READY = ( msg_commander.body_state && msg_commander.body_setp_wrench );
}

// callback for body state topic
void callback_body_state(const momav::BodyState::ConstPtr& msg) {

	// check if messages have been missed
	/*if (msg_body_state.header.seq != 0 && msg.get()->header.seq-1 > msg_body_state.header.seq+4) {
		ROS_WARN("[controller_wrench] controller is slow. body_state messages dropped!");
	}*/

	// read body state message
	msg_body_state = *(msg.get());
	if(!READY) { return; }

	// run controller to generate msg_motor_setp & msg_servo_setp
	if(!run_controller()) { return; }

	// publish result
	pub_motor_setp.publish(msg_motor_setp);
	pub_servo_setp.publish(msg_servo_setp);
}

// callback for wrench setpoint topic
void callback_body_setp(const momav::BodySetp::ConstPtr& msg) { 
	msg_body_setp = *(msg.get());
}


// ################################################################################################

// run wrench controler to generate msg_motor_setp and msg_servo_setp
bool run_controller() {

	// initialize result vector
	static Vector18d ual = Vector18d::Zero();	// stacked vector of throttle setting (u) arm angles (a) and constraint multipliers (l) of last controller run
	Vector18d ual_last = ual;					// copy of last ual in case needed for resets
	auto u = ual.segment<6>(0); 				// map to different parts of ula
	auto a = ual.segment<6>(6); 				//
	auto l = ual.segment<6>(12); 				//

	// get force/torque setpoints in body frame
	Quaterniond q_b(msg_body_state.orientation.w, msg_body_state.orientation.x, msg_body_state.orientation.y, msg_body_state.orientation.z);
	Vector3d F_b = q_b.inverse() * Vector3d(msg_body_setp.force.x, msg_body_setp.force.y, msg_body_setp.force.z + _m*9.80665);
	Vector3d M_b = q_b.inverse() * Vector3d(msg_body_setp.torque.x, msg_body_setp.torque.y, msg_body_setp.torque.z);

	// counter tracking how many consecutive times the controller failed
	static int FAILS = -1;
	if (FAILS == -1)      { simple_controller(ual, ual_last, F_b, M_b); ual_last = ual; } // initialize ual for first run
	if (FAILS>=_max_fail) { return false; } // shut down if to many fails

	// run controller
	RETURN_CODE code = precise_controller(ual, ual_last, F_b, M_b);

	// handle errors
	if (code == RETURN_CODE::SUCCESS) { 
		FAILS = 0;
	} else {
		simple_controller(ual, ual_last, F_b, M_b); // use simple solution instead
		FAILS++;

		if (code == RETURN_CODE::ERROR_MAX_ITER) 	 { ROS_ERROR("[controller_wrench] controller failed. exceeded maximum iterations!"); }
		else if (code == RETURN_CODE::ERROR_U_RANGE) { ROS_ERROR("[controller_wrench] controller failed. throttle out of range!"); }
		else if (code == RETURN_CODE::ERROR_NAN) 	 { ROS_ERROR("[controller_wrench] controller failed. result contains NaN!"); }
		if (FAILS==_max_fail) {
			ROS_ERROR("[controller_wrench] controller is broken. shutting down!"); 
			u = Vector6d::Zero();
		}
	}

	// submit result
	msg_motor_setp.header.stamp = ros::Time::now();
	msg_servo_setp.header.stamp = ros::Time::now();
	for(int i=0; i<6; i++) {
		msg_motor_setp.throttle[i] = u(i);
		msg_servo_setp.setpoint[i] = a(i);
	}

	return true;
}

// ################################################################################################

// precise controller solving "min O(u,a) subj.to: G(u,a)=0"
RETURN_CODE precise_controller(Vector18d &ual, Vector18d ual_last, Vector3d F_b, Vector3d M_b) {
	//auto t0 = std::chrono::high_resolution_clock::now();

	auto u = ual.segment<6>(0); 		// map to different parts of ula: throttle (u), arm angle (a) & lagrange multip. (l)
	auto a = ual.segment<6>(6); 		//
	auto l = ual.segment<6>(12); 		//

	double O = 0.0, O_prev = 0.0;		// current objective & objective of previous iteration
	Vector6d G = Vector6d::Zero();		// current constraint residuals
	Matrix18d H = Matrix18d::Zero();	// current hessian
	Vector18d K = Vector18d::Zero();	// current gradient

	auto a_last = ual_last.segment<6>(6);// map to arm angle of last controller run
	Vector3d n[6];						// current arm force directions n (multiple uses -> larger scope)
	double a_vel[6];					// current arm rotation velocity (multiple uses -> larger scope)
	static double h = 1.0;				// estimate of controller update rate (start at 1hz, to slow down arm velocity estimate)

	LDLT<Matrix18d> solver(18);			// LDLT linear system solver object (works even though H is not positive semi definite?!)

	// get controller update rate
	static auto t_old = std::chrono::high_resolution_clock::now();
	auto t_new = std::chrono::high_resolution_clock::now();
	double dt = ((double) std::chrono::duration_cast<std::chrono::microseconds>(t_new-t_old).count()) / 1.0e6;
	h = 0.95*h + 0.05*dt;
	t_old = t_new;

	// iterate solver until converged or max iter reached
	int iter = 0;
	while(iter < _max_iter) {

		//reset summing variables
		O = 0.0;
		G.setZero();
		double O_part = 0.0;

		// calculate G & O to check convergence
		for (int i=0; i<6; i++) {

			// calculate arm thrust direction n
			double sin_a2 = sin(a(i)/2.0), cos_a2 = cos(a(i)/2.0);
			auto q = Quaterniond(cos_a2, _x[i](0)*sin_a2, _x[i](1)*sin_a2, _x[i](2)*sin_a2);
			n[i] = q*_z[i];

			// calculate arm force vector f
			auto f = _cf[i]*u(i)*n[i];

			// calculate arm torque vector m
			double m_sign = i%2*2-1;
			auto m = _cf[i]*u(i)*_r[i].cross(n[i]) + m_sign*_cm[i]*u(i)*n[i];

			// calculate arm velocity
			a_vel[i] = (a(i)-a_last(i))/h;

			// calculate arm constraint G contribution
			G.segment<3>(0) += f;
			G.segment<3>(3) += m;

			// calculate arm objective O contribution
			O += _uw*u(i)*u(i) + _aw*a_vel[i]*a_vel[i];
			O_part += _aw*a_vel[i]*a_vel[i];
			if (u(i) > _uph) {
				O += _upw*(u(i)-_uph)*(u(i)-_uph);
			} else if (u(i) < _upl) {
				O += _upw*(u(i)-_upl)*(u(i)-_upl);
			}
			if ( a_vel[i] > _aph ) {
				O += _apw*(a_vel[i]-_aph)*(a_vel[i]-_aph);
			} else if ( a_vel[i] < -_aph ) {
				O += _apw*(a_vel[i]+_aph)*(a_vel[i]+_aph);
			}

		}
		// add setpoint force/torque contributionto constraint G
		G.segment<3>(0) -= F_b; G.segment<3>(3) -= M_b;

		// break if solution converged
		if (G.norm() < _tresh_G && abs(O_prev-O)/O_prev < _tresh_O) { break; }

		// calculate H & K for update step
		for (int i=0; i<6; i++) {

			// calculate arm force direction n derivatives
			auto n_a = _x[i].cross(n[i]);
			auto n_aa = _x[i].cross(n_a);

			// calculate arm force vector f derivatives
			auto f_u = _cf[i]*n[i]; 
			auto f_uu = Vector3d::Zero();
			auto f_a = _cf[i]*u(i)*n_a;
			auto f_aa = _cf[i]*u(i)*n_aa;
			auto f_ua = _cf[i]*n_a;

			// calculate arm torque vector m derivatives
			double m_sign = i%2*2-1;
			auto m_u = _cf[i]*_r[i].cross(n[i]) + m_sign*_cm[i]*n[i];
			auto m_uu = Vector3d::Zero();
			auto m_a = _cf[i]*u(i)*_r[i].cross(n_a) + m_sign*_cm[i]*u(i)*n_a;
			auto m_aa = _cf[i]*u(i)*_r[i].cross(n_aa) + m_sign*_cm[i]*u(i)*n_aa;
			auto m_ua = _cf[i]*_r[i].cross(n_a) + m_sign*_cm[i]*n_a;

			// calculate arm constraint G contribution derivatives
			Vector6d G_u, G_uu, G_a, G_aa, G_ua;
			G_u.segment<3>(0)  = f_u;  G_u.segment<3>(3)  = m_u;
			G_uu.segment<3>(0) = f_uu; G_uu.segment<3>(3) = m_uu;
			G_a.segment<3>(0)  = f_a;  G_a.segment<3>(3)  = m_a;
			G_aa.segment<3>(0) = f_aa; G_aa.segment<3>(3) = m_aa;
			G_ua.segment<3>(0) = f_ua; G_ua.segment<3>(3) = m_ua;

			// calculate arm objective O contribution derivatives
			auto O_u = 2*_uw*u(i);
			auto O_uu = 2*_uw;
			auto O_a = 2*_aw*a_vel[i]/h;
			auto O_aa = 2*_aw/(h*h);
			auto O_ua = 0.0;
			if (u(i) > _uph) {
				O_u += 2*_upw*(u(i)-_uph);
				O_uu += 2*_upw;
			} else if (u(i) < _upl) {
				O_u += 2*_upw*(u(i)-_upl);
				O_uu += 2*_upw;
			}
			if (a_vel[i] > _aph) {
				O_a += 2*_apw*(a_vel[i]-_aph)/h;
				O_aa += 2*_apw/(h*h);
			} else if (a_vel[i] < -_aph) {
				O_a += 2*_apw*(a_vel[i]+_aph)/h;
				O_aa += 2*_apw/(h*h);
			}

			// assemble gradient K
			K(i+0)  = O_u;
			K(i+6)  = O_a;

			// assemble hessian H
			H(i+0,i+0) = O_uu + G_uu.transpose()*l;
			H(i+6,i+6) = O_aa + G_aa.transpose()*l;
			H(i+0,i+6) = O_ua + G_ua.transpose()*l;
			H(i+6,i+0) = O_ua + G_ua.transpose()*l;
			H.block<1,6>(i+0,12) = G_u.transpose();
			H.block<1,6>(i+6,12) = G_a.transpose();
			H.block<6,1>(12,i+0) = G_u;
			H.block<6,1>(12,i+6) = G_a;

		}
		// add constraint G term to gradient K
		K.segment<6>(12) = G;

		// calculate update step
		solver.compute(H);
		Vector18d dual = solver.solve(-K); 
		auto du = dual.segment<6>(0); 	// map to different parts of dual
		auto da = dual.segment<6>(6); 	//
		auto dl = dual.segment<6>(12);	//
		dl -= l;

		//perform update step
		double alpha = std::min(1.0, std::min( _du_max/du.cwiseAbs().maxCoeff() , _da_max/da.cwiseAbs().maxCoeff()));
		ual = ual + alpha*dual;

		//ROS_WARN("iter: %d, alpha: %f, O(u): %f, O(a): %f, G: %f", iter, alpha, O-O_part, O_part, G.norm());
		//ROS_WARN("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,   %.2f,%.2f,%.2f,%.2f,%.2f,%.2f", u(0),u(1),u(2),u(3),u(4),u(5), a(0),a(1),a(2),a(3),a(4),a(5));

		//prepare for next iteration
		O_prev = O;
		iter++;
	}

	// print compact result
	//ROS_WARN("iter: %d", iter);
	//ROS_WARN("vel: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", a_vel[0],a_vel[1],a_vel[2],a_vel[3],a_vel[4],a_vel[5]);
	//ROS_WARN("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,   %.2f,%.2f,%.2f,%.2f,%.2f,%.2f", u(0),u(1),u(2),u(3),u(4),u(5), a(0),a(1),a(2),a(3),a(4),a(5));

	// print solving time
	//auto t1 = std::chrono::high_resolution_clock::now();
	//unsigned int d1 = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
	//ROS_WARN("iter: %d, time: %d", iter, d1);

	// do not submit if results contain NAN
	if(isnan(ual.sum()))  { return RETURN_CODE::ERROR_NAN; }
	// do not submit if throttle out of range
	if(u.maxCoeff()>1.0 || u.minCoeff()<0.0) { return RETURN_CODE::ERROR_U_RANGE; }
	// do not submit result if convergence failed
	if(iter == _max_iter) {	return RETURN_CODE::ERROR_MAX_ITER; }

	return RETURN_CODE::SUCCESS;
}

// simple controller solving "min u*u subj.to: G(u,a)=0" (linear program)
RETURN_CODE simple_controller(Vector18d &ual, Vector18d ual_last, Vector3d F_b, Vector3d M_b) {

	auto u = ual.segment<6>(0); 	// map to different parts of ula: stacked throttle (u), arm angle (a) & lagrange multip. (l)
	auto a = ual.segment<6>(6); 	//
	auto l = ual.segment<6>(12); 	//
	Vector6d a_last = ual_last.segment<6>(6);	// map to arm angle of last controller run

	// fill in allocation matrix K
	static bool first = true;
	static Matrix<double,12,6> K_inv;
	if (first) {
		Matrix<double,6,12> K(6,12);
		for(int i=0; i<6; i++) {
			double m_sign = i%2*2-1;
			K.block<3,1>(0,i) = _cf[i]*_z[i];
			K.block<3,1>(3,i) = _cf[i]*_r[i].cross(_z[i]) + m_sign*_cm[i]*_z[i];
			K.block<3,1>(0,i+6) = _cf[i]*_y[i];
			K.block<3,1>(3,i+6) = _cf[i]*_r[i].cross(_y[i]) + m_sign*_cm[i]*_y[i];
		}
		K_inv = K.transpose()*(K*K.transpose()).inverse();
		first = false;
	}

	// calculate leas-square solution for cs
	Vector6d wrench; wrench << F_b, M_b;
	VectorXd cs = K_inv*wrench;
	auto c = cs.segment<6>(0);
	auto s = cs.segment<6>(6);

	// compute new ual
	for (int i=0; i<6; i++) {
		u(i) = sqrt(c(i)*c(i) + s(i)*s(i));
		a(i) = atan2(s(i),c(i));
		l(i) = 0.0;

		// correct for discontinuities at PI/2, -PI/2
		if(!isnan(a_last.sum())) {
			a(i) += round(a_last(i)/(2*M_PI))*2*M_PI;
			if (a(i)-a_last(i) >  M_PI) { a(i) -= 2*M_PI; }
			if (a(i)-a_last(i) < -M_PI) { a(i) += 2*M_PI; }
		}
	}

	return RETURN_CODE::SUCCESS;
}

