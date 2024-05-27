
#include "ros/ros.h"
#include "momav/BodyState.h"
#include "momav/BodySetp.h"

#include <math.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
using namespace Eigen;
typedef Matrix<double,3,4> Matrix34d;

// ################################################################################################

// callbacks
void callback_body_setp(const momav::BodySetp::ConstPtr& msg);

// ros topics
ros::Subscriber sub_body_setp;  momav::BodySetp msg_body_setp;
ros::Publisher  pub_body_state; momav::BodyState msg_body_state;

// ros params
std::string _body_state_topic;
double _m, _Ix, _Iy, _Iz;		// mass and angular mass of momav
int _rate; double _h;
double _tresh_R, _max_eval, _factor;

// prototypes for simulation
//Vector3d residual_translation( Vector3d r, Vector3d rm1, Vector3d rm2, Vector3d rm3, Vector3d F );
Vector3d solve_translation( Vector3d rm1, Vector3d rm2, Vector3d rm3, Vector3d F );
Vector4d residual_rotation( Vector4d q, Vector4d qm1, Vector4d qm2, Vector4d qm3, Vector3d T );
Vector4d solve_rotation( Vector4d qm1, Vector4d qm2, Vector4d qm3, Vector3d T );

// prototypes for helper functions
Vector3d _d_BDF2(Vector3d rm0, Vector3d rm1, Vector3d rm2);
Vector3d _dd_BDF2(Vector3d rm0, Vector3d rm1, Vector3d rm2, Vector3d rm3);
Vector4d _d_BDF2(Vector4d qm0, Vector4d qm1, Vector4d qm2);
Vector4d _dd_BDF2(Vector4d qm0, Vector4d qm1, Vector4d qm2, Vector4d qm3);
Matrix3d  _get_C(Vector4d q);
Matrix34d _get_E(Vector4d q);

// ################################################################################################

int main(int argc, char **argv) {
	
	//setup ros node
	ros::init(argc, argv, "driver_sim");
	ros::NodeHandle n;
	sub_body_setp  = n.subscribe("body_setp_wrench", 1, callback_body_setp);
	pub_body_state = n.advertise<momav::BodyState>("body_state_sim", 1);
	ros::param::get("/momav/m", _m);
	ros::param::get("/momav/Ix", _Ix);
	ros::param::get("/momav/Iy", _Iy);
	ros::param::get("/momav/Iz", _Iz);
	ros::param::get("~rate", _rate); _h = 1.0/((double)_rate);
	ros::param::get("~tresh_R", _tresh_R);
	ros::param::get("~max_eval", _max_eval);
	ros::param::get("~factor", _factor);

	// initialize setpoint force/torque
	msg_body_setp.force.x=0.0;  msg_body_setp.force.y=0.0;  msg_body_setp.force.z=0.0;
	msg_body_setp.torque.x=0.0; msg_body_setp.torque.y=0.0; msg_body_setp.torque.z=0.0;

	// translation/orientation state for last timesteps
	Vector3d r(0,0,0), rm1(0,0,0), rm2(0,0,0), rm3(0,0,0);
	Vector4d q(1,0,0,0), qm1(1,0,0,0), qm2(1,0,0,0), qm3(1,0,0,0);
	// current force/torque
	Vector3d F(0,0,0), T(0,0,0);


	//main loop
	ros::Rate rate(_rate);
	while (ros::ok()) {

		// shift states from previous loop
		rm3 = rm2; rm2 = rm1; rm1 = r;
		qm3 = qm2; qm2 = qm1; qm1 = q;

		// get current force/torque
		F = Vector3d( msg_body_setp.force.x,  msg_body_setp.force.y,  msg_body_setp.force.z  );
		T = Vector3d( msg_body_setp.torque.x, msg_body_setp.torque.y, msg_body_setp.torque.z );

		// add a tiny bit of noise
		F += Vector3d::Random()*0.1;
		T += Vector3d::Random()*0.01;

		// compute next state
		r = solve_translation(rm1, rm2, rm3, F);
		q = solve_rotation(qm1, qm2, qm3, T);

		// enforce ground
		if (r.z() < 0.0) { r.z() = 0.0; }

		// compute derivatives
		Vector3d dr, ddr;
		dr  = _d_BDF2(r, rm1, rm2);
		ddr = _dd_BDF2(r, rm1, rm2, rm3);
		Vector4d dq, ddq; Matrix34d E; Vector3d w, dw;
		dq  = _d_BDF2(q, qm1, qm2);
		ddq = _dd_BDF2(q, qm1, qm2, qm3);
		E = _get_E(q);
		w  = E*dq;
		dw = E*ddq;

		// copy over results
		msg_body_state.translation.x          = r[0];   msg_body_state.translation.y          = r[1];   msg_body_state.translation.z           = r[2];
		msg_body_state.linear_velocity.x      = dr[0];  msg_body_state.linear_velocity.y      = dr[1];  msg_body_state.linear_velocity.z       = dr[2];
		msg_body_state.linear_acceleration.x  = ddr[0]; msg_body_state.linear_acceleration.y  = ddr[1]; msg_body_state.linear_acceleration.z   = ddr[2];
		msg_body_state.orientation.w          = q[0];   msg_body_state.orientation.x          = q[1];   msg_body_state.orientation.y          = q[2];    msg_body_state.orientation.z = q[3];
		msg_body_state.angular_velocity.x     = w[0];   msg_body_state.angular_velocity.y     = w[1];   msg_body_state.angular_velocity.z     = w[2];
		msg_body_state.angular_acceleration.x = dw[0];  msg_body_state.angular_acceleration.y = dw[1];  msg_body_state.angular_acceleration.z = dw[2];

		// publish result
		msg_body_state.header.stamp = ros::Time::now();
		pub_body_state.publish(msg_body_state);
		ros::spinOnce(); rate.sleep(); ros::spinOnce();
	}

	return 0;
}

// callback for body setpoint wrench topic
void callback_body_setp(const momav::BodySetp::ConstPtr& msg) { 
	msg_body_setp = *(msg.get());
}

// ################################################################################################

Vector3d _d_BDF2(Vector3d rm0, Vector3d rm1, Vector3d rm2) {
	return 1/_h * (1.5*rm0 - 2.0*rm1 + 0.5*rm2);
}
Vector3d _dd_BDF2(Vector3d rm0, Vector3d rm1, Vector3d rm2, Vector3d rm3) {
	return 1/(_h*_h) * (2.0*rm0 - 5.0*rm1 + 4.0*rm2 - 1.0*rm3);
}
Vector4d _d_BDF2(Vector4d qm0, Vector4d qm1, Vector4d qm2) {
	return 1/_h * (1.5*qm0 - 2.0*qm1 + 0.5*qm2);
}
Vector4d _dd_BDF2(Vector4d qm0, Vector4d qm1, Vector4d qm2, Vector4d qm3) {
	return 1/(_h*_h) * (2.0*qm0 - 5.0*qm1 + 4.0*qm2 - 1.0*qm3);
}

Matrix3d _get_C(Vector4d q) {
	Matrix3d C;
	C << q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3],  2.0*q[1]*q[2] - 2.0*q[0]*q[3],                  2.0*q[0]*q[2] + 2.0*q[1]*q[3],
	     2.0*q[0]*q[3] + 2.0*q[1]*q[2],                  q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3],  2.0*q[2]*q[3] - 2.0*q[0]*q[1],
	     2.0*q[1]*q[3] - 2.0*q[0]*q[2],                  2.0*q[0]*q[1] + 2.0*q[2]*q[3],                  q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
	return C;
}
Matrix34d _get_E(Vector4d q) {
	Matrix34d E;
	E << -q[1], +q[0], -q[3], +q[2],
	     -q[2], +q[3], +q[0], -q[1],
	     -q[3], -q[2], +q[1], +q[0];
	return 2.0*E;
}

// Generic functor
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor {

	typedef _Scalar Scalar;
	enum {
	    InputsAtCompileTime = NX,
	    ValuesAtCompileTime = NY
	};
	typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
	typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
	typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

	int m_inputs, m_values;

	Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
	Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

	int inputs() const { return m_inputs; }
	int values() const { return m_values; }

};

// ################################################################################################

/*Vector3d residual_translation( Vector3d r, Vector3d rm1, Vector3d rm2, Vector3d rm3, Vector3d F ) {

	// translation derivatives (bdf2)
	Vector3d dr, ddr;
	dr  = _d_BDF2(r, rm1, rm2);
	ddr = _dd_BDF2(r, rm1, rm2, rm3);

	// linear momentum derivatives
	Vector3d p, dp;
	p  = _m * dr;
	dp = _m * ddr;

	// EoM residual
	Vector3d R;
	R = dp - F;

	return R;
}*/

Vector3d solve_translation( Vector3d rm1, Vector3d rm2, Vector3d rm3, Vector3d F ) {

	Vector3d r;
	r = 0.5*( (F/_m*_h*_h) + 5.0*rm1 - 4.0*rm2 + 1.0*rm3 );

	return r;
}

// ################################################################################################

Vector4d residual_rotation( Vector4d q, Vector4d qm1, Vector4d qm2, Vector4d qm3, Vector3d T ) {

	// rotation vector derivatives (bdf2)
	Vector4d dq, ddq;
	dq  = _d_BDF2(q, qm1, qm2);
	ddq = _dd_BDF2(q, qm1, qm2, qm3);

	// rotation matrix & derivatives mapping
	Matrix3d C = _get_C(q);
	Matrix34d E = _get_E(q);

	// rotation velocity & acceleration
	Vector3d w, dw;
	w  = E*dq;
	dw = E*ddq;

	// moment of inertia
	Matrix3d Ib, I;
	Ib = DiagonalMatrix<double, 3>(_Ix, _Iy, _Iz);
	I  = C * Ib * C.transpose();

	// angular momentum & derivative
	Vector3d L, dL;
	L  = I*w;
	dL = I*dw + w.cross(I*w);

	// EoM residual
	Vector4d R;
	R.segment<3>(0) = dL - T;
	R[3] = (q.norm()-1.0)*1.0e6;

	return R;
}
struct functor_rotation : Functor<double> {

	Vector4d qm1, qm2, qm3; Vector3d T;
	functor_rotation(Vector4d _qm1, Vector4d _qm2, Vector4d _qm3, Vector3d _T) :
	Functor<double>(4,4), qm1(_qm1),qm2(_qm2),qm3(_qm3),T(_T) {}

	int operator() (const VectorXd& q, VectorXd& R) const {
		R = residual_rotation(q, qm1, qm2, qm3, T);
		return 0;
	}
};

Vector4d solve_rotation( Vector4d qm1, Vector4d qm2, Vector4d qm3, Vector3d T ) {

	// initialize solver
	VectorXd q(4); q = qm1;
	functor_rotation functor(qm1, qm2, qm3, T);
	HybridNonLinearSolver<functor_rotation> solver(functor);
	solver.parameters.xtol = _tresh_R;
	solver.parameters.maxfev = _max_eval;
	solver.parameters.factor = _factor;

	// run solver
	int info = solver.solveNumericalDiff(q);
	if (info != 1) { ROS_WARN("[driver_sim] failed to simulate drone movement"); }

	return q;
}

// ################################################################################################