
#include "ros/ros.h"
#include "momav/BodyState.h"

#include <math.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
using namespace Eigen;

#include <librealsense2/rs.hpp>

// ################################################################################################

// ros topics
ros::Publisher pub_body_state;
momav::BodyState msg_body_state;

// ros params
std::string _serial_dev;
Vector3d _pos;
Matrix3d _rot;

// ################################################################################################

int main(int argc, char **argv) try {
	
	//setup ros node
	ros::init(argc, argv, "driver_cam");
	ros::NodeHandle n;
	ros::Rate rate(1000);
	pub_body_state = n.advertise<momav::BodyState>("body_state_cam", 1);
	ros::param::get("~dev_id", _serial_dev);
	std::string yaml_str; YAML::Node yaml_obj;
	ros::param::get("~pos", yaml_str); yaml_obj = YAML::Load(yaml_str);
	_pos << yaml_obj[0].as<double>(), yaml_obj[1].as<double>(), yaml_obj[2].as<double>();
	ros::param::get("~rot", yaml_str); yaml_obj = YAML::Load(yaml_str);
	_rot << yaml_obj[0][0].as<double>(), yaml_obj[0][1].as<double>(), yaml_obj[0][2].as<double>(),
			yaml_obj[1][0].as<double>(), yaml_obj[1][1].as<double>(), yaml_obj[1][2].as<double>(),
			yaml_obj[2][0].as<double>(), yaml_obj[2][1].as<double>(), yaml_obj[2][2].as<double>();

	// boot offsets
	Matrix3d R_offset;
	Vector3d r_offset;

	//connect to realsense t265
	rs2::pipeline pipe;
	rs2::config cfg;
	rs2::frameset frames;
	cfg.enable_device(_serial_dev);
	cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
	pipe.start(cfg);

	//main loop
	int fails = 0; bool init = false;
	while (ros::ok()) {
	
		bool success = pipe.poll_for_frames(&frames);
		if (!success) { ros::spinOnce(); rate.sleep(); ros::spinOnce(); fails++; continue; } else { fails=0; }
		if (fails > 500) ROS_ERROR("[driver_cam] timeout!");
		rs2::frame frame = frames.first_or_default(RS2_STREAM_POSE);
		rs2_pose pose_data = frame.as<rs2::pose_frame>().get_pose_data();

		// translation / rotation
		Vector3d r = Vector3d(pose_data.translation.x,          -pose_data.translation.z,          pose_data.translation.y);
		Vector3d v = Vector3d(pose_data.velocity.x,             -pose_data.velocity.z,             pose_data.velocity.y);
		Vector3d a = Vector3d(pose_data.acceleration.x,         -pose_data.acceleration.z,         pose_data.acceleration.y);
		Vector3d w = Vector3d(pose_data.angular_velocity.x,     -pose_data.angular_velocity.z,     pose_data.angular_velocity.y);
		Vector3d y = Vector3d(pose_data.angular_acceleration.x, -pose_data.angular_acceleration.z, pose_data.angular_acceleration.y);

		// transform rotation coordinate frame
		// C: camera, B: body, I: inertial, J: rs2 inertial
		Quaterniond q_JC = Quaterniond(pose_data.rotation.w,pose_data.rotation.x,pose_data.rotation.y,pose_data.rotation.z); q_JC.normalize();
		Matrix3d R_JC = Matrix3d(q_JC);
		Matrix3d R_IJ; R_IJ << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0;
		Matrix3d R_CB = _rot.transpose();
		Matrix3d R_IB = R_IJ*R_JC*R_CB;

		// boot offsets
		if(!init) {
			r_offset = r - _pos;
			R_offset = R_IB.transpose();
			init = true; 
		}
		r = r - r_offset;
		R_IB = R_offset * R_IB;
		Quaterniond q_ib = Quaterniond(R_IB);

		// rigid body correction
		r = r + R_IB*(-_pos);
		v = v + w.cross(R_IB*(-_pos));
		a = a + y.cross(R_IB*(-_pos)) + w.cross(w.cross(R_IB*(-_pos)));

		// copy over results
		msg_body_state.orientation.x = q_ib.x();      msg_body_state.orientation.y = q_ib.y();      msg_body_state.orientation.z = q_ib.z();      msg_body_state.orientation.w = q_ib.w();
		msg_body_state.translation.x = r(0);          msg_body_state.translation.y = r(1);          msg_body_state.translation.z = r(2);
		msg_body_state.linear_velocity.x = v(0);      msg_body_state.linear_velocity.y = v(1);      msg_body_state.linear_velocity.z = v(2);
		msg_body_state.angular_velocity.x = w(0);     msg_body_state.angular_velocity.y = w(1);     msg_body_state.angular_velocity.z = w(2);
		msg_body_state.linear_acceleration.x = a(0);  msg_body_state.linear_acceleration.y = a(1);  msg_body_state.linear_acceleration.z = a(2);
		msg_body_state.angular_acceleration.x = y(0); msg_body_state.angular_acceleration.y = y(1); msg_body_state.angular_acceleration.z = y(2);

		msg_body_state.header.stamp = ros::Time::now();
		pub_body_state.publish(msg_body_state);
		ros::spinOnce();
	}

	return 0;
}
catch (const rs2::error & e) {
	ROS_ERROR("[driver_cam] realsense error calling %s: %s", e.get_failed_function().c_str(), e.what());
	return 0;
}
catch (const std::exception& e) {
	ROS_ERROR("[driver_cam] unknown error: %s", e.what());
	return 0;
}
