
#include "ros/ros.h"
#include "momav/BodyState.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

#include <math.h>
#include <Eigen/Dense>
using namespace Eigen;

// xsens mti3
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>

// bmp390
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
extern "C" {
	#include <linux/i2c.h>
	#include <linux/i2c-dev.h>
	#include <i2c/smbus.h>
}

// ################################################################################################

// ros topics
ros::Publisher pub_body_state;
momav::BodyState msg_body_state;
momav::BodyState last_body_state;

// ros params
std::string _body_state_topic;
std::string _serial_dev, _i2c_dev;
double _gyro_cal_time;

void _error(std::string msg);


// ################################################################################################

// BMP390 stuff
#define BMP390_P_DATA_PA	uint8_t(0x04)
#define BMP390_CALIB_DATA	uint8_t(0x31)
#define BMP390_PWR_CTRL		uint8_t(0x1B)
#define BMP390_OSR			uint8_t(0x1C)
#define BMP390_ODR			uint8_t(0x1D)
#define BMP390_IIR_CONFIG	uint8_t(0x1F)
#define BMP390_CONCAT_BYTES(msb, lsb)   (((uint16_t)msb << 8) | (uint16_t)lsb)
int bmp390_dev;
float bmp390_elev;
struct {
	float parT1, parT2, parT3;
	float parP1, parP2, parP3, parP4, parP5, parP6, parP7, parP8, parP9, parP10, parP11;
	float tempLin;
	float elev0;
} bmp390_calib;

// read/write register
void bmp390_readReg(uint8_t reg, void* pBuf, size_t size) {
	uint8_t * _pBuf = (uint8_t*)pBuf;
	write(bmp390_dev, &reg, 1);
	read(bmp390_dev, _pBuf, size);
}
void bmp390_writeReg(uint8_t reg, const void* pBuf, size_t size) {
	uint8_t * _pBuf = new uint8_t[size+1];
	_pBuf[0] = reg;
	 memcpy(_pBuf+1, (uint8_t*) pBuf, size);
	 write(bmp390_dev, _pBuf, size+1);
}

// read elevation
void bmp390_read() {
	if (!bmp390_dev) { return; }

	// read uncompressed press & temp
	uint8_t buf[6] = {0};
	uint32_t uncompPress, uncompTemp;
	bmp390_readReg(BMP390_P_DATA_PA, &buf, sizeof(buf));
	uncompPress = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);
	uncompTemp  = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) | ((uint32_t)buf[5] << 16);

	// get calibrated press & temp
	float temp, press;
	float partialData1, partialData2, partialData3, partialData4, partialOut1, partialOut2;
	partialData1 = (float)(uncompTemp - bmp390_calib.parT1);
	partialData2 = (float)(partialData1 * bmp390_calib.parT2);
	temp = partialData2 + pow(partialData1, 2) * bmp390_calib.parT3;
	bmp390_calib.tempLin = temp;
	partialData1 = bmp390_calib.parP6 * bmp390_calib.tempLin;
	partialData2 = bmp390_calib.parP7 * pow(bmp390_calib.tempLin, 2);
	partialData3 = bmp390_calib.parP8 * pow(bmp390_calib.tempLin, 3);
	partialOut1 = bmp390_calib.parP5 + partialData1 + partialData2 + partialData3;
	partialData1 = bmp390_calib.parP2 * bmp390_calib.tempLin;
	partialData2 = bmp390_calib.parP3 * pow(bmp390_calib.tempLin, 2);
	partialData3 = bmp390_calib.parP4 * pow(bmp390_calib.tempLin, 3);
	partialOut2 = uncompPress * (bmp390_calib.parP1 + partialData1 + partialData2 + partialData3);
	partialData1 = (float)uncompPress * (float)uncompPress;
	partialData2 = bmp390_calib.parP9 + bmp390_calib.parP10 * bmp390_calib.tempLin;
	partialData3 = partialData1 * partialData2;
	partialData4 = partialData3 + pow((float)uncompPress, 3) * bmp390_calib.parP11;
	press = partialOut1 + partialOut2 + partialData4;

	// get elevation
	bmp390_elev = (1.0 - pow(press / 101325.0, 0.190284)) * 44307.7 - bmp390_calib.elev0;
}

// init sensor
bool bmp390_init() {

	// connect to device
	if ((bmp390_dev = open(_i2c_dev.c_str(), O_RDWR)) < 0) { 		_error("[driver_imu] failed to open the i2c bus for BMP390"); bmp390_dev = 0; return 0; }
	if (ioctl (bmp390_dev, I2C_SLAVE, 0x77) < 0) { 					_error("[driver_imu] failed to open i2c slave BMP390"); bmp390_dev = 0; return 0; }
	if (i2c_smbus_write_quick(bmp390_dev, I2C_SMBUS_WRITE) < 0) {	_error("[driver_imu] failed to talk to i2c slave BMP390"); bmp390_dev = 0; return 0; }
	ros::Duration(0.05).sleep();

	// configure device
	uint8_t mode_pwr = (1 | 1<<1 | 3<<4);							// ePressEN | eTempEN | eNormalMode
	bmp390_writeReg(BMP390_PWR_CTRL, &mode_pwr, sizeof(mode_pwr));
	ros::Duration(0.05).sleep();
	uint8_t mode_osr = (4 | 1<<3);									// ePressOSRMode16 | eTempOSRMode2
	bmp390_writeReg(BMP390_OSR, &mode_osr, sizeof(mode_osr));
	ros::Duration(0.05).sleep();
	uint8_t mode_odr = 0x03;										// BMP3XX_ODR_25_HZ
	bmp390_writeReg(BMP390_ODR, &mode_odr, sizeof(mode_odr));
	ros::Duration(0.05).sleep();
	uint8_t mode_iir = 0x04;										// BMP3XX_IIR_CONFIG_COEF_3
	bmp390_writeReg(BMP390_IIR_CONFIG, &mode_iir, sizeof(mode_iir));
	ros::Duration(0.05).sleep();

	// get calibration data
	uint8_t regData[21] = {0};
	bmp390_readReg(BMP390_CALIB_DATA, &regData, 21);
	bmp390_calib.parT1 = ((float) ((int16_t)BMP390_CONCAT_BYTES(regData[1], regData[0])) / pow(2, -8));
	bmp390_calib.parT2 = ((float) ((int16_t)BMP390_CONCAT_BYTES(regData[3], regData[2])) / pow(2, 30));
	bmp390_calib.parT3 = ((float) ((int8_t)regData[4]) / pow(2, 48));
	bmp390_calib.parP1 = ((float) ((int16_t)BMP390_CONCAT_BYTES(regData[6], regData[5]) - (16384)) / pow(2, 20));
	bmp390_calib.parP2 = ((float) ((int16_t)BMP390_CONCAT_BYTES(regData[8], regData[7]) - (16384)) / pow(2, 29));
	bmp390_calib.parP3 = ((float) ((int8_t)regData[9]) / pow(2, 32));
	bmp390_calib.parP4 = ((float) ((int8_t)regData[10]) / pow(2, 37));
	bmp390_calib.parP5 = ((float) ((int16_t)BMP390_CONCAT_BYTES(regData[12], regData[11])) / pow(2, -3));
	bmp390_calib.parP6 = ((float) ((int16_t)BMP390_CONCAT_BYTES(regData[14], regData[13])) / pow(2, 6));
	bmp390_calib.parP7 = ((float) ((int8_t)regData[15]) / pow(2, 8));
	bmp390_calib.parP8 = ((float) ((int8_t)regData[16]) / pow(2, 15));
	bmp390_calib.parP9 = ((float) ((int16_t)BMP390_CONCAT_BYTES(regData[18], regData[17])) / pow(2, 48));
	bmp390_calib.parP10= ((float) ((int8_t)regData[19]) / pow(2, 48));
	bmp390_calib.parP11= ((float) ((int8_t)regData[20]) / pow(2, 65));
	ros::Duration(0.05).sleep();
	bmp390_read();
	bmp390_calib.elev0 = bmp390_elev;

	return 1;
}

// shutdown sensor
void bmp390_shutdown() {
	close(bmp390_dev);
}


// ################################################################################################

// xsens mti3 stuff
XsControl* xs_control;
XsPortInfo xs_port;
XsDevice* xs_device;
Journaller* gJournal = 0;

// callback handler for sensor reading
class CallbackHandler : public XsCallback { protected:
	void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override {

		// compute time since last loop
		msg_body_state.header.stamp = ros::Time::now();
		double dt = (msg_body_state.header.stamp - last_body_state.header.stamp).toSec();
		if (dt > 0.1) { dt = 0.0; }

		// compute translation movement
		XsQuaternion q_xs = packet->orientationQuaternion();
		Quaterniond q = Quaterniond(q_xs.w(), q_xs.x(), q_xs.y(), q_xs.z());
		XsVector w_xs = packet->calibratedGyroscopeData();
		Vector3d w = q * Vector3d(w_xs[0],w_xs[1],w_xs[2]);
		Vector3d w_prev = Vector3d(last_body_state.angular_velocity.x, last_body_state.angular_velocity.y, last_body_state.angular_velocity.z);
		Vector3d y = (w - w_prev) / dt;
		if (dt < 1e-6) { y = Vector3d(0.0,0.0,0.0); }

		// compute rotation movement
		XsVector a_xs = packet->calibratedAcceleration();
		Vector3d a = q * Vector3d(a_xs[0],a_xs[1],a_xs[2]) - Vector3d(0,0,9.80665);
		Vector3d v_prev = Vector3d(last_body_state.linear_velocity.x, last_body_state.linear_velocity.y, last_body_state.linear_velocity.z);
		Vector3d v = v_prev + a * dt;
		Vector3d r_prev = Vector3d(last_body_state.translation.x, last_body_state.translation.y, last_body_state.translation.z);
		Vector3d r = r_prev + v * dt;

		// use barometer altitude
		r(2) = bmp390_elev;
		v(2) = (r(2) - last_body_state.translation.z) / dt;
		a(2) = (v(2) - last_body_state.linear_acceleration.z) / dt;
		if (dt < 1e-6)   { v(2) = 0.0; a(2) = 0.0; }
		if (v(2) < 1e-6) { a(2) = 0.0; }

		// copy over results
		msg_body_state.orientation.x = q.x();         msg_body_state.orientation.y = q.y();         msg_body_state.orientation.z = q.z();         msg_body_state.orientation.w = q.w();
		msg_body_state.translation.x = r(0);          msg_body_state.translation.y = r(1);          msg_body_state.translation.z = r(2);
		msg_body_state.linear_velocity.x = v(0);      msg_body_state.linear_velocity.y = v(1);      msg_body_state.linear_velocity.z = v(2);
		msg_body_state.angular_velocity.x = w(0);     msg_body_state.angular_velocity.y = w(1);     msg_body_state.angular_velocity.z = w(2);
		msg_body_state.linear_acceleration.x = a(0);  msg_body_state.linear_acceleration.y = a(1);  msg_body_state.linear_acceleration.z = a(2);
		msg_body_state.angular_acceleration.x = y(0); msg_body_state.angular_acceleration.y = y(1); msg_body_state.angular_acceleration.z = y(2);

		// publish body state
		pub_body_state.publish(msg_body_state);
		// copy body state for next run
		memcpy(&last_body_state, &msg_body_state, sizeof(momav::BodyState));
	}
} xs_callback;

// init device
bool mti3_init() {

	//connect to MTi device
	xs_control = XsControl::construct();
	if (!xs_control) { _error("[driver_imu] failed to create MTi-3 object"); return 0; }
	xs_port = XsPortInfo(_serial_dev.c_str(), XsBaud::numericToRate(921600));
	if (xs_port.empty()) { _error("[driver_imu] failed to find MTi-3 port"); return 0; }
	xs_control->openPort(xs_port.portName().toStdString(), xs_port.baudrate());
	ros::Duration(0.5).sleep();
	xs_control->closePort(xs_port.portName().toStdString());
	ros::Duration(0.5).sleep();
	bool open = xs_control->openPort(xs_port.portName().toStdString(), xs_port.baudrate()); //connecting twice or it wont work..
	if (open != true) { _error("[driver_imu] failed to open MTi-3 port"); return 0; }
	xs_device = xs_control->device(xs_port.deviceId());
	if (!xs_device) { _error("[driver_imu] failed to connect to MTi-3 device"); return 0; }

	// calibrate gyro
	xs_device->gotoMeasurement();
	ros::Duration(0.5).sleep();
	xs_device->setNoRotation(_gyro_cal_time);
	ros::Duration(_gyro_cal_time+0.5).sleep();

	// add callback handler
	xs_device->addCallbackHandler(&xs_callback);

	return 1;
}

// shutdown device
void mti3_shutdown() {
	xs_control->closePort(xs_port.portName().toStdString());
	xs_control->destruct();
}

// ################################################################################################

int main(int argc, char **argv) {
	
	//setup ros node
	ros::init(argc, argv, "driver_imu");
	ros::NodeHandle n;
	ros::Rate rate(25);
	pub_body_state = n.advertise<momav::BodyState>("body_state_imu", 1);
	ros::param::get("/momav/body_state", _body_state_topic);
	ros::param::get("~dev_ser", _serial_dev);
	ros::param::get("~dev_i2c", _i2c_dev);
	ros::param::get("~gyro_cal", _gyro_cal_time);

	//connect to bmp390 & xsens device
	if (!bmp390_init()) { return 0; }
	if (!mti3_init())   { bmp390_shutdown(); return 0; };

	//measure until node is stopped
	while (ros::ok())  {
		bmp390_read();
		// xsens read by callback handler
		ros::spinOnce();
		rate.sleep();
	}

	// shutdown bmp390 & mti3 devices
	bmp390_shutdown();
	mti3_shutdown();
	return 0;
}

void _error(std::string msg) {
	if ( _body_state_topic == "body_state_imu" ) {
		ROS_ERROR(msg.c_str());
	} else {
		ROS_WARN(msg.c_str());
	}
}
