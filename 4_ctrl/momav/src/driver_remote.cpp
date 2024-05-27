
#include "ros/ros.h"
#include "momav/Remote.h"

#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>


// ################################################################################################

//remote
int device_handle = 0;

//SBUS
const int BF_SIZE = 128;
const uint8_t _sbusHeader = 0x0F, _sbusFooter = 0x00;
const uint8_t _payloadSize = 24;
uint8_t _parserState, _prevByte, _curByte, _buffer[BF_SIZE];
uint8_t _payload[_payloadSize];
double channels[16];
double channels_curr[16];
std::string _device;
double _flt_lim, _flt_slow, _flt_fast;
bool parseSBUS();


// ################################################################################################

// program entry point
int main(int argc, char **argv) {

	//setup ros node
	ros::init(argc, argv, "driver_remote");
	ros::NodeHandle n;
	ros::param::get("~device", _device);
	ros::param::get("~flt_lim", _flt_lim);
	ros::param::get("~flt_slow", _flt_slow);
	ros::param::get("~flt_fast", _flt_fast);
	ros::Publisher pub_remote = n.advertise<momav::Remote>("remote", 1);
	momav::Remote msg_remote;
	ros::Rate rate(1000);

	//connect to remote
	device_handle = open( _device.c_str(), O_RDWR|O_NONBLOCK|O_NDELAY );
	if ( device_handle < 0 ) { ROS_ERROR("[remote] failed to connect to remote"); return 0; }
	struct termios tty; memset (&tty, 0, sizeof tty);
	tcgetattr ( device_handle, &tty );
	tty.c_cflag	&= ~PARENB;			// enable parity
	tty.c_cflag	&= ~CSTOPB;			// 1 stop bit
	tty.c_cflag	&= ~CSIZE;			// clear size
	tty.c_cflag	|= CS8;				// set 8 bits per byte
	tty.c_cflag	&= ~CRTSCTS;		// no flow control
	tty.c_cflag |= CREAD|CLOCAL;	// Turn on READ & ignore ctrl lines (CLOCAL = 1)
	tty.c_iflag &=  ~(IXON|IXOFF|IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
	tty.c_lflag		= 0;			// no signaling chars, no echo, no canonical processing
	tty.c_oflag		= 0;			// no remapping, no delays
	tty.c_cc[VMIN]	= 0;			// read doesn't block
	tty.c_cc[VTIME]	= 2;			// 0.2 seconds read timeout
	cfsetispeed(&tty, B115200);		//input baud rate
	cfsetospeed(&tty, B115200);		//output baud rate
	if (tcsetattr(device_handle, TCSANOW, &tty) != 0)  { ROS_ERROR("[remote] failed to configure remote connection"); close(device_handle); return 0; }

	//loop to get at least one clean sbus packet to initialize channels
	while (ros::ok())  {
		if(parseSBUS()) { break; }
		ros::spinOnce(); rate.sleep(); ros::spinOnce();
	}
	while (ros::ok())  {
		if(parseSBUS()) { break; }
		ros::spinOnce(); rate.sleep(); ros::spinOnce();
	}
	for(int i=0; i<16; i++) { channels[i] = channels_curr[i]; }

	//loop until user exits
	while (ros::ok())  {

		bool new_packet = parseSBUS();
		if (new_packet) {

			// get filtered remote input
			for(int i=0; i<16; i++) { 
				if(std::abs(channels_curr[i]-channels[i]) > _flt_lim) {
					channels[i] = (_flt_fast)*(channels_curr[i]) + (1.0-_flt_fast)*channels[i];
				} else {
					channels[i] = (_flt_slow)*(channels_curr[i]) + (1.0-_flt_slow)*channels[i];
				}
				msg_remote.channels[i] = channels[i];
			}

			msg_remote.header.stamp = ros::Time::now();
			pub_remote.publish(msg_remote);
		}

		ros::spinOnce(); rate.sleep(); ros::spinOnce();
	}

	close(device_handle);
	return 0;
}


// ################################################################################################

bool parseSBUS() {

	int n = read(device_handle, &_buffer, BF_SIZE);
	if (n == 0) { return false; }
	bool new_packet = false;
	for (int i=0; i<n; i++) {
		_curByte = _buffer[i];

		if (_parserState == 0) {
			if (_curByte == _sbusHeader && _prevByte == _sbusFooter) {
				_parserState++;
			}
		}
		else {
			if (_parserState - 1 < 24) {
				_payload[_parserState - 1] = _curByte;
				_parserState++;
			}
			if (_parserState - 1 == _payloadSize) {
				if (_curByte == _sbusFooter) {
					
					// decode raw remote input
					uint16_t channels_raw[16];
					channels_raw[0]  = (uint16_t)((_payload[0] >> 0  | _payload[1] << 8) & 0x07FF);
					channels_raw[1]  = (uint16_t)((_payload[1] >> 3  | _payload[2] << 5) & 0x07FF);
					channels_raw[2]  = (uint16_t)((_payload[2] >> 6  | _payload[3] << 2 | _payload[4] << 10) & 0x07FF);
					channels_raw[3]  = (uint16_t)((_payload[4] >> 1  | _payload[5] << 7) & 0x07FF);
					channels_raw[4]  = (uint16_t)((_payload[5] >> 4  | _payload[6] << 4) & 0x07FF);
					channels_raw[5]  = (uint16_t)((_payload[6] >> 7  | _payload[7] << 1 | _payload[8] << 9) & 0x07FF);
					channels_raw[6]  = (uint16_t)((_payload[8] >> 2  | _payload[9] << 6) & 0x07FF);
					channels_raw[7]  = (uint16_t)((_payload[9] >> 5  | _payload[10] << 3) & 0x07FF);
					channels_raw[8]  = (uint16_t)((_payload[11] >> 0 | _payload[12] << 8) & 0x07FF);
					channels_raw[9]  = (uint16_t)((_payload[12] >> 3 | _payload[13] << 5) & 0x07FF);
					channels_raw[10] = (uint16_t)((_payload[13] >> 6 | _payload[14] << 2 | _payload[15] << 10) & 0x07FF);
					channels_raw[11] = (uint16_t)((_payload[15] >> 1 | _payload[16] << 7) & 0x07FF);
					channels_raw[12] = (uint16_t)((_payload[16] >> 4 | _payload[17] << 4) & 0x07FF);
					channels_raw[13] = (uint16_t)((_payload[17] >> 7 | _payload[18] << 1 | _payload[19] << 9) & 0x07FF);
					channels_raw[14] = (uint16_t)((_payload[19] >> 2 | _payload[20] << 6) & 0x07FF);
					channels_raw[15] = (uint16_t)((_payload[20] >> 5 | _payload[21] << 3) & 0x07FF);

					// get float remote input
					for(int i=0; i<16; i++) { channels_curr[i] = ((double)(channels_raw[i])-175)/(2000.0-2.0*175.0); }

					new_packet = true;
				}
				_parserState = 0;
			}
		}
		_prevByte = _curByte;
	}

	return new_packet;
}

