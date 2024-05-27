
#include "ros/ros.h"
#include "momav/BodyState.h"

#include <chrono>
#include <Eigen/Dense>
using namespace Eigen;

#include <NatNetCAPI.h>
#include <NatNetClient.h>

// ################################################################################################

// ros topics
ros::Publisher pub_body_state;
momav::BodyState msg_body_state;

// ros params
int _body_id;
int _flt_win, _flt_pow;
uint64_t _host_clock;

// prototypes
void FrameCallback(sFrameOfMocapData *data, void* pUserData);
void MessageHandler( Verbosity msgType, const char* msg );

// data filtering
struct flt_frame{
	Vector3d r, v, a;
	Quaterniond q; Vector3d w, y;
	uint64_t timestamp;
};
flt_frame* flt_frames;
int flt_head = 0;

// ################################################################################################

int main( int argc, char **argv) {

	//setup ros node
	ros::init(argc, argv,"driver_track");
	ros::NodeHandle n;
	pub_body_state = n.advertise<momav::BodyState>("body_state_track", 1);
	ros::param::get("~body_id", _body_id);
	ros::param::get("~flt_win", _flt_win); ros::param::get("~flt_pow", _flt_pow);

	// Print NatNet client version info
	/*unsigned char ver[4];
	NatNet_GetVersion( ver );
	printf( "NatNet Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3] );*/

	// Create NatNet client
	NatNetClient* g_pClient = new NatNetClient();

	// Discover server
	const unsigned int kDiscoveryWaitTimeMillisec = 5 * 1000; // Wait 5 seconds for responses.
	const int kMaxDescriptions = 10; // Get info for, at most, the first 10 servers to respond.
	sNatNetDiscoveredServer servers[kMaxDescriptions];
	int actualNumDescriptions = kMaxDescriptions;
	NatNet_BroadcastServerDiscovery( servers, &actualNumDescriptions );
	if (actualNumDescriptions < 1) { ROS_ERROR("[driver_track] could not find optitrack server!"); return 0; }
	if (actualNumDescriptions > 1) { ROS_WARN ("[driver_track] more than one optitrack server found!"); }

	// Create server description
	char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;
	snprintf(
		g_discoveredMulticastGroupAddr, sizeof g_discoveredMulticastGroupAddr, "%d.%d.%d.%d",
		servers[0].serverDescription.ConnectionMulticastAddress[0],
		servers[0].serverDescription.ConnectionMulticastAddress[1],
		servers[0].serverDescription.ConnectionMulticastAddress[2],
		servers[0].serverDescription.ConnectionMulticastAddress[3]
	);
	sNatNetClientConnectParams g_connectParams;
	g_connectParams.connectionType    = servers[0].serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
	g_connectParams.serverCommandPort = servers[0].serverCommandPort;
	g_connectParams.serverDataPort    = servers[0].serverDescription.ConnectionDataPort;
	g_connectParams.serverAddress     = servers[0].serverAddress;
	g_connectParams.localAddress      = servers[0].localAddress;
	g_connectParams.multicastAddress  = g_discoveredMulticastGroupAddr;

	// Init Client and connect to NatNet server
	int retCode = g_pClient->Connect( g_connectParams );
	if (retCode != ErrorCode_OK) { ROS_ERROR("[driver_track] unable to connect to server. error code: %d", retCode); return 0; } 

	// request return values
	void* pResult;
	int nBytes = 0;
	ErrorCode ret = ErrorCode_OK;

	// print server info
	sServerDescription g_serverDescription;
	memset( &g_serverDescription, 0, sizeof( g_serverDescription ) );
	ret = g_pClient->GetServerDescription( &g_serverDescription );
	if ( ret != ErrorCode_OK || !g_serverDescription.HostPresent ) { ROS_ERROR("[driver_track] unable to connect to server. host not present."); return 0; }
	_host_clock = g_serverDescription.HighResClockFrequency;

	// print server description
	/*printf("\n[SampleClient] Server application info:\n");
	printf("Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp, g_serverDescription.HostAppVersion[0], g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2], g_serverDescription.HostAppVersion[3]);
	printf("NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0], g_serverDescription.NatNetVersion[1], g_serverDescription.NatNetVersion[2], g_serverDescription.NatNetVersion[3]);
	printf("Client IP:%s\n", g_connectParams.localAddress);
	printf("Server IP:%s\n", g_connectParams.serverAddress);
	printf("Server Name:%s\n", g_serverDescription.szHostComputerName);*/

	// get mocap frame rate
	/* = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
	if (ret == ErrorCode_OK) {
		double fRate = *((double*)pResult);
		ROS_INFO("Mocap Framerate : %3.2f", fRate);
	} else { ROS_ERROR("Error getting frame rate."); }*/

	// Install natnet logging callback for some internal details
	//NatNet_SetLogCallback(MessageHandler);

	// Set the frame callback handler
	g_pClient->SetFrameReceivedCallback(FrameCallback, g_pClient);

	// initialize filter
	flt_frames = new flt_frame[_flt_win];

	// loop until ros is closed
	ros::spin();

	g_pClient->Disconnect();
	delete g_pClient; g_pClient = NULL;
	delete flt_frames;
	return 0;
}

// ################################################################################################

// frame data callback
void FrameCallback(sFrameOfMocapData *data, void* pUserData) {

	/*static auto t_old = std::chrono::high_resolution_clock::now();
	auto t_new = std::chrono::high_resolution_clock::now();
	double dt = ((double) std::chrono::duration_cast<std::chrono::microseconds>(t_new-t_old).count()) / 1.0e3;
	t_old = t_new;
	NatNetClient* pClient = (NatNetClient*) pUserData;
	double latency1 = pClient->SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp) * 1000.0;
	double latency2 = pClient->SecondsSinceHostTimestamp(data->CameraDataReceivedTimestamp) * 1000.0;
	double latency3 = pClient->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;
	ROS_WARN("expose:%f, solve:%f, transmit:%f, last:%f", latency1-latency2, latency2-latency3, latency3, dt);*/

	// current body state variables
	Vector3d r, v, a;
	Quaterniond q; Vector3d w, y;

	// read data from mocap frame
	bool success = false;
	for(int i=0; i < data->nRigidBodies; i++) {
		if (_body_id == data->RigidBodies[i].ID) {
			r = Vector3d(data->RigidBodies[i].x, data->RigidBodies[i].y, data->RigidBodies[i].z);
			q = Quaterniond(data->RigidBodies[i].qw, data->RigidBodies[i].qx, data->RigidBodies[i].qy, data->RigidBodies[i].qz);
			success = true;
		}
	}
	if(!success) { return; }

	// get timestamp
	static uint64_t timestamp_prev = data->CameraMidExposureTimestamp;
	uint64_t timestamp_now = data->CameraMidExposureTimestamp;
	bool first_run = (timestamp_prev == timestamp_now);
	double rate = ((double) _host_clock) / ((double) (timestamp_now - timestamp_prev));
	if (first_run) { rate = 0.0; }
	timestamp_prev = timestamp_now;

	// ofset position by position at boot
	static Vector3d r_init;
	if (first_run) { r_init = r; }
	r = r - r_init;

	// add data to filter buffer
	flt_frames[flt_head] = {r,v,a,q,w,y, timestamp_now};
	if (first_run) { for(int i; i<_flt_win; i++) { flt_frames[i] = {r,v,a,q,w,y, timestamp_now-i}; }}
	flt_head++; if (flt_head>=_flt_win) { flt_head = 0; }

	// solve least square matrix for filter
	MatrixXd A(_flt_win, _flt_pow+1), B(_flt_win, 7), X(_flt_pow+1, 7);
	for (int i=0; i<_flt_win; i++) {
		double dt = -((double)(timestamp_now - flt_frames[i].timestamp)) / ((double) _host_clock);
		for (int j=0; j<_flt_pow+1; j++)  {
			A(i,j) = pow(dt,j);
		}
		int j=0;
		B(i,j++) = flt_frames[i].r.x(); B(i,j++) = flt_frames[i].r.y(); B(i,j++) = flt_frames[i].r.z();
		B(i,j++) = flt_frames[i].q.x(); B(i,j++) = flt_frames[i].q.y(); B(i,j++) = flt_frames[i].q.z(); B(i,j++) = flt_frames[i].q.w();
	}
	auto decomp = A.householderQr();
	X = decomp.solve(B);

	//extract filtered position
	//r = Vector3d(X(0,0),X(0,1),X(0,2));
	v = Vector3d(X(1,0),X(1,1),X(1,2));
	a = Vector3d(X(2,0),X(2,1),X(2,2)); a = 2.0*a;

	//extract filtered rotation
	//Matrix<double,4,1> qv;   qv   << X(0,6),X(0,3),X(0,4),X(0,5); qv.normalize();
	Matrix<double,4,1> qv;   qv   << q.w(),q.x(),q.y(),q.z();
	Matrix<double,4,1> dqv;  dqv  << X(1,6),X(1,3),X(1,4),X(1,5);
	Matrix<double,4,1> ddqv; ddqv << X(2,6),X(2,3),X(2,4),X(2,5); ddqv = 2.0*ddqv;
	Matrix<double,3,4> Eq;   Eq   << -qv(1), +qv(0), -qv(3), +qv(2), -qv(2), +qv(3), +qv(0), -qv(1), -qv(3), -qv(2), +qv(1), +qv(0); Eq = 2.0*Eq;
	//q = Quaterniond(qv(0),qv(1),qv(2),qv(3));
	w = Eq * dqv;
	y = Eq * ddqv;

	// copy over results
	msg_body_state.translation.x          = r.x(); msg_body_state.translation.y          = r.y(); msg_body_state.translation.z          = r.z();
	msg_body_state.linear_velocity.x      = v.x(); msg_body_state.linear_velocity.y      = v.y(); msg_body_state.linear_velocity.z      = v.z();
	msg_body_state.linear_acceleration.x  = a.x(); msg_body_state.linear_acceleration.y  = a.y(); msg_body_state.linear_acceleration.z  = a.z();
	msg_body_state.orientation.x          = q.x(); msg_body_state.orientation.y          = q.y(); msg_body_state.orientation.z          = q.z(); msg_body_state.orientation.w = q.w();
	msg_body_state.angular_velocity.x     = w.x(); msg_body_state.angular_velocity.y     = w.y(); msg_body_state.angular_velocity.z     = w.z();
	msg_body_state.angular_acceleration.x = y.x(); msg_body_state.angular_acceleration.y = y.y(); msg_body_state.angular_acceleration.z = y.z();

	// publish result
	msg_body_state.header.stamp = ros::Time::now();
	pub_body_state.publish(msg_body_state);

}

// natnet logging callback for some internal details
void MessageHandler( Verbosity msgType, const char* msg ) {
	if ( msgType < Verbosity_Info ) { return; }
	char bfr [256];
	strcpy(bfr, "[NatNetLib]");
	switch ( msgType ) {
		case Verbosity_Debug:   strcat(bfr, " [DEBUG]: "); break;
		case Verbosity_Info:    strcat(bfr, "  [INFO]: "); break;
		case Verbosity_Warning: strcat(bfr, "  [WARN]: "); break;
		case Verbosity_Error:   strcat(bfr, " [ERROR]: "); break;
		default:                strcat(bfr, " [?????]: "); break;
	}
	strcat(bfr, msg);
	ROS_WARN("%s", bfr);
}