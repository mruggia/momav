#!/usr/bin/env python3

import rospy
from momav.msg import CommanderStream

from threading import Thread, Lock

import numpy as np
import matplotlib.pyplot as plt
plt.switch_backend('QT5Agg')
plt.rcParams.update({
	"font.family": "CMU Serif",
	"font.size": 10
})

#############################################################################################################

#global plot variables
fig=[]; gs=[];
ax_3d=[]; ax_xy=[]; ax_xz=[]; ax_yz=[];
ax_epos=[]; ax_vel=[]; ax_force=[];
ax_eorient=[];ax_angvel=[];ax_torque=[];
ax_throttle=[];ax_angle=[];ax_remote=[];
plt_3d_orig=[]; plt_3d_setp=[]; plt_3d_stat=[];
plt_xy_orig=[]; plt_xy_setp=[]; plt_xy_stat=[];
plt_xz_orig=[]; plt_xz_setp=[]; plt_xz_stat=[];
plt_yz_orig=[]; plt_yz_setp=[]; plt_yz_stat=[];
plt_epos=[]; plt_vel=[]; plt_force=[];
plt_eorient=[];plt_angvel=[];plt_torque=[];
plt_throttle=[];plt_angle=[];plt_remote=[];

#global plot constants
coord_width = 3; coord_height = coord_width/2; coord_ticks = 0.5; coord_buffer = coord_height * 0.075
frame_mrk_sz = 12; frame_vec_sz = 3; frame_vec_len = 0.5
time_width = 5

#global data variables
STREAM_LOCK = Lock()
curr_pos_orig=np.array([0,0,0]);   curr_pos_setp=np.array([0,0,0]);   curr_pos_stat=np.array([0,0,0])
curr_orient_orig=np.array([1,0,0,0]); curr_orient_setp=np.array([1,0,0,0]); curr_orient_stat=np.array([1,0,0,0])
ser_time_abs=np.array([0]); ser_time=np.array([0]);
ser_epos=np.array([[0,0,0]]); ser_vel=np.array([[0,0,0]]); ser_force=np.array([[0,0,0]]); 
ser_eorient=np.array([[0,0,0]]); ser_angvel=np.array([[0,0,0]]); ser_torque=np.array([[0,0,0]]);
ser_throttle=np.array([[0,0,0,0,0,0]]); ser_angle=np.array([[0,0,0,0,0,0]]); ser_remote=np.array([[0,0,0,0,0,0]]);

#############################################################################################################

def run_node():

	init_fig()

	rospy.init_node('monitor')
	rospy.Subscriber("commander_stream", CommanderStream, stream_callback)

	while not rospy.is_shutdown():
		if not fig: init_fig()
		update_fig()
		plt.draw()
		plt.pause(0.01)

def stream_callback(commander_stream):
	global ser_time_abs, ser_time
	global curr_pos_setp, curr_pos_stat
	global curr_orient_setp, curr_orient_stat
	global ser_epos, ser_vel, ser_force
	global ser_eorient, ser_angvel, ser_torque
	global ser_throttle, ser_angle, ser_remote

	curr_time_abs = commander_stream.header.stamp.secs + commander_stream.header.stamp.nsecs/1e9
	curr_pos_setp = np.array([commander_stream.body_setp_wrench.translation.x, commander_stream.body_setp_wrench.translation.y, commander_stream.body_setp_wrench.translation.z])
	curr_pos_stat = np.array([commander_stream.body_state.translation.x, commander_stream.body_state.translation.y, commander_stream.body_state.translation.z])
	curr_epos = curr_pos_stat - curr_pos_setp
	curr_vel = np.array([commander_stream.body_state.linear_velocity.x, commander_stream.body_state.linear_velocity.y, commander_stream.body_state.linear_velocity.z])
	curr_force = np.array([commander_stream.body_setp_wrench.force.x, commander_stream.body_setp_wrench.force.y, commander_stream.body_setp_wrench.force.z])
	
	curr_orient_setp = np.array([commander_stream.body_setp_wrench.orientation.w, commander_stream.body_setp_wrench.orientation.x, commander_stream.body_setp_wrench.orientation.y, commander_stream.body_setp_wrench.orientation.z])
	curr_orient_stat = np.array([commander_stream.body_state.orientation.w, commander_stream.body_state.orientation.x, commander_stream.body_state.orientation.y, commander_stream.body_state.orientation.z])
	curr_q_err = np.array([
		+ curr_orient_setp[0]*curr_orient_stat[0] + curr_orient_setp[1]*curr_orient_stat[1] + curr_orient_setp[2]*curr_orient_stat[2] + curr_orient_setp[3]*curr_orient_stat[3], 
		- curr_orient_setp[1]*curr_orient_stat[0] + curr_orient_setp[0]*curr_orient_stat[1] + curr_orient_setp[3]*curr_orient_stat[2] - curr_orient_setp[2]*curr_orient_stat[3], 
		- curr_orient_setp[2]*curr_orient_stat[0] - curr_orient_setp[3]*curr_orient_stat[1] + curr_orient_setp[0]*curr_orient_stat[2] + curr_orient_setp[1]*curr_orient_stat[3], 
		- curr_orient_setp[3]*curr_orient_stat[0] + curr_orient_setp[2]*curr_orient_stat[1] - curr_orient_setp[1]*curr_orient_stat[2] + curr_orient_setp[0]*curr_orient_stat[3]
	])
	curr_eorient = np.array([
		np.arctan2(+2*curr_q_err[0]*curr_q_err[1]+2*curr_q_err[2]*curr_q_err[3], curr_q_err[0]**2-curr_q_err[1]**2-curr_q_err[2]**2+curr_q_err[3]**2),
		np.arctan2(-2*curr_q_err[1]*curr_q_err[3]+2*curr_q_err[0]*curr_q_err[2], np.sqrt((+2*curr_q_err[0]*curr_q_err[1]+2*curr_q_err[2]*curr_q_err[3])**2+(curr_q_err[0]**2-curr_q_err[1]**2-curr_q_err[2]**2+curr_q_err[3]**2)**2)),
		np.arctan2(+2*curr_q_err[0]*curr_q_err[3]+2*curr_q_err[1]*curr_q_err[2], curr_q_err[0]**2+curr_q_err[1]**2-curr_q_err[2]**2-curr_q_err[3]**2)
	])
	curr_angvel = np.array([commander_stream.body_state.angular_velocity.x, commander_stream.body_state.angular_velocity.y, commander_stream.body_state.angular_velocity.z])
	curr_torque = np.array([commander_stream.body_setp_wrench.torque.x, commander_stream.body_setp_wrench.torque.y, commander_stream.body_setp_wrench.torque.z])

	curr_throttle = np.array(commander_stream.motor_setp.throttle)
	curr_angle = np.array(commander_stream.servo_setp.setpoint)
	curr_angle = np.mod(curr_angle+np.pi, 2*np.pi)-np.pi
	curr_remote = np.array([commander_stream.remote.channels[1],commander_stream.remote.channels[2],commander_stream.remote.channels[0], commander_stream.remote.channels[13],commander_stream.remote.channels[15],commander_stream.remote.channels[12]])

	STREAM_LOCK.acquire()

	ser_time_abs = np.append(ser_time_abs, [curr_time_abs], axis=0)
	ser_time = ser_time_abs - ser_time_abs[-1]
	ser_epos = np.append(ser_epos, [curr_epos*1000], axis=0)
	ser_vel = np.append(ser_vel, [curr_vel*1000], axis=0)
	ser_force = np.append(ser_force, [curr_force], axis=0)
	ser_eorient = np.append(ser_eorient, [curr_eorient*180/np.pi], axis=0)
	ser_angvel = np.append(ser_angvel, [curr_angvel*180/np.pi], axis=0)
	ser_torque = np.append(ser_torque, [curr_torque], axis=0)
	ser_throttle = np.append(ser_throttle, [curr_throttle*100], axis=0)
	ser_angle = np.append(ser_angle, [curr_angle*180/np.pi], axis=0)
	ser_remote = np.append(ser_remote, [curr_remote*100], axis=0)

	trim = ser_time.size - np.sum(ser_time < -time_width-0.1)
	ser_time_abs = ser_time_abs[-trim:]
	ser_time = ser_time[-trim:]
	ser_epos = ser_epos[-trim:]
	ser_vel = ser_vel[-trim:]
	ser_force = ser_force[-trim:]
	ser_eorient = ser_eorient[-trim:]
	ser_angvel = ser_angvel[-trim:]
	ser_torque = ser_torque[-trim:]
	ser_throttle = ser_throttle[-trim:]
	ser_angle = ser_angle[-trim:]
	ser_remote = ser_remote[-trim:]

	STREAM_LOCK.release()


#############################################################################################################

def init_fig():
	global fig, gs
	global ax_3d, ax_xy, ax_xz, ax_yz
	global ax_epos, ax_vel, ax_force
	global ax_eorient, ax_angvel, ax_torque
	global ax_throttle, ax_angle, ax_remote
	global plt_3d_orig, plt_3d_setp, plt_3d_stat
	global plt_xy_orig, plt_xy_setp, plt_xy_stat
	global plt_xz_orig, plt_xz_setp, plt_xz_stat
	global plt_yz_orig, plt_yz_setp, plt_yz_stat
	global plt_epos, plt_vel, plt_force
	global plt_eorient, plt_angvel, plt_torque
	global plt_throttle, plt_angle, plt_remote

	fig = plt.figure(figsize=(12,8))
	gs = fig.add_gridspec(6,3)
	plt.subplots_adjust(top=0.96, bottom=0.06, left=0.04, right=0.98, hspace=0.40, wspace=0.15)

	def onresize(event):
		update_2d_xlim(ax_xy, "2d_xy")
		update_2d_xlim(ax_xz, "2d_xz")
		update_2d_xlim(ax_yz, "2d_yz")
	fig.canvas.mpl_connect('resize_event', onresize)

	def onclose(event):
		global fig; fig = []
	fig.canvas.mpl_connect('close_event', onclose)

	ax_3d       = init_ax("3d", gs[0:3, 0], 'body position XYZ [m]')

	ax_xy       = init_ax("2d_xy", gs[0:3, 1], 'body position XY [m]');
	ax_yz       = init_ax("2d_yz", gs[0:3, 2], '')
	ax_xz       = init_ax("2d_xz", gs[0:3, 2], 'body position XZ/YZ [m]')

	ax_epos     = init_ax("1d", gs[3, 0], 'body position error [mm]')
	ax_vel      = init_ax("1d", gs[4, 0], 'body velocity [mm/s]')
	ax_force    = init_ax("1d", gs[5, 0], 'force setpoint [N]')

	ax_eorient  = init_ax("1d", gs[3, 1], 'body orientation error [deg]')
	ax_angvel   = init_ax("1d", gs[4, 1], 'body rotational velocity [deg/sec]')
	ax_torque   = init_ax("1d", gs[5, 1], 'torque setpoint [Nm]')

	ax_throttle = init_ax("1d", gs[3, 2], 'throttle setpoint [%]');		ax_throttle.set_ylim(-10, 110)
	ax_angle    = init_ax("1d", gs[4, 2], 'arm angle setpoint [deg]');	ax_angle.set_ylim(-180, 180)
	ax_remote   = init_ax("1d", gs[5, 2], 'remote channels [%]');		ax_remote.set_ylim(-10, 110)

	plt_3d_orig = init_plt(ax_3d, "3d_frame_orig")
	plt_3d_setp = init_plt(ax_3d, "3d_frame_setp")
	plt_3d_stat = init_plt(ax_3d, "3d_frame_stat")

	plt_xy_orig = init_plt(ax_xy, "2d_frame_orig")
	plt_xy_setp = init_plt(ax_xy, "2d_frame_setp")
	plt_xy_stat = init_plt(ax_xy, "2d_frame_stat")

	plt_xz_orig = init_plt(ax_xz, "2d_frame_orig")
	plt_xz_setp = init_plt(ax_xz, "2d_frame_setp")
	plt_xz_stat = init_plt(ax_xz, "2d_frame_stat")

	plt_yz_orig = init_plt(ax_yz, "2d_frame_orig")
	plt_yz_setp = init_plt(ax_yz, "2d_frame_setp")
	plt_yz_stat = init_plt(ax_yz, "2d_frame_stat")

	plt_epos    = init_plt(ax_epos, "1d_3")
	plt_vel     = init_plt(ax_vel, "1d_3")
	plt_force   = init_plt(ax_force, "1d_3")

	plt_eorient = init_plt(ax_eorient, "1d_3")
	plt_angvel  = init_plt(ax_angvel, "1d_3")
	plt_torque  = init_plt(ax_torque, "1d_3")

	plt_throttle= init_plt(ax_throttle, "1d_6")
	plt_angle   = init_plt(ax_angle, "1d_6")
	plt_remote  = init_plt(ax_remote, "1d_6")

#############################################################################################################

def update_fig():

	STREAM_LOCK.acquire()

	update_plt_frame(plt_3d_orig, "3d",    curr_pos_orig, curr_orient_orig)
	update_plt_frame(plt_xy_orig, "2d_xy", curr_pos_orig, curr_orient_orig)
	update_plt_frame(plt_xz_orig, "2d_xz", curr_pos_orig, curr_orient_orig)
	update_plt_frame(plt_yz_orig, "2d_yz", curr_pos_orig, curr_orient_orig)

	update_plt_frame(plt_3d_setp, "3d",    curr_pos_setp, curr_orient_setp)
	update_plt_frame(plt_xy_setp, "2d_xy", curr_pos_setp, curr_orient_setp)
	update_plt_frame(plt_xz_setp, "2d_xz", curr_pos_setp, curr_orient_setp)
	update_plt_frame(plt_yz_setp, "2d_yz", curr_pos_setp, curr_orient_setp)

	update_plt_frame(plt_3d_stat, "3d",    curr_pos_stat, curr_orient_stat)
	update_plt_frame(plt_xy_stat, "2d_xy", curr_pos_stat, curr_orient_stat)
	update_plt_frame(plt_xz_stat, "2d_xz", curr_pos_stat, curr_orient_stat)
	update_plt_frame(plt_yz_stat, "2d_yz", curr_pos_stat, curr_orient_stat)

	update_plt_series(ax_epos, plt_epos, "1d_3", ser_time, ser_epos)
	update_plt_series(ax_vel, plt_vel, "1d_3", ser_time, ser_vel)
	update_plt_series(ax_force, plt_force, "1d_3", ser_time, ser_force)

	update_plt_series(ax_eorient, plt_eorient, "1d_3", ser_time, ser_eorient)
	update_plt_series(ax_angvel, plt_angvel, "1d_3", ser_time, ser_angvel)
	update_plt_series(ax_torque, plt_torque, "1d_3", ser_time, ser_torque)

	update_plt_series(ax_throttle, plt_throttle, "1d_6", ser_time, ser_throttle)
	update_plt_series(ax_angle, plt_angle, "1d_6", ser_time, ser_angle)
	update_plt_series(ax_remote, plt_remote, "1d_6", ser_time, ser_remote)

	STREAM_LOCK.release()


#############################################################################################################
#############################################################################################################

def init_ax(type, loc, name):
	global fig

	if type == "3d":

		ref = fig.add_subplot(loc, projection='3d', computed_zorder=False)
		ref.title.set_text(name)
		ref.view_init(elev=30., azim=210)
		ref.set_box_aspect((coord_width,coord_width,coord_height))
		ref.axes.set_xlim3d(left=-coord_width/2-coord_buffer, right=coord_width/2+coord_buffer)
		ref.axes.set_ylim3d(bottom=-coord_width/2-coord_buffer, top=coord_width/2+coord_buffer)
		ref.axes.set_zlim3d(bottom=0, top=coord_height+coord_buffer)
		ref.xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
		ref.yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
		ref.zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
		ref.set_xticks(np.arange(-coord_width/2, coord_width/2+coord_ticks, coord_ticks))
		ref.set_yticks(np.arange(-coord_width/2, coord_width/2+coord_ticks, coord_ticks))
		ref.set_zticks(np.arange(0, coord_height+coord_ticks, coord_ticks))

	elif type == "2d_xy" or type=="2d_xz" or type=="2d_yz" :

		ref = fig.add_subplot(loc)
		ref.title.set_text(name)
		ref.grid()

		if type == "2d_xy":

			ref_pos = ref.get_position()
			ref_pos = [ref_pos.x0, ref_pos.y0+0.03,  ref_pos.width, ref_pos.height-0.03]
			ref.set_position(ref_pos)
			ref.set_ylim(-coord_width/2-coord_buffer*2, coord_width/2+coord_buffer*2)
			ref.set_yticks(np.arange(-coord_width/2, coord_width/2+coord_ticks, coord_ticks))
			update_2d_xlim(ref, type)

		elif type == "2d_xz":

			ref_pos = ref.get_position()
			ref_pos = [ref_pos.x0, ref_pos.y0+(ref_pos.height-0.03)/2+0.03,  ref_pos.width, (ref_pos.height-0.03)/2]
			ref.set_position(ref_pos)
			ref.set_ylim(0-coord_buffer, coord_height+coord_buffer)
			ref.set_yticks(np.arange(0, coord_height+coord_ticks, coord_ticks))
			update_2d_xlim(ref, type)
			ref.set_xticklabels([])

		elif type == "2d_yz":

			ref_pos = ref.get_position()
			ref_pos = [ref_pos.x0, ref_pos.y0+0.03,  ref_pos.width, (ref_pos.height-0.03)/2]
			ref.set_position(ref_pos)
			ref.set_ylim(0-coord_buffer, coord_height+coord_buffer)
			ref.set_yticks(np.arange(0, coord_height+coord_ticks, coord_ticks))
			update_2d_xlim(ref, type)
	
	elif type == "1d":

		ref = fig.add_subplot(loc)
		ref.grid()
		ref.set_xlim(-time_width, 0)
		ref.set_xticks(np.arange(-time_width, 0+coord_ticks, coord_ticks))
		ref.title.set_text(name)

		gs = loc.get_geometry()
		if (gs[0]-1)*gs[1] > gs[2]:
			plt.setp(ref.get_xticklabels(), visible=False)
		else:
			ref.set_xlabel('time [sec]')

	return ref

def update_2d_xlim(ax, type):
	global fig

	ratio = ax.get_position().width/ax.get_position().height * fig.get_size_inches()[0]/fig.get_size_inches()[1]
	height = ax.get_ylim()[1]-ax.get_ylim()[0]
	width = height*ratio
	tick = ax.get_yticks()[1]-ax.get_yticks()[0]
	buffer = ax.get_yticks()[0]-ax.get_ylim()[0]
	left = -width/2
	right = width/2
	tick_left = np.ceil(left/tick)*tick
	tick_right = np.floor(right/tick)*tick

	ax.set_xticks(np.arange(tick_left, tick_right+tick, tick))
	if type == "2d_xy":
		ax.set_xlim(right, left)
	else:
		ax.set_xlim(left, right)

#############################################################################################################

def init_plt(ax, type):

	if type == "3d_frame_orig" or type == "3d_frame_setp" or type == "3d_frame_stat":

		if type   == "3d_frame_orig": col_m = '#C0C0C0'; col_x = '#51A9E6'; col_y = '#FFB140'; col_z = '#5ED25E'
		elif type == "3d_frame_setp": col_m = '#404040'; col_x = '#51A9E6'; col_y = '#FFB140'; col_z = '#5ED25E'
		elif type == "3d_frame_stat": col_m = '#FFFFFF'; col_x = '#1F77B4'; col_y = '#FF7F0E'; col_z = '#2CA02C'

		ret = [[],[],[],[]]
		if type == "3d_frame_stat":
			ret[0] = ax.plot([0], [0], [0], marker='.', color=col_m, markeredgecolor='black', linestyle="", markersize=frame_mrk_sz, zorder=3)
		else:
			ret[0] = ax.plot([0], [0], [0], marker='.', color=col_m, linestyle="", markersize=frame_mrk_sz, zorder=3)
		ret[1] = ax.plot([0, 1], [0, 0], zs=[0, 0], color=col_x, linewidth=frame_vec_sz)
		ret[2] = ax.plot([0, 0], [0, 1], zs=[0, 0], color=col_y, linewidth=frame_vec_sz)
		ret[3] = ax.plot([0, 0], [0, 0], zs=[0, 1], color=col_z, linewidth=frame_vec_sz)

	elif type == "2d_frame_orig" or type == "2d_frame_setp" or type == "2d_frame_stat":

		if type   == "2d_frame_orig": col_m = '#C0C0C0'; col_x = '#51A9E6'; col_y = '#FFB140'; col_z = '#5ED25E'
		elif type == "2d_frame_setp": col_m = '#404040'; col_x = '#51A9E6'; col_y = '#FFB140'; col_z = '#5ED25E'
		elif type == "2d_frame_stat": col_m = '#FFFFFF'; col_x = '#1F77B4'; col_y = '#FF7F0E'; col_z = '#2CA02C'

		ret = [[],[],[],[]]
		if type == "2d_frame_stat":
			ret[0] = ax.plot([0], [0], marker='.', color=col_m, markeredgecolor='black', linestyle="", markersize=frame_mrk_sz, zorder=3)
		else:
			ret[0] = ax.plot([0], [0], marker='.', color=col_m, linestyle="", markersize=frame_mrk_sz, zorder=3)
		ret[1] = ax.plot([0, 1], [0, 0], color=col_x, linewidth=frame_vec_sz)
		ret[2] = ax.plot([0, 0], [0, 1], color=col_y, linewidth=frame_vec_sz)
		ret[3] = ax.plot([0, 0], [0, 0], color=col_z, linewidth=frame_vec_sz)

	elif type == "1d_3":

		ret = [[],[],[]]
		for i in range(3):
			ret[i] = ax.plot([],[])

	elif type == "1d_6":

		col = ['#1F77B4','#FF7F0E','#2CA02C', '#51A9E6','#FFB140','#5ED25E']

		ret = [[],[],[],[],[],[]]
		for i in range(6):
			ret[i] = ax.plot([],[], color=col[i])


	return ret

#############################################################################################################

def update_plt_frame(plt, type, p, q):

	C = [[q[0]**2+q[1]**2-q[2]**2-q[3]**2, 2*q[1]*q[2]-2*q[0]*q[3], 2*q[0]*q[2]+2*q[1]*q[3]],
		 [2*q[0]*q[3]+2*q[1]*q[2], q[0]**2-q[1]**2+q[2]**2-q[3]**2, 2*q[2]*q[3]-2*q[0]*q[1]],
		 [2*q[1]*q[3]-2*q[0]*q[2], 2*q[0]*q[1]+2*q[2]*q[3], q[0]**2-q[1]**2-q[2]**2+q[3]**2]]

	if type == "3d":
		plt[0][0].set_data([p[0]], [p[1]]); plt[0][0].set_3d_properties([p[2]])
		plt[1][0].set_data([p[0], p[0]+C[0][0]*frame_vec_len], [p[1], p[1]+C[1][0]*frame_vec_len]); plt[1][0].set_3d_properties([p[2], p[2]+C[2][0]*frame_vec_len])
		plt[2][0].set_data([p[0], p[0]+C[0][1]*frame_vec_len], [p[1], p[1]+C[1][1]*frame_vec_len]); plt[2][0].set_3d_properties([p[2], p[2]+C[2][1]*frame_vec_len])
		plt[3][0].set_data([p[0], p[0]+C[0][2]*frame_vec_len], [p[1], p[1]+C[1][2]*frame_vec_len]); plt[3][0].set_3d_properties([p[2], p[2]+C[2][2]*frame_vec_len])

	elif type == "2d_xy":
		plt[0][0].set_data([p[1]], [p[0]])
		plt[1][0].set_data([p[1], p[1]+C[1][0]*frame_vec_len], [p[0], p[0]+C[0][0]*frame_vec_len])
		plt[2][0].set_data([p[1], p[1]+C[1][1]*frame_vec_len], [p[0], p[0]+C[0][1]*frame_vec_len])
		plt[3][0].set_data([p[1], p[1]+C[1][2]*frame_vec_len], [p[0], p[0]+C[0][2]*frame_vec_len])

	elif type == "2d_xz":
		plt[0][0].set_data([p[0]], [p[2]])
		plt[1][0].set_data([p[0], p[0]+C[0][0]*frame_vec_len], [p[2], p[2]+C[2][0]*frame_vec_len])
		plt[2][0].set_data([p[0], p[0]+C[0][1]*frame_vec_len], [p[2], p[2]+C[2][1]*frame_vec_len])
		plt[3][0].set_data([p[0], p[0]+C[0][2]*frame_vec_len], [p[2], p[2]+C[2][2]*frame_vec_len])
		
	elif type == "2d_yz":
		plt[0][0].set_data([p[1]], [p[2]])
		plt[1][0].set_data([p[1], p[1]+C[1][0]*frame_vec_len], [p[2], p[2]+C[2][0]*frame_vec_len])
		plt[2][0].set_data([p[1], p[1]+C[1][1]*frame_vec_len], [p[2], p[2]+C[2][1]*frame_vec_len])
		plt[3][0].set_data([p[1], p[1]+C[1][2]*frame_vec_len], [p[2], p[2]+C[2][2]*frame_vec_len])

def update_plt_series(ax, plt, type, time, data):
	
	if type == "1d_3":
		plt[0][0].set_data(time, data[:,0])
		plt[1][0].set_data(time, data[:,1])
		plt[2][0].set_data(time, data[:,2])

		ylim = np.max([0.1, np.max(np.absolute(data))*1.2 ]); 
		ax.set_ylim(-ylim, ylim)

	elif type == "1d_6":
		plt[0][0].set_data(time, data[:,0])
		plt[1][0].set_data(time, data[:,1])
		plt[2][0].set_data(time, data[:,2])
		plt[3][0].set_data(time, data[:,3])
		plt[4][0].set_data(time, data[:,4])
		plt[5][0].set_data(time, data[:,5])

############################################################################################################

if __name__ == '__main__':
	run_node()