
import os, sys
import rosbag
import math
import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import filedialog
###############################################################################

def quaternion_to_euler_angle(w, x, y, z):
    ysqr = y * y
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))
    return X, Y, Z

def fix_angle_overflow(data):
    prev = float('nan')
    for idx in range(0,len(data)):
        if np.isnan(data[idx]): data[idx] = prev
        if data[idx] - prev > +180.0: data[idx] -= 360.0
        if data[idx] - prev < -180.0: data[idx] += 360.0
        prev = data[idx]
        
def fix_time(data, t_min, t_max):
    t_len = math.floor(t_max-t_min)
    data = np.array(data)
    data[0] = data[0] - t_min
    mask = (data[0]>-0.01)&(data[0]<t_len+0.01)
    data = data.compress(mask, axis=1)
    data = list(data)
    return data

###############################################################################

#get path to bag file
root = tk.Tk()
root.overrideredirect(True)
root.geometry('0x0+0+0')
root.focus_force()
file_path = filedialog.askopenfilename(parent=root, initialdir=os.getcwd(), filetypes=[("rosbag", "*.bag")])
root.destroy()
if file_path == '': sys.exit()

#configure figure
plt.switch_backend('QT5Agg')
plt.rc('font', size=10)
fig, axs = plt.subplots(3, 2, sharex=True)

plt.subplots_adjust(top=0.973, bottom=0.078, left=0.035, right=0.994, hspace=0.151, wspace=0.075)
figManager = plt.get_current_fig_manager()
figManager.window.showMaximized()

###############################################################################


topics = {
    'body_state_imu': '/momav/body_state_imu',
    'body_setp_wrench': '/momav/body_setp_wrench',
    'remote': '/momav/remote',
    'motor_setp': '/momav/motor_setp',
    'servo_setp': '/momav/servo_setp'
}
topics_flip = dict((v,k) for k,v in topics.items())

body_state_imu = [[],[],[],[],[],[],[],[],[],[]] #t, r,p,y, dr,dp,dy, ddx,ddy,ddz
body_setp_wrench = [[],[],[],[],[],[],[]] #t, fx,fy,fz, tx,ty,tz
remote = [[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]] #t, channels 1-16
motor_setp = [[],[],[],[],[],[],[]] #t, throttles 1-6
servo_setp = [[],[],[],[],[],[],[]] #t, angle setpoints 1-6
#motor_state = ...
#servo_state = ...

#extract data
bag = rosbag.Bag(file_path)
for topic, msg, t in bag.read_messages(topics=list(topics.values())):
    topic = topics_flip[topic]
    
    if topic == 'body_state_imu':
        body_state_imu[0].append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
        quat = quaternion_to_euler_angle(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        body_state_imu[1].append(quat[0]); body_state_imu[2].append(quat[1]); body_state_imu[3].append(quat[2])
        body_state_imu[4].append(msg.angular_velocity.x); body_state_imu[5].append(msg.angular_velocity.y); body_state_imu[6].append(msg.angular_velocity.z)
        body_state_imu[7].append(msg.linear_acceleration.x); body_state_imu[8].append(msg.linear_acceleration.y); body_state_imu[9].append(msg.linear_acceleration.z)
        
    if topic == 'body_setp_wrench':
        body_setp_wrench[0].append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
        body_setp_wrench[1].append(msg.wrench.force.x); body_setp_wrench[2].append(msg.wrench.force.y); body_setp_wrench[3].append(msg.wrench.force.z)
        body_setp_wrench[4].append(msg.wrench.torque.x); body_setp_wrench[5].append(msg.wrench.torque.y); body_setp_wrench[6].append(msg.wrench.torque.z)
        
    if topic == 'remote':
        remote[0].append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
        for i in range(0,16):
            remote[i+1].append(msg.channels[i]*100.0)
        
    if topic == 'motor_setp':
        motor_setp[0].append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
        for i in range(0,6):
            motor_setp[i+1].append(msg.throttle[i])
    
    if topic == 'servo_setp':
        servo_setp[0].append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
        for i in range(0,6):
            servo_setp[i+1].append(msg.setpoint[i]*180.0/math.pi)
        
        
bag.close()

###############################################################################

#rescale time
t_min = body_state_imu[0][0]; t_max = body_state_imu[0][-1];
body_state_imu   = fix_time(body_state_imu, t_min,t_max)
body_setp_wrench = fix_time(body_setp_wrench, t_min,t_max)
remote           = fix_time(remote, t_min,t_max)
motor_setp       = fix_time(motor_setp, t_min,t_max)
servo_setp       = fix_time(servo_setp, t_min,t_max)


###############################################################################

#plot helper function
def subplot(i,j, name, time, data, unit):
    axs[i][j].set_title(name)
    for k in range(0,len(data)):
        axs[i][j].plot(time, data[k], '-') 
    axs[i][j].set_ylabel('['+unit+']')
    axs[i][j].yaxis.set_label_coords(-0.05,0.5)

subplot(0,0, 'body_state (pos)', [float("nan")], [[0]], 'm')
subplot(0,1, 'body_state (att)', body_state_imu[0], body_state_imu[1:4], 'deg')
subplot(1,0, 'body_setp (force)', body_setp_wrench[0], body_setp_wrench[1:4], 'N')
subplot(1,1, 'body_setp (torque)', body_setp_wrench[0], body_setp_wrench[4:7], 'Nm')
subplot(2,0, 'remote (xyz)', remote[0], [remote[3],remote[2],remote[1]], '%')
subplot(2,1, 'remote (rpy)', remote[0], [remote[14],remote[16],remote[13]], '%')
#subplot(3,0, 'motor_setp', motor_setp[0], motor_setp[1:7], '%')
#subplot(3,1, 'servo_setp', servo_setp[0], servo_setp[1:7], 'deg')

axs[2][0].set_xlim([0, math.floor(t_max-t_min)])
axs[2][1].set_xlim([0, math.floor(t_max-t_min)])
axs[2][0].set_xlabel('[sec]')
axs[2][1].set_xlabel('[sec]')
axs[2][0].legend(['x','y','z'], loc='upper center', bbox_to_anchor=(0.5, -0.15), fancybox=False, ncol=3)
axs[2][1].legend(['roll','pitch','yaw'], loc='upper center', bbox_to_anchor=(0.5, -0.15), fancybox=False, ncol=3)


plt.show()







