
import os, sys
import rosbag
import math
import numpy as np
from scipy.signal import butter, lfilter, freqz
import matplotlib.pyplot as plt
plt.switch_backend('QT5Agg')
import tkinter as tk
from tkinter import filedialog
from copy import deepcopy
import pandas as pd
from scipy.spatial.transform import Rotation as R


###############################################################################

class BagReader:
    
    topics = {
        'body_state_imu':    '/momav/body_state_imu',
        'body_setp_wrench':  '/momav/body_setp_wrench',
        'body_state_wrench': '/rokubimini/ft_sensor0/ft_sensor_readings/wrench',
        'remote':            '/momav/remote',
        'motor_setp':        '/momav/motor_setp',
        'servo_setp':        '/momav/servo_setp',
        'motor_state':       '/momav/motor_state',
        'servo_state':       '/momav/servo_state',
        'calib_stamp':       '/momav/calib_stamp'
    }
    topics_flip = dict((v,k) for k,v in topics.items())
    topics_type = dict((k,'') for k in topics.keys())
    data_raw = dict((k,[]) for k in topics.keys())
    data = dict((k,[]) for k in topics.keys())
    
    def __init__(self, path=None):
        if path==None: path = self.open_file_dialog()
        self.read_data(path)

    def open_file_dialog(self):
        root = tk.Tk()
        root.overrideredirect(True)
        root.geometry('0x0+0+0')
        root.focus_force()
        file_path = filedialog.askopenfilename(parent=root, initialdir=os.getcwd(), filetypes=[("rosbag", "*.bag")])
        root.destroy()
        if file_path == '': sys.exit()
        return file_path
    
    
    # READ DATA ###############################################################
    
    def read_data(self, path):
        bag_file = rosbag.Bag(path)
        
        # HELPER FUNCTIONS ####################################################
        
        def quaternion_to_euler_angle(w, x, y, z):
            r = R.from_quat([x, y, z, w])
            e = r.as_euler('zyx', degrees=True)
            return e[0], e[1], e[2]

        def fix_time(data, t_min, t_max):
            t_len = math.floor(t_max-t_min)
            data = np.array(data)
            data[0] = data[0] - t_min
            mask = (data[0]>-0.01)&(data[0]<t_len+0.01)
            data = data.compress(mask, axis=1)
            data = list(data)
            return data
        
        def fix_angle_overflow(data):
            prev = float('nan')
            for idx in range(0,len(data)):
                if np.isnan(data[idx]): data[idx] = prev
                if data[idx] - prev > +math.pi: data[idx] -= 2*math.pi
                if data[idx] - prev < -math.pi: data[idx] += 2*math.pi
                prev = data[idx]
        
        
        # READ RAW DATA #######################################################
        
        for topic, msg, t in bag_file.read_messages(topics=list(self.topics.values())):
            topic = self.topics_flip[topic]
            topic_type = type(msg).__name__
            self.topics_type[topic] = topic_type
            
            if topic_type == '_sensor_msgs__Imu':
                if self.data_raw[topic] == []: self.data_raw[topic] = [[],[],[],[],[],[],[],[],[],[]] #t, r,p,y, dr,dp,dy, ddx,ddy,ddz
                
                quat = quaternion_to_euler_angle(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
                
                self.data_raw[topic][0].append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
                self.data_raw[topic][1].append(quat[0]);                   self.data_raw[topic][2].append(quat[1]);                   self.data_raw[topic][3].append(quat[2])
                self.data_raw[topic][4].append(msg.angular_velocity.x);    self.data_raw[topic][5].append(msg.angular_velocity.y);    self.data_raw[topic][6].append(msg.angular_velocity.z)
                self.data_raw[topic][7].append(msg.linear_acceleration.x); self.data_raw[topic][8].append(msg.linear_acceleration.y); self.data_raw[topic][9].append(msg.linear_acceleration.z)
                
            if topic_type == '_geometry_msgs__WrenchStamped':
                if self.data_raw[topic] == []: self.data_raw[topic] = [[],[],[],[],[],[],[]] #t, fx,fy,fz, tx,ty,tz
                
                self.data_raw[topic][0].append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
                self.data_raw[topic][1].append(msg.wrench.force.x);  self.data_raw[topic][2].append(msg.wrench.force.y);  self.data_raw[topic][3].append(msg.wrench.force.z)
                self.data_raw[topic][4].append(msg.wrench.torque.x); self.data_raw[topic][5].append(msg.wrench.torque.y); self.data_raw[topic][6].append(msg.wrench.torque.z)
                
            if topic_type == '_momav__Remote':
                if self.data_raw[topic] == []: self.data_raw[topic] = [[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]] #t, channels 1-16
                
                self.data_raw[topic][0].append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
                for i in range(0,16):
                    self.data_raw[topic][i+1].append(msg.channels[i]*100.0)
                
            if topic_type == '_momav__MotorSetp':
                if self.data_raw[topic] == []: self.data_raw[topic] = [[],[],[],[],[],[],[]] #t, throttles 1-6
                
                self.data_raw[topic][0].append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
                for i in range(0,6):
                    self.data_raw[topic][i+1].append(msg.throttle[i])
            
            if topic_type == '_momav__ServoSetp':
                if self.data_raw[topic] == []: self.data_raw[topic] = [[],[],[],[],[],[],[]] #t, angle setpoints 1-6
                
                self.data_raw[topic][0].append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
                for i in range(0,6):
                    self.data_raw[topic][i+1].append(msg.setpoint[i])
                    
            if topic_type == '_momav__MotorState':
                if self.data_raw[topic] == []: self.data_raw[topic] = [[],[],[],[],[],[],[]] #t, rpm 1-6
                
                self.data_raw[topic][0].append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
                for i in range(0,6):
                    self.data_raw[topic][i+1].append(msg.rpm[i])
            
            if topic_type == '_momav__ServoState':
                if self.data_raw[topic] == []: self.data_raw[topic] = [[],[],[],[],[],[],[]] #t, angle positions 1-6
                
                self.data_raw[topic][0].append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
                for i in range(0,6):
                    self.data_raw[topic][i+1].append(msg.position[i])
                    
            if topic_type == '_std_msgs__Header':
                if self.data_raw[topic] == []: self.data_raw[topic] = [[]]
                self.data_raw[topic][0].append(msg.stamp.secs + msg.stamp.nsecs/1e9)
        
        bag_file.close()
        
        
        # FIX TIME  ###########################################################
        
        self.t_min = float('inf'); self.t_max = float('-inf')
        for topic in self.topics.keys():
            if self.data_raw[topic] == []: continue
            if (self.t_min > self.data_raw[topic][0][0]):  self.t_min = self.data_raw[topic][0][0]
            if (self.t_max < self.data_raw[topic][0][-1]): self.t_max = self.data_raw[topic][0][-1]
        
        for topic in self.topics.keys():
            if self.data_raw[topic] == []: continue
            self.data_raw[topic] = fix_time(self.data_raw[topic], self.t_min,self.t_max)
            
        time = np.arange(0, self.t_max-self.t_min, 0.01)
        for topic in self.topics.keys():
            if self.data_raw[topic] == []: continue
            self.data[topic] = deepcopy(self.data_raw[topic])
            self.data[topic][0] = time.copy()
            for i in range(1,len(self.data[topic])):
                self.data[topic][i] = np.interp(self.data[topic][0], self.data_raw[topic][0], self.data_raw[topic][i])


bag = BagReader('bags/032.bag')

###############################################################################

def plot_start():
    plt.rc('font', size=8)
    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(6.8,3.8))
    plt.subplots_adjust(top=0.945, bottom=0.175, left=0.08, right=0.99, hspace=0.235, wspace=0.005)
    #figManager = plt.get_current_fig_manager()
    #figManager.window.showMaximized()
    return fig, axs

def subplot(axis, name, time, data, unit):
    axis.set_title(name)
    for k in range(0,len(data)):
        axis.plot(time, data[k], '-')
    axis.set_ylabel('['+unit+']')
    axis.yaxis.set_label_coords(-0.07,0.5)
    axis.grid(True)

###############################################################################

#trim time
for topic in bag.data:
    for series in range(len(bag.data[topic])):
        bag.data[topic][series] = bag.data[topic][series][3000:6500]

#set attitude offset
bag.data['body_state_imu'][1] -= np.mean(bag.data['body_state_imu'][1])
bag.data['body_state_imu'][2] -= np.mean(bag.data['body_state_imu'][2])
bag.data['body_state_imu'][3] -= np.mean(bag.data['body_state_imu'][3])
#bag.data['body_setp_wrench'][4] -= np.mean(bag.data['body_setp_wrench'][4])
#bag.data['body_setp_wrench'][5] -= np.mean(bag.data['body_setp_wrench'][5])
#bag.data['body_setp_wrench'][6] -= np.mean(bag.data['body_setp_wrench'][6])

for i in range(1,7): bag.data['servo_setp'][i] *= 180.0/math.pi



###############################################################################

fig, axs = plot_start()

subplot(axs[0], 'Body Orientation', bag.data['body_state_imu'][0], bag.data['body_state_imu'][1:4], 'deg')
subplot(axs[1], 'Torque Setpoint', bag.data['body_setp_wrench'][0], bag.data['body_setp_wrench'][4:7], 'Nm')

axs[1].set_xlabel('[sec]')
axs[1].legend(['Roll','Pitch','Yaw'], loc='upper center', bbox_to_anchor=(0.5, -0.3), fancybox=False, ncol=3)
axs[1].set_xlim(30, 65)


fig, axs = plot_start()

subplot(axs[0], 'Arm Angle Setpoint', bag.data['servo_setp'][0], bag.data['servo_setp'][1:7], 'deg')
subplot(axs[1], 'Throttle Setpoint', bag.data['motor_setp'][0], bag.data['motor_setp'][1:7], '%')

axs[1].set_xlabel('[sec]')
axs[1].legend(['Arm 1','Arm 2','Arm 3','Arm 4','Arm 5','Arm 6'], loc='upper center', bbox_to_anchor=(0.5, -0.3), fancybox=False, ncol=6)
axs[1].set_xlim(30, 65)











