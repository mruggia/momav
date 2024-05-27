
import os, sys
import rosbag
import math
import numpy as np
from scipy.signal import butter, lfilter, freqz
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import filedialog
from copy import deepcopy
import pandas as pd


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


bag = BagReader()

###############################################################################

fig = []; axs = []
def plot_start():
    global fig, axs
    plt.switch_backend('QT5Agg')
    plt.rc('font', size=10)
    fig, axs = plt.subplots(3, 2, sharex=True)
    plt.subplots_adjust(top=0.973, bottom=0.078, left=0.035, right=0.994, hspace=0.151, wspace=0.075)
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()

def subplot(i,j, name, time, data, unit):
    axs[i][j].set_title(name)
    for k in range(0,len(data)):
        axs[i][j].plot(time, data[k], '.-')
    axs[i][j].set_ylabel('['+unit+']')
    axs[i][j].yaxis.set_label_coords(-0.05,0.5)
    
def plot_show():
    axs[2][0].set_xlabel('[sec]')
    axs[2][1].set_xlabel('[sec]')
    axs[2][0].legend(['x','y','z'], loc='upper center', bbox_to_anchor=(0.5, -0.15), fancybox=False, ncol=3)
    axs[2][1].legend(['roll','pitch','yaw'], loc='upper center', bbox_to_anchor=(0.5, -0.15), fancybox=False, ncol=3)



###############################################################################


#clean data
def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter(order, cutoff, fs=fs, btype='low', analog=False)
    y = lfilter(b, a, data)
    return y

for i in range(1,7):
    bag.data['body_state_wrench'][i] = bag.data['body_state_wrench'][i] - bag.data['body_state_wrench'][i][0]
    bag.data['body_state_wrench'][i] = butter_lowpass_filter(bag.data['body_state_wrench'][i], 2.0, 100.0, 5)
    
    
gyro_time = []
gyro_data = [[],[],[],[],[],[],[],[],[]] #motor_id,angle,throttle, fx,fy,fz, tx,ty,tz
for i in range(0, len(bag.data['motor_setp'][0])):

    motor_id = -1; angle = -1; throttle = -1
    for j in range(0,6):
        if bag.data['motor_setp'][j+1][i] > throttle:
            motor_id = j
            throttle = bag.data['motor_setp'][j+1][i]
            angle = bag.data['servo_setp'][j+1][i]

    if not (np.isclose(throttle,0.3) or np.isclose(throttle,0.5) or np.isclose(throttle,0.7)): continue
    
    gyro_time.append(bag.data['body_state_wrench'][0][i])
    for j in range(0,6):
        gyro_data[j+3].append(bag.data['body_state_wrench'][j+1][i])
    
    motor_id = -1; angle = -1; throttle = -1
    for j in range(0,6):
        if bag.data['motor_setp'][j+1][i] > throttle:
            motor_id = j
            throttle = bag.data['motor_setp'][j+1][i]
            angle = bag.data['servo_setp'][j+1][i]
    gyro_data[0].append(motor_id)
    gyro_data[1].append(angle)
    gyro_data[2].append(throttle)



csv = 'motor_id;angle;throttle;fx;fy;fz;tx;ty;tz\n'
for t in range(0,len(gyro_data[0])):
    for i in range(0,len(gyro_data)):
        csv += str(gyro_data[i][t]) + ";"
    csv += '\n'
csv_file = open("data_motors.csv", "w")
csv_file.write(csv)
csv_file.close()


plot_start()

subplot(1,0, 'body_state_wrench (force)',  bag.data['body_state_wrench'][0], bag.data['body_state_wrench'][1:4], 'N')
subplot(1,1, 'body_state_wrench (torque)', bag.data['body_state_wrench'][0], bag.data['body_state_wrench'][4:7], 'Nm')
subplot(2,0, 'motor_state', bag.data_raw['motor_setp'][0], bag.data_raw['motor_setp'][1:7], 'rpm')
subplot(2,1, 'servo_state', bag.data_raw['servo_state'][0], bag.data_raw['servo_state'][1:7], 'deg')

subplot(0,0, 'calib_stamp', bag.data_raw['calib_stamp'][0], [bag.data_raw['calib_stamp'][0]*0.0], 'deg')

plot_show()


###############################################################################

#plot_start()
#
#subplot(0,0, 'body_state (pos)', [float("nan")], [[0]], 'm')
#subplot(0,1, 'body_state (att)', bag.data['body_state_imu'][0], bag.data['body_state_imu'][1:4], 'deg')
#
#plot_show()

