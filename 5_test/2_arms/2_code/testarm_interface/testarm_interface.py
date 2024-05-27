import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import threading
import time
from datetime import datetime

import serial
from cobs import cobs
from struct import unpack, pack

from BotaSerialSensor import *

import os
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)

###############################################################################

arm_com_port = 'COM12'
bota_com_port = 'COM9'

plot_t_step = 10
plot_t_sec = 4
plot_t_data = list(np.arange(-plot_t_sec,0, plot_t_step/1000))
plot_t_len = len(plot_t_data)

srv_pos_data = np.empty(plot_t_len); srv_pos_data[:] = np.NaN
srv_stp_data = np.empty(plot_t_len); srv_stp_data[:] = np.NaN
srv_err_data = np.empty(plot_t_len); srv_err_data[:] = np.NaN
srv_vlt_data = np.empty(plot_t_len); srv_vlt_data[:] = np.NaN
esc_stp_data = np.empty(plot_t_len); esc_stp_data[:] = np.NaN
esc_vlt_data = np.empty(plot_t_len); esc_vlt_data[:] = np.NaN
esc_cur_data = np.empty(plot_t_len); esc_cur_data[:] = np.NaN
esc_rpm_data = np.empty(plot_t_len); esc_rpm_data[:] = np.NaN
bota_fx_data = np.empty(plot_t_len); bota_fx_data[:] = np.NaN
bota_fy_data = np.empty(plot_t_len); bota_fy_data[:] = np.NaN
bota_fz_data = np.empty(plot_t_len); bota_fz_data[:] = np.NaN
bota_mx_data = np.empty(plot_t_len); bota_mx_data[:] = np.NaN
bota_my_data = np.empty(plot_t_len); bota_my_data[:] = np.NaN
bota_mz_data = np.empty(plot_t_len); bota_mz_data[:] = np.NaN

log_txt = 'com_ts,srv_pos,srv_setp,srv_volt,esc_setp,esc_volt,esc_curr,esc_rpm,bota_fx,bota_fy,bota_fz,bota_mx,bota_my,bota_mz\n'
def log_save():
    log_name = datetime.now().strftime("logs/log_%Y-%m-%d_%H-%M-%S.csv")
    log_file = open(log_name, 'w')
    log_file.write(log_txt)
    log_file.close()
log_enable = False
log_start_time = time. time()


###############################################################################
### ARM

print("connecting to arm controller... ", end ="", flush=True)
try:
    arm_ser = serial.Serial('COM12', 2000000, timeout=1)
except:
    print("FAILED")
    exit()
print("SUCCESS")

def read_from_port():
    global srv_pos_data, srv_stp_data, srv_err_data, srv_vlt_data
    global esc_stp_data, esc_vlt_data, esc_cur_data, esc_rpm_data
    global log_txt
    
    while not arm_thread_stop:
        try:
            data = arm_ser.read_until(b'\x00')
            data = cobs.decode (data[0:-1])
            data = unpack('Lfffffff', data)
        except:
            continue
        srv_pos_data = np.roll(srv_pos_data, -1); srv_pos_data[-1] = np.rad2deg(data[1])
        srv_stp_data = np.roll(srv_stp_data, -1); srv_stp_data[-1] = np.rad2deg(data[2])
        srv_err_data = np.roll(srv_err_data, -1); srv_err_data[-1] = np.rad2deg(data[2]-data[1])
        srv_vlt_data = np.roll(srv_vlt_data, -1); srv_vlt_data[-1] = data[3]*8.4
        esc_stp_data = np.roll(esc_stp_data, -1); esc_stp_data[-1] = data[4]
        esc_vlt_data = np.roll(esc_vlt_data, -1); esc_vlt_data[-1] = data[5]
        esc_cur_data = np.roll(esc_cur_data, -1); esc_cur_data[-1] = data[6]
        esc_rpm_data = np.roll(esc_rpm_data, -1); esc_rpm_data[-1] = data[7]
        # if (log_enable):
        #     timestamp = time. time() - log_start_time
        #     log_txt += "%.3f,%.3f,%.3f,%.3f, %.2f,%.2f,%.2f,%.0f, %.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n" % (
        #         timestamp, 
        #         srv_pos_data[-1],srv_stp_data[-1],srv_vlt_data[-1], 
        #         esc_stp_data[-1],esc_vlt_data[-1],esc_cur_data[-1],esc_rpm_data[-1],
        #         bota_fx_data[-1],bota_fy_data[-1],bota_fz_data[-1],
        #         bota_mx_data[-1],bota_my_data[-1],bota_mz_data[-1]
        #     )

arm_thread_stop = False
arm_thread = threading.Thread(target=read_from_port)
arm_thread.start()


###############################################################################
### BOTA SENSOR

bota_fx_offset = 0; bota_fy_offset = 0; bota_fz_offset = 0
bota_mx_offset = 0; bota_my_offset = 0; bota_mz_offset = 0

def bota_callback(timestamp,fx,fy,fz,mx,my,mz,temperature):
    global bota_fx_data, bota_fy_data, bota_fz_data, bota_mx_data, bota_my_data, bota_mz_data
    global log_txt

    fx = fx - bota_fx_offset; fy = fy - bota_fy_offset; fz = fz - bota_fz_offset
    mx = mx - bota_mx_offset; my = my - bota_my_offset; mz = mz - bota_mz_offset

    bota_fx_data = np.roll(bota_fx_data, -1); bota_fx_data[-1] = fx
    bota_fy_data = np.roll(bota_fy_data, -1); bota_fy_data[-1] = fy
    bota_fz_data = np.roll(bota_fz_data, -1); bota_fz_data[-1] = fz
    bota_mx_data = np.roll(bota_mx_data, -1); bota_mx_data[-1] = mx
    bota_my_data = np.roll(bota_my_data, -1); bota_my_data[-1] = my
    bota_mz_data = np.roll(bota_mz_data, -1); bota_mz_data[-1] = mz

    if (log_enable):
        timestamp = time. time() - log_start_time
        log_txt += "%.3f,%.3f,%.3f,%.3f, %.2f,%.2f,%.2f,%.0f, %.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n" % (
            timestamp, 
            srv_pos_data[-1],srv_stp_data[-1],srv_vlt_data[-1], 
            esc_stp_data[-1],esc_vlt_data[-1],esc_cur_data[-1],esc_rpm_data[-1],
            bota_fx_data[-1],bota_fy_data[-1],bota_fz_data[-1],
            bota_mx_data[-1],bota_my_data[-1],bota_mz_data[-1]
        )

def bota_zero():
    global bota_fx_offset, bota_fy_offset, bota_fz_offset
    global bota_mx_offset, bota_my_offset, bota_mz_offset

    bota_fx_offset = np.mean(bota_fx_data)
    bota_fy_offset = np.mean(bota_fy_data)
    bota_fz_offset = np.mean(bota_fz_data)
    bota_mx_offset = np.mean(bota_mx_data)
    bota_my_offset = np.mean(bota_my_data)
    bota_mz_offset = np.mean(bota_mz_data)

print("connecting to bota... ", end ="", flush=True)
try:
    bota_sensor = BotaSerialSensor(bota_com_port)
    bota_sensor.run(bota_callback)
except:
    print("FAILED")
    # exit()
print("SUCCESS")


###############################################################################
### PLOT

print("starting plotter... ", end ="", flush=True)
fig, axs = plt.subplots(3,1, figsize=(25, 12))
fig.suptitle("hold position", fontsize=16) 
plt.subplots_adjust(top=0.931, bottom=0.079, left=0.098, right=0.985, hspace=0.202, wspace=0.2)

srv_ax_ang = axs[0]
srv_ax_ang.set_ylim(-15,15)
srv_ax_ang.set_xlim(-plot_t_sec, 0)
srv_ax_ang.set_xlabel("time [s]")
srv_ax_ang.set_ylabel("angle [°]")
srv_ax_ang.set_xticks(list(np.arange(-plot_t_sec,0, 0.1)), minor=True)
srv_ax_ang.grid(which='major', alpha=0.5)
srv_ax_ang.grid(which='minor', alpha=0.2)

srv_ax_vlt = srv_ax_ang.twinx()
srv_ax_vlt.set_ylim(-12.6, 12.6)
srv_ax_vlt.set_ylabel("voltage [V]")
srv_ax_vlt.spines['left'].set_position(("axes", -0.04))
srv_ax_vlt.yaxis.set_ticks([-11.2, -8.4, -5.6, -2.8, 0, 2.8, 5.6, 8.4, 11.2])
srv_ax_vlt.yaxis.set_label_position('left')
srv_ax_vlt.yaxis.set_ticks_position('left')

srv_pos_plt, = srv_ax_ang.plot([], [], label='servo position [°]')
srv_stp_plt, = srv_ax_ang.plot([], [], label='servo setpoint [°]')
srv_err_plt, = srv_ax_ang.plot([], [], label='servo error [°]')
srv_vlt_plt, = srv_ax_vlt.plot([], [], label='servo voltage [V]', color='tab:red')

srv_ax_ang.legend(handles=[srv_pos_plt,srv_stp_plt,srv_err_plt,srv_vlt_plt], loc='upper left')
srv_ax_ang.legend(handles=[srv_pos_plt,srv_stp_plt,srv_err_plt,srv_vlt_plt], loc='upper left')

esc_ax_rpm = axs[1]
esc_ax_rpm.set_ylim(0, 20000)
esc_ax_rpm.set_xlim(-plot_t_sec, 0)
esc_ax_rpm.set_xlabel("time [s]")
esc_ax_rpm.set_ylabel("revolutions [rpm]")
esc_ax_rpm.set_xticks(list(np.arange(-plot_t_sec,0, 0.1)), minor=True)
esc_ax_rpm.grid(which='major', alpha=0.5)
esc_ax_rpm.grid(which='minor', alpha=0.2)

esc_ax_vlt = esc_ax_rpm.twinx()
esc_ax_vlt.set_ylim(18, 26)
esc_ax_vlt.set_ylabel("voltage [V]")
esc_ax_vlt.spines['left'].set_position(("axes", -0.075))
esc_ax_vlt.yaxis.set_label_position('left')
esc_ax_vlt.yaxis.set_ticks_position('left')

esc_ax_cur = esc_ax_rpm.twinx()
esc_ax_cur.set_ylim(0, 40)
esc_ax_cur.set_ylabel("current [A]")
esc_ax_cur.spines['left'].set_position(("axes", -0.04))
esc_ax_cur.yaxis.set_label_position('left')
esc_ax_cur.yaxis.set_ticks_position('left')

esc_rpm_plt, = esc_ax_rpm.plot([], [], label='motor revolutions [rpm]', color='tab:blue')
esc_vlt_plt, = esc_ax_vlt.plot([], [], label='motor voltage [V]', color='tab:orange')
esc_cur_plt, = esc_ax_cur.plot([], [], label='motor current [A]', color='tab:green')

esc_ax_rpm.legend(handles=[esc_rpm_plt,esc_vlt_plt,esc_cur_plt], loc='upper left')
esc_ax_vlt.legend(handles=[esc_rpm_plt,esc_vlt_plt,esc_cur_plt], loc='upper left')
esc_ax_cur.legend(handles=[esc_rpm_plt,esc_vlt_plt,esc_cur_plt], loc='upper left')

bota_ax_frc = axs[2]
bota_ax_frc.set_ylim(-30, 30)
bota_ax_frc.set_xlim(-plot_t_sec, 0)
bota_ax_frc.set_xlabel("time [s]")
bota_ax_frc.set_ylabel("force [N]")
bota_ax_frc.set_xticks(list(np.arange(-plot_t_sec,0, 0.1)), minor=True)
bota_ax_frc.grid(which='major', alpha=0.5)
bota_ax_frc.grid(which='minor', alpha=0.2)

bota_ax_mom = bota_ax_frc.twinx()
bota_ax_mom.set_ylim(-0.2, 0.2)
bota_ax_mom.set_ylabel("moment [Nm]")
bota_ax_mom.spines['left'].set_position(("axes", -0.04))
bota_ax_mom.yaxis.set_label_position('left')
bota_ax_mom.yaxis.set_ticks_position('left')

bota_frc_plt, = bota_ax_frc.plot([], [], label='motor thrust [N]', color='tab:blue')
bota_mom_plt, = bota_ax_mom.plot([], [], label='motor torque [Nm]', color='tab:orange')

bota_ax_frc.legend(handles=[bota_frc_plt,bota_mom_plt], loc='upper left')
bota_ax_mom.legend(handles=[bota_frc_plt,bota_mom_plt], loc='upper left')

def on_update(data):
    srv_pos_plt.set_data(plot_t_data, srv_pos_data)
    srv_stp_plt.set_data(plot_t_data, srv_stp_data)
    srv_err_plt.set_data(plot_t_data, srv_err_data)
    srv_vlt_plt.set_data(plot_t_data, srv_vlt_data)
    
    esc_rpm_plt.set_data(plot_t_data, esc_rpm_data)
    esc_vlt_plt.set_data(plot_t_data, esc_vlt_data)
    esc_cur_plt.set_data(plot_t_data, esc_cur_data)

    bota_frc_plt.set_data(plot_t_data, np.sqrt(np.square(bota_fy_data)+np.square(bota_fz_data)) )
    #bota_mom_plt.set_data(plot_t_data, bota_mz_data)

def on_close(event):
    
    send_servo_command(0)
    send_throttle_command(0)
    
    global arm_thread_stop, traj_thread_stop
    arm_thread_stop = True
    traj_thread_stop = True
    arm_thread.join()
    traj_thread.join()

    send_servo_command(0)
    send_throttle_command(0)

    arm_ser.close()
    bota_sensor.stop()
    exit()

fig.canvas.mpl_connect('close_event', on_close)
ani = animation.FuncAnimation(fig, on_update, interval=10, repeat=False, cache_frame_data=False)
print("SUCCESS")


###############################################################################
### SEND COMMANDS

# def send_trajectory():
#     global log_enable

#     send_hold(0.0,plot_t_sec+1)
#     bota_zero()
#     send_hold(0.0,1)
#     log_enable = True

#     send_hold(0.0,1)
#     send_throttle_ramp(0.8)
#     send_hold(0.0,1)
    
#     send_steps(np.pi/24, 1, 4)
#     send_steps(np.pi/12, 1, 4)
#     send_steps(np.pi/6, 1, 4)
#     send_steps(np.pi, 1, 4)
    
#     send_hold(0.0, 2)
    
#     send_chirp(np.pi/24, 0.5, 8, 8)
#     send_chirp(np.pi/12, 0.5, 8, 8)
#     send_chirp(np.pi/6, 0.5, 8, 8)
#     send_chirp(np.pi/3, 0.5, 8, 8)
    
#     send_hold(0.0,2)
    
#     send_ramp(4*np.pi, 8)
    
#     send_hold(0.0,1)
#     send_throttle_ramp(0.0)
#     srv_ax_ang.set_ylim(-15,15)
#     send_hold(0.0,1)
#     log_enable = False
#     log_save()

def send_trajectory():
    global log_enable

    print("running test... ")
    send_throttle_command(0)
    send_hold(0.0,plot_t_sec+1)
    bota_zero()
    send_hold(0.0,1)

    motor_start = 0.2; motor_step = 0.05; motor_stop = 0.8
    throttles = np.arange(motor_start, motor_stop+0.001, motor_step)
    for throttle in throttles:
        send_throttle_ramp(throttle)
        print('throttle: %.2f' % throttle)
        send_hold(0.0,1)

        log_enable = True
        send_hold(0.0,1)
        send_ramp_single(2*np.pi, 16*2)
        send_hold(0.0,1)
        log_enable = False

    log_save()
    send_throttle_ramp(0.0)
    print("SUCCESS")
    
    
def send_steps(angle, delay, count):
    fig.suptitle("step signal " + str(round(2*np.rad2deg(angle),1)) + "°", fontsize=16) 
    srv_ax_ang.set_ylim(np.rad2deg(-angle*1.5), np.rad2deg(angle*1.5))
    for i in range(count):
        send_servo_command(angle)
        if traj_thread_stop: return
        time.sleep(delay)
        send_servo_command(-angle)
        if traj_thread_stop: return
        time.sleep(delay)
        
def send_chirp(angle, f_start, f_end, duration):
    fig.suptitle("chirp signal +/-" + str(round(np.rad2deg(angle),1)) + "° @ "+str(f_start)+"Hz - "+str(f_end)+"Hz", fontsize=16) 
    srv_ax_ang.set_ylim(np.rad2deg(-angle*1.5), np.rad2deg(angle*1.5))
    t = 0
    c = ( f_end - f_start ) /duration
    t_start = time.time()
    while (t<duration):
        val = np.sin( 2*np.pi*( c/2*t*t + f_start*t ))
        send_servo_command(val*angle)
        t += 0.01
        dt = (t_start+t)-time.time()
        if (dt > 0): time.sleep(dt)
        if traj_thread_stop: return
    send_servo_command(0)
    time.sleep(1)
    
def send_ramp(angle, duration):
    fig.suptitle("ramp signal " + str(round(2*np.rad2deg(angle),1)) + "°", fontsize=16) 
    srv_ax_ang.set_ylim(np.rad2deg(-angle*1.5), np.rad2deg(angle*1.5))
    t = 0
    t_start = time.time()
    while(t<duration/4):
        send_servo_command( angle*(t/(duration/4)) )
        t += 0.01
        dt = (t_start+t)-time.time()
        if (dt > 0): time.sleep(dt)
        if traj_thread_stop: return
    while(t<duration*3/4):
        send_servo_command( angle-2*angle*((t-duration/4)/(duration/2)) )
        t += 0.01
        dt = (t_start+t)-time.time()
        if (dt > 0): time.sleep(dt)
        if traj_thread_stop: return
    while(t<duration):
        send_servo_command( -angle+angle*((t-duration*3/4)/(duration/4)) )
        t += 0.01
        dt = (t_start+t)-time.time()
        if (dt > 0): time.sleep(dt)
        if traj_thread_stop: return
    send_servo_command(0)
    time.sleep(1)

def send_ramp_single(angle, duration):
    fig.suptitle("ramp signal " + str(round(2*np.rad2deg(angle),1)) + "°", fontsize=16) 
    srv_ax_ang.set_ylim(np.rad2deg(-angle*1.5), np.rad2deg(angle*1.5))
    t = 0
    t_start = time.time()
    while(t<duration/2):
        send_servo_command( angle*(t/(duration/2)) )
        t += 0.01
        dt = (t_start+t)-time.time()
        if (dt > 0): time.sleep(dt)
        if traj_thread_stop: return
    while(t<duration):
        send_servo_command( angle-angle*((t-duration/2)/(duration/2)) )
        t += 0.01
        dt = (t_start+t)-time.time()
        if (dt > 0): time.sleep(dt)
        if traj_thread_stop: return
    send_servo_command(0)
        
def send_hold(angle, duration):
    if traj_thread_stop: return
    fig.suptitle("hold position " + str(round(np.rad2deg(angle),1)) + "°", fontsize=16) 
    send_servo_command(angle)
    time.sleep(duration)

def send_throttle_ramp(pos_goal):
    pos_curr = send_throttle_ramp.pos_old
    while ( abs(pos_curr-pos_goal) >= 0.005):
        if traj_thread_stop:
            send_throttle_command(0)
            return
        pos_curr -= np.sign(pos_curr-pos_goal)*0.005
        send_throttle_command(pos_curr)
        time.sleep(0.02)
    send_throttle_command(pos_goal)
    send_throttle_ramp.pos_old = pos_goal
send_throttle_ramp.pos_old = 0.0

def send_servo_command(pos):
    data = b'\x01' + pack('f', pos)
    data = cobs.encode(data) + b'\x00'
    arm_ser.write(data)
    
def send_throttle_command(pos):
    data = b'\x02' + pack('f', pos)
    data = cobs.encode(data) + b'\x00'
    arm_ser.write(data)

traj_thread_stop = False
traj_thread = threading.Thread(target=send_trajectory)
traj_thread.start()


###############################################################################

plt.show()

