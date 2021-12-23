import os
os.system("sudo pigpiod")

import time
import pigpio
time.sleep(1)


### Required for Kalman

import sys
import time
import smbus

import MPU9250
import kalman 

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

sensorfusion = kalman.Kalman()

imu.readSensor()
imu.computeOrientation()
sensorfusion.roll = imu.roll
sensorfusion.pitch = imu.pitch
sensorfusion.yaw = imu.yaw

currTime = time.time()


### Establish a connection so that wifi command center can send commands here
import socket 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_IP = "0.0.0.0"
UDP_PORT = 5050
sock.bind((UDP_IP,UDP_PORT))
## Enter above UDP_PORT in mobile app
time.sleep(1)


## Tell here the pins where ESC are connected (These are gpio numbers and not board numbers)
ESC1 = 27
ESC2 = 17


pi = pigpio.pi()
pi.set_servo_pulsewidth(ESC1, 0)

max_value = 1700
min_value = 1199

hover_pwm = 1500

class ESC:
    def __init__(self, pin):#, location)#, rotation):
        self.gpio_pin = pin
        
        #-------------------------------------------------------------------------------------------
        # Initialize the RPIO DMA PWM for this ESC.
        # In other words, ARM
        #-------------------------------------------------------------------------------------------
        self.pulse_width = 0
        pi.set_servo_pulsewidth(self.gpio_pin, 0)
        time.sleep(0.5)
        pi.set_servo_pulsewidth(self.gpio_pin, max_value)
        time.sleep(0.5)        
        pi.set_servo_pulsewidth(self.gpio_pin, min_value)
        time.sleep(0.5)

    def set(self, pulse_width):
        pulse_width = pulse_width if pulse_width >= min_value else min_value
        pulse_width = pulse_width if pulse_width <= max_value else max_value

        self.pulse_width = pulse_width

        pi.set_servo_pulsewidth(self.gpio_pin, self.pulse_width)
        
        ### This is to debug        
        #print("pulse width :{}".format(self.pulse_width))

    def kill_esc(self):
        pi.set_servo_pulsewidth(self.gpio_pin, 0)


print("Wait 3 sec")
esc1 = ESC(ESC1)
esc2 = ESC(ESC2)
time.sleep(3)
print("ESC Calibrated, Now working on IMU")


import numpy as np
import matplotlib.pyplot as plt
from PID import *

PID_PI_P_GAIN = 2.0
PID_PI_I_GAIN = 0.0
PID_PI_D_GAIN = 0.0

dt_angles = 0.01

import sys

import time



from quat2euler import quat2euler_angle

## Mahony
from mahony_ahrs import Mahony
from ahrs import Quaternion
orientation = Mahony(frequency = 100.0, k_P = 6, k_I = 2)

## Initialize MPU
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

mpu = MPU9250(address_ak = AK8963_ADDRESS, address_mpu_master = MPU9050_ADDRESS_68, address_mpu_slave = None, bus = 1, gfs = GFS_1000, afs = AFS_8G, mfs = AK8963_BIT_16, mode = AK8963_MODE_C100HZ)
## Configure the registers and set calibrated biases 
mpu.configure()
mpu.calibrateMPU6500()
mpu.magScale = [1.381642, 0.97435897, 1.02702703]
mpu.mbias = [18.44951923, 13.66289396, -45.109660220]

mpu.configure()

print("Abias {} Gbias {}, MagScale {}, Mbias{} ".format(mpu.abias, mpu.gbias, mpu.magScale, mpu.mbias))
epochs = 2000
Q = np.tile([1.0, 0.0,0.0,0.0], (epochs,1))


pa_pid = PID(PID_PI_P_GAIN, PID_PI_I_GAIN, PID_PI_D_GAIN)
ang = []

for t in range(1, epochs):
    ## Gives list
    a = mpu.readAccelerometerMaster()
    a = [item*9.80665 for item in a]
    
    g = mpu.readGyroscopeMaster()
    g = [item/57.2958 for item in g]
        
    m = mpu.readMagnetometerMaster()
    #m = [item for item in m]    
        
    Q[t] = orientation.updateMARG(Q[t-1], gyr = g, acc = a, mag = m)    
    X,Y,Z = quat2euler_angle(Q[t,0], Q[t,1], Q[t,2], Q[t,3])
    
    #### KALMAN ####
    #imu.readSensor()
    #imu.computeOrientation()
    newTime = time.time()
    dt = newTime - currTime
    currTime = newTime
    
    sensorfusion.computeAndUpdateRollPitchYaw(a[0], a[1], a[2], g[0], g[1], g[2], m[0], m[1], m[2], 0.01)
    
    ang.append([X, sensorfusion.roll, Y, sensorfusion.pitch,Z , sensorfusion.yaw])
        
    #p_out, i_out, d_out = pa_pid.Compute(X, 0, dt_angles)
    #error = p_out + i_out + d_out
    #err.append(error)
    #print(error)    
    #pwm_to_give = hover_pwm + error
    #esc1.set(int(round(pwm_to_give)))    
    #time.sleep(0.1)
    
    ## Debug
    #print("PWM: {:.2f}\t error:{:.2f} PITCH: {:.2f}\t ROLL: {:.2f}\t YAW: {:.2f}".format(pwm_to_give, error, X, Y, Z))
    #print("PITCH: {:.2f} {:.2f} \t ROLL: {:.2f} {:.2f}\t YAW: {:.2f} {:.2f}".format(X, sensorfusion.roll, Y, sensorfusion.pitch, Z, sensorfusion.yaw))
    #print("PITCH Error: {:.2f}\t ROLL Error: {:.2f}\t YAW Error: {:.2f}".format(X-sensorfusion.roll, Y-sensorfusion.pitch, Z-sensorfusion.yaw))
    #print("PITCH {:.2f} {:.2f} {:.2f}\t ROLL {:.2f} {:.2f} {:.2f}".format(X, sensorfusion.roll, X - sensorfusion.roll, Y, sensorfusion.pitch, Y - sensorfusion.pitch))
    print("YAW: ORTH {:.2f} \t KAL {:.2f}\t Err {:.2f}".format(Z, sensorfusion.yaw, Z-sensorfusion.yaw))    
    time.sleep(0.01)
    
    
ang = np.array(ang)

fig, axs = plt.subplots(3)
fig.suptitle("Error between Mahony and Kalman Filters")
axs[0].plot(ang[:,0], label = "Mahony")
axs[0].plot(ang[:,1], label = "Kalman")
axs[0].grid()
axs[0].legend("MK")
axs[0].set_ylabel("Pitch (degree)")
axs[1].plot(ang[:,2], label = "Mahony")
axs[1].plot(ang[:,3], label = "Kalman")
axs[1].legend("MK")
axs[1].set_ylabel("Roll (degree)")
axs[1].grid()
axs[2].plot(ang[:,4], label = "Mahony")
axs[2].plot(ang[:,5], label = "Kalman")
#axs[2].legend("Yaw")
axs[2].set_ylabel("Yaw (degree)")
axs[2].grid()

plt.legend()
plt.show()

esc1.kill_esc()
esc2.kill_esc()
pi.stop()

