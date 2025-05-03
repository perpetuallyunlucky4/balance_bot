import smbus2
import time
from collections import deque
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation


mpu6500_addr = 0x68 #run i2cdetect -y 1 to find the address of the imu

pwr_mgmt = 0x6B #address in internal memory for sleeping and waking the imu
smplrt = 0x19 #sets sample rate
config = 0x1A
gyro_config = 0x1B
accel_config = 0x1C		#gyroscope and accelometer configuration

accelout_x = 0x3B
accelout_y = 0x3D
accelout_z = 0x3F
gyroout_x = 0x43
gyroout_y = 0x45
gyroout_z = 0x47	#addresses of the imu output

g2ms2 = 9.81#g to m/s^2

alpha = 0.5#filtering constant
prev_a = np.zeros(3)
prev_g = np.zeros(3) #initialize the filtering history

memlength = 50 #number of data history to plot

x_vals = deque(range(memlength), maxlen=memlength) #create time values

accel_data = [deque([0.0]*memlength, maxlen=memlength) for i in range(3)]
gyro_data = [deque([0.0]*memlength, maxlen=memlength) for i in range(3)]	#fill imu data history with 0.0s


bus = smbus2.SMBus(1) #initialize the bus

def init_mpu6500():
    bus.write_byte_data(mpu6500_addr, pwr_mgmt, 0) #starts in sleep mode, setting pwr_mgmt to 0 awakens it
    time.sleep(0.1)
    
    bus.write_byte_data(mpu6500_addr, smplrt, 7) #setting smplrt to 7 configures the sample rate to 1khz
    
    bus.write_byte_data(mpu6500_addr, accel_config, 0)
    bus.write_byte_data(mpu6500_addr, gyro_config, 0)	#sets the acclerometer and gyroscope to their default accuracies
    
def read_data(addr):
    high = bus.read_byte_data(mpu6500_addr, addr)
    low = bus.read_byte_data(mpu6500_addr, addr+1)
    
    value = ((high << 8) | low) #combine bytes
    
    if value > 32767:
        value = value - 65536 #twos complement: makes the bytes signed integers
    
    return value

def read_mpu6500(filtered=True):
    
    global prev_g, prev_a, alpha #make the wighting values global
    
    accel_raw = np.array([
        read_data(accelout_x)/16384.0 * g2ms2,
        read_data(accelout_y)/16384.0 * g2ms2,
        read_data(accelout_z)/16384.0 * g2ms2,
    ])
    
    gyro_raw = np.array([
        read_data(gyroout_x)/131.0,
        read_data(gyroout_y)/131.0,
        read_data(gyroout_z)/131.0,
    ])	#create numpy arrays containing the sensor values
    
    if filtered:
        accel = alpha * prev_a + (1 - alpha) * accel_raw
        gyro = alpha * prev_g + (1 - alpha) * gyro_raw
        
        prev_a = accel
        prev_g = gyro #filter the values if fitered = True
        
    else:
        accel = accel_raw
        gyro = gyro_raw #if filtered = False, the output is the raw data

    print(f"accel x: {accel[0]:.4f}, y: {accel[1]:.4f}, z: {accel[2]:.4f}")
    print(f"gyro x: {gyro[0]:.4f}, y: {gyro[0]:.4f}, z: {gyro[0]:.4f}")
    print("======================================") #print the data to 4 .sf
    
    return accel, gyro


def animate(i):
    x_vals.append(x_vals[-1] + 1 if x_vals else 0) #update the time frame
    accel, gyro = read_mpu6500() #read data
    
    for i in range(3):
        accel_data[i].append(accel[i])
        gyro_data[i].append(gyro[i]) #add data to the deque, last value gets pushed off
        
        accel_lines[i].set_data(x_vals, accel_data[i])
        gyro_lines[i].set_data(x_vals, gyro_data[i]) #update the lines
        
    ax1.set_xlim(x_vals[0], x_vals[-1])
    ax2.set_xlim(x_vals[0], x_vals[-1]) #update the screen with the time frame
    
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5, 6)) #create plots
ax1.set_title("accel")
ax2.set_title("gyro")

ax1.grid()
ax2.grid()

accel_lines = [ax1.plot(x_vals, data, color=color)[0] for data, color in zip(accel_data, ["red", "green", "blue"])]
gyro_lines = [ax2.plot(x_vals, data, color=color)[0] for data, color in zip(gyro_data, ["red", "green", "blue"])] #create an array of lines with the colors and data

ax1.set_ylim(-20, 20) #sets the height of the accel plot
ax1.set_xlim(0, memlength) #sets the width of the accel plot

ax2.set_ylim(-250, 250) #sets the the gyro plot
ax2.set_xlim(0, memlength) #sets the width of the gyro plot

#=================== main code ===================#

init_mpu6500()

ani = animation.FuncAnimation(fig, animate, interval=100)
plt.tight_layout()
plt.show()



