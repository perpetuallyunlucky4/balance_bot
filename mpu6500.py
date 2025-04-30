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
prev_ax, prev_ay, prev_az = 0, 0, 0
prev_gx, prev_gy, prev_gz = 0, 0, 0		#past value for filtering

memlength = 10

ax_vals = deque(maxlen=memlength)
ay_vals = deque(maxlen=memlength)
az_vals = deque(maxlen=memlength)

gx_vals = deque(maxlen=memlength)
gy_vals = deque(maxlen=memlength)
gz_vals = deque(maxlen=memlength)

x_vals = deque(maxlen=memlength)    #creates an array with max length of the memlength: 10

for i in range(memlength):
    x_vals.append(i) #adds the first [memlength] values to start from 0 to [memlength]
    ax_vals.append(0.0)
    ay_vals.append(0.0)
    az_vals.append(0.0)
    gx_vals.append(0.0)
    gy_vals.append(0.0)
    gz_vals.append(0.0)    #fills the initial sensor values to 0s
    
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5, 6))

ax_line,  = ax1.plot(x_vals, ax_vals, color="green")
ay_line,  = ax1.plot(x_vals, ay_vals, color="blue")
az_line,  = ax1.plot(x_vals, az_vals, color="red") #initializes the accelerometer lines to zeros 
ax1.set_ylim(-20, 20) #sets the height of the accel plot
ax1.set_xlim(0, memlength) #sets the width of the accel plot
ax1.set_title("accel")
ax1.grid() #adds grids

gx_line,  = ax2.plot(x_vals, gx_vals, color="green")
gy_line,  = ax2.plot(x_vals, gy_vals, color="blue")
gz_line,  = ax2.plot(x_vals, gz_vals, color="red") #initializes the gyro lines with the zeros
ax2.set_ylim(-250, 250) #sets the the gyro plot
ax2.set_xlim(0, memlength) #sets the width of the gyro plot
ax2.set_title("gyro")
ax2.grid() #adds a grid



bus = smbus2.SMBus(1) #initializes the i2c bus

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
    
    global prev_gx, prev_gy, prev_gz, prev_ax, prev_ay, prev_az, alpha #make the wighting values global
    
    axraw = read_data(accelout_x)/16384.0 * g2ms2
    ayraw = read_data(accelout_y)/16384.0 * g2ms2
    azraw = read_data(accelout_z)/16384.0 * g2ms2	#convert to m/s^2
    
    gxraw = read_data(gyroout_x)/131.0
    gyraw = read_data(gyroout_y)/131.0
    gzraw = read_data(gyroout_z)/131.0	#convert to d/s
    
    if filtered:
        ax = alpha * prev_ax + (1 - alpha) * axraw
        ay = alpha * prev_ay + (1 - alpha) * ayraw
        az = alpha * prev_az + (1 - alpha) * azraw
        
        gx = alpha * prev_gx + (1 - alpha) * gxraw
        gy = alpha * prev_gy + (1 - alpha) * gyraw 
        gz = alpha * prev_gz + (1 - alpha) * gzraw	#filters the output if filtered=True
        
        prev_ax, prev_ay, prev_az = ax, ay, az
        prev_gx, prev_gy, prev_gz = gx, gy, gz #updates teh previous sensor values
    else:
        ax = axraw
        ay = ayraw
        az = azraw
        
        gx = gxraw
        gy = gyraw
        gz = gzraw #if not filtering, the raw data is the outputted data

    print(f"accel x: {ax:.4f}, y: {ay:.4f}, z: {az:.4f}")
    print(f"gyro x: {gx:.4f}, y: {gy:.4f}, z: {gz:.4f}")
    print("======================================") #print the data
    
    return (ax, ay, az), (gx, gy, gz)

def animate(frame):
    last_x = x_vals[-1] if x_vals else 0
    x_vals.append(last_x + 1) # adds to the time array a value one greater then the previous, max length is 10, so the first value gets pushed off
    
    accel, gyro = read_mpu6500()
    ax_val, ay_val, az_val = accel
    gx_val, gy_val, gz_val = gyro #gets the mpu6500 readings
    
    ax_vals.append(ax_val)
    ay_vals.append(ay_val)
    az_vals.append(az_val)
    
    gx_vals.append(gx_val)
    gy_vals.append(gy_val)
    gz_vals.append(gz_val)    #append sensor readings to the value arrays. the last value gets pushed off; the max length is 10
    
    ax1.set_xlim(x_vals[0], x_vals[-1]) #update the time axis
    ax_line.set_data(x_vals, ax_vals)
    ay_line.set_data(x_vals, ay_vals)
    az_line.set_data(x_vals, az_vals) #update accel the lines to show the latest data
    ax1.relim() #redraw limits
    ax1.autoscale_view()
    
    ax2.set_xlim(x_vals[0], x_vals[-1]) #update time limits
    gx_line.set_data(x_vals, gx_vals) 
    gy_line.set_data(x_vals, gy_vals)
    gz_line.set_data(x_vals, gz_vals) #update the gyro lines to show the latest data
    ax2.relim()
    ax2.autoscale_view() #scale the view

animation = animation.FuncAnimation(fig, animate, interval=100) #animates the figure every 100ms, by running the animate function
plt.tight_layout()
plt.show() #show the plot
