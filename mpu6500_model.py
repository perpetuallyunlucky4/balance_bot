import smbus
import time
import math

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

bus = smbus.SMBus(1) #initializes the i2c bus

def mpu6500_init():
    bus.write_byte_data(mpu6500_addr, pwr_mgmt, 0) #starts in sleep mode, setting pwr_mgmt to 0 awakens it
    time.sleep(0.1)
    
    bus.write_byte_data(mpu6500_addr, smplrt, 7) #setting smplrt to 7 configures the sample rate to 1khz
    
    bus.write_byte_data(mpu6500_addr, accel_config, 0)
    bus.write_byte_data(mpu6500_addr, gyro_config, 0)	#sets the acclerometer and gyroscope to their default accuracies
    
def read_data(addr):
    high = bus.read_byte_data(mpu6500_addr, addr)
    low = bus.read_byte_data(mpu6500_addr, addr+1)
    
    value = ((high << 8) | low) #combine bytes
    
    if value > 3276:
        value = value - 65536
    
    return value

def read_mpu6500():
    ax = read_data(accelout_x)/16384.0
    ay = read_data(accelout_y)/16384.0
    az = read_data(accelout_z)/16384.0
    
    gx = read_data(gyroout_x)/131.0
    gy = read_data(gyroout_y)/131.0
    gz = read_data(gyroout_z)/131.0
    
    print(f"accel x: {ax}, y: {ay}, z: {az}")
    print(f"gyro x: {gx}, y: {gy}, z: {gz}")
    
    print("==========================================================================")
    return ax, ay, az, gx, gy, gz

while True:
    read_mpu6500()
    time.sleep(0.25)
