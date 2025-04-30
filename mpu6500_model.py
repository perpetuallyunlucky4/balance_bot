import smbus2
import time

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

alpha = 0.8
prev_ax, prev_ay, prev_az = 0, 0, 0
prev_gx, prev_gy, prev_gz = 0, 0, 0

bus = smbus2.SMBus(1) #initializes the i2c bus

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
    
    if value > 32767:
        value = value - 65536
    
    return value

def read_mpu6500():
    
    global prev_gx, prev_gy, prev_gz, prev_ax, prev_ay, prev_az, alpha
    
    ax = (alpha * prev_ax + (1 - alpha) * read_data(accelout_x)/16384.0)
    ay = (alpha * prev_ay + (1 - alpha) * read_data(accelout_y)/16384.0)
    az = (alpha * prev_az + (1 - alpha) * read_data(accelout_z)/16384.0)
    
    gx = (alpha * prev_gx + (1 - alpha) * read_data(gyroout_x)/131.0)
    gy = (alpha * prev_gy + (1 - alpha) * read_data(gyroout_y)/131.0) 
    gz = (alpha * prev_gz + (1 - alpha) * read_data(gyroout_z)/131.0) 
    
    prev_ax, prev_ay, prev_az = ax, ay, az
    prev_gx, prev_gy, prev_gz = gx, gy, gz
    
    print(f"accel x: {ax}, y: {ay}, z: {az}")
    print(f"gyro x: {gx}, y: {gy}, z: {gz}")
    print("==========================================================================")
    
    return ax, ay, az, gx, gy, gz

def calibrate_mpu6500(duration=5):
    print("Calibrating MPU6500... Keep the device STILL and FLAT.")
    time.sleep(2)

    ax_offset, ay_offset, az_offset = 0, 0, 0
    gx_offset, gy_offset, gz_offset = 0, 0, 0
    sample_count = 0

    start_time = time.time()

    while time.time() - start_time < duration:
        ax, ay, az, gx, gy, gz = read_mpu6500()
        ax_offset += ax
        ay_offset += ay
        az_offset += az
        gx_offset += gx
        gy_offset += gy
        gz_offset += gz
        sample_count += 1
        time.sleep(0.01)  # ~100 Hz sampling

    # Average the collected values
    ax_offset /= sample_count
    ay_offset /= sample_count
    az_offset /= sample_count
    gx_offset /= sample_count
    gy_offset /= sample_count
    gz_offset /= sample_count

    # Normally az should be close to 1g (gravity)
    az_offset -= 1.0

    print("\nCalibration complete.")
    print(f"Accelerometer offsets: ax={ax_offset:.4f}, ay={ay_offset:.4f}, az={az_offset:.4f}")
    print(f"Gyroscope offsets: gx={gx_offset:.4f}, gy={gy_offset:.4f}, gz={gz_offset:.4f}")

    return (ax_offset, ay_offset, az_offset), (gx_offset, gy_offset, gz_offset)

