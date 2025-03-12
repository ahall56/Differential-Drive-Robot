"""
BNO055 Driver for Romi Robot
Driver to interface with the BNO055 IMU using I2C
"""

import pyb
import time
import os
import math

class IMU:
    # Register addresses
    REG_CHIP_ID = 0x00
    REG_PAGE_ID = 0x07
    REG_OPR_MODE = 0x3D
    REG_UNIT_SEL = 0x3B
    REG_CALIB_STAT = 0x35
    REG_TEMP_SOURCE = 0x40
    REG_SYS_TRIGGER = 0x3F
    
    # Euler angle registers
    REG_EUL_HEADING_LSB = 0x1A
    REG_EUL_HEADING_MSB = 0x1B
    REG_EUL_ROLL_LSB = 0x1C
    REG_EUL_ROLL_MSB = 0x1D
    REG_EUL_PITCH_LSB = 0x1E
    REG_EUL_PITCH_MSB = 0x1F
    
    # Angular velocity registers
    REG_GYR_DATA_X_LSB = 0x14
    REG_GYR_DATA_X_MSB = 0x15
    REG_GYR_DATA_Y_LSB = 0x16
    REG_GYR_DATA_Y_MSB = 0x17
    REG_GYR_DATA_Z_LSB = 0x18
    REG_GYR_DATA_Z_MSB = 0x19
    
    # Calibration data registers
    REG_ACC_OFFSET_X_LSB = 0x55
    REG_ACC_OFFSET_X_MSB = 0x56
    REG_ACC_OFFSET_Y_LSB = 0x57
    REG_ACC_OFFSET_Y_MSB = 0x58
    REG_ACC_OFFSET_Z_LSB = 0x59
    REG_ACC_OFFSET_Z_MSB = 0x5A
    
    REG_MAG_OFFSET_X_LSB = 0x5B
    REG_MAG_OFFSET_X_MSB = 0x5C
    REG_MAG_OFFSET_Y_LSB = 0x5D
    REG_MAG_OFFSET_Y_MSB = 0x5E
    REG_MAG_OFFSET_Z_LSB = 0x5F
    REG_MAG_OFFSET_Z_MSB = 0x60
    
    REG_GYR_OFFSET_X_LSB = 0x61
    REG_GYR_OFFSET_X_MSB = 0x62
    REG_GYR_OFFSET_Y_LSB = 0x63
    REG_GYR_OFFSET_Y_MSB = 0x64
    REG_GYR_OFFSET_Z_LSB = 0x65
    REG_GYR_OFFSET_Z_MSB = 0x66
    
    REG_ACC_RADIUS_LSB = 0x67
    REG_ACC_RADIUS_MSB = 0x68
    REG_MAG_RADIUS_LSB = 0x69
    REG_MAG_RADIUS_MSB = 0x6A
    
    # Operating modes
    OPERATION_MODE_CONFIG = 0x00
    OPERATION_MODE_IMU = 0x08  # Relative orientation
    OPERATION_MODE_COMPASS = 0x09  # Heading only
    OPERATION_MODE_M4G = 0x0A  # Similar to IMU but uses magnetometer
    OPERATION_MODE_NDOF_FMC_OFF = 0x0B  # Absolute orientation
    OPERATION_MODE_NDOF = 0x0C  # Absolute orientation with fast calibration
    
    # Expected chip ID
    CHIP_ID = 0xA0
    
    # I2C address
    I2C_ADDR = 0x28
    
    def __init__(self, i2c):
        """Initialize the BNO055 sensor.
        
        Args:
            i2c: A pyb.I2C object initialized in controller mode
        """
        self.i2c = i2c
        self.addr = self.I2C_ADDR
        
        # Verify the device is present
        if self.read_register(self.REG_CHIP_ID) != self.CHIP_ID:
            raise RuntimeError("BNO055 not detected at I2C address 0x28")
            
        # Reset the device
        self._reset()
        
        # Set to configuration mode
        self.set_operation_mode(self.OPERATION_MODE_CONFIG)
        
        # Set units to degrees and dps
        self.write_register(self.REG_UNIT_SEL, 0x00)
        
        # Set temperature source to gyroscope
        self.write_register(self.REG_TEMP_SOURCE, 0x01)
        
        # By default, use NDOF mode for full 9-axis fusion
        self.set_operation_mode(self.OPERATION_MODE_NDOF)
    
    def _reset(self):
        """Reset the BNO055 sensor"""
        # Set the RST_SYS bit (bit 5) in SYS_TRIGGER register
        self.write_register(self.REG_SYS_TRIGGER, 0x20)
        # Wait for the device to reset
        time.sleep(0.7)  # 650ms as per datasheet
        
    def read_register(self, reg_addr):
        """Read a single byte from the specified register.
        
        Args:
            reg_addr: Register address to read from
            
        Returns:
            Value read from the register
        """
        return self.i2c.mem_read(1, self.addr, reg_addr)[0]
    
    def write_register(self, reg_addr, data):
        """Write a single byte to the specified register.
        
        Args:
            reg_addr: Register address to write to
            data: Byte to write
        """
        self.i2c.mem_write(bytes([data]), self.addr, reg_addr)
    
    def read_registers(self, reg_addr, length):
        """Read multiple bytes starting from the specified register.
        
        Args:
            reg_addr: Starting register address
            length: Number of bytes to read
            
        Returns:
            Bytes read from the registers
        """
        return self.i2c.mem_read(length, self.addr, reg_addr)
    
    def write_registers(self, reg_addr, data):
        """Write multiple bytes starting at the specified register.
        
        Args:
            reg_addr: Starting register address
            data: Bytes to write
        """
        self.i2c.mem_write(data, self.addr, reg_addr)
    
    def set_operation_mode(self, mode):
        """Change the operating mode of the BNO055.
        
        Args:
            mode: One of the OPERATION_MODE_* constants
        """
        current_mode = self.read_register(self.REG_OPR_MODE) & 0x0F
        
        # If already in the requested mode, return
        if current_mode == mode:
            return
            
        # Enter CONFIG mode first if not already
        if current_mode != self.OPERATION_MODE_CONFIG:
            self.write_register(self.REG_OPR_MODE, self.OPERATION_MODE_CONFIG)
            time.sleep(0.025)  # 19ms as per datasheet
        
        # Change to the requested mode if not CONFIG mode
        if mode != self.OPERATION_MODE_CONFIG:
            self.write_register(self.REG_OPR_MODE, mode)
            time.sleep(0.010)  # 7ms as per datasheet
    
    def get_calibration_status(self):
        """Retrieve and parse the calibration status byte.
        
        Returns:
            Dictionary containing calibration status for each system:
            {'sys': 0-3, 'gyro': 0-3, 'accel': 0-3, 'mag': 0-3}
            where 0 = not calibrated, 3 = fully calibrated
        """
        cal_status = self.read_register(self.REG_CALIB_STAT)
        
        return {
            'sys': (cal_status >> 6) & 0x03,
            'gyro': (cal_status >> 4) & 0x03,
            'accel': (cal_status >> 2) & 0x03,
            'mag': cal_status & 0x03
        }
    
    def is_fully_calibrated(self):
        """Check if all sensors are fully calibrated.
        
        Returns:
            True if all sensors report calibration level 3, False otherwise
        """
        status = self.get_calibration_status()
        return all(level == 3 for level in status.values())
    
    def get_calibration_coefficients(self):
        """Retrieve calibration coefficients from the IMU.
        
        Returns:
            Bytes containing calibration coefficients for all sensors
        """
        # Switch to CONFIG mode to access calibration data
        previous_mode = self.read_register(self.REG_OPR_MODE) & 0x0F
        self.set_operation_mode(self.OPERATION_MODE_CONFIG)
        
        # Read all calibration registers at once (22 registers, 44 bytes)
        cal_data = self.read_registers(self.REG_ACC_OFFSET_X_LSB, 22)
        
        # Restore previous mode
        self.set_operation_mode(previous_mode)
        
        return cal_data
    
    def set_calibration_coefficients(self, cal_data):
        """Write calibration coefficients to the IMU.
        
        Args:
            cal_data: Bytes containing calibration coefficients
        """
        if len(cal_data) != 22:
            raise ValueError("Calibration data must be 22 bytes")
            
        # Switch to CONFIG mode to write calibration data
        previous_mode = self.read_register(self.REG_OPR_MODE) & 0x0F
        self.set_operation_mode(self.OPERATION_MODE_CONFIG)
        
        # Write all calibration registers at once
        self.write_registers(self.REG_ACC_OFFSET_X_LSB, cal_data)
        
        # Restore previous mode
        self.set_operation_mode(previous_mode)
    
    def get_euler_angles(self):
        """Read Euler angles from the IMU.
        
        Returns:
            Dictionary containing Euler angles in degrees:
            {'heading': heading, 'roll': roll, 'pitch': pitch}
        """
        # Read 6 bytes (3 16-bit values)
        data = self.read_registers(self.REG_EUL_HEADING_LSB, 6)
        
        # Convert from little-endian 16-bit signed integers to degrees
        heading = (data[1] << 8 | data[0]) / 16.0
        if heading > 180:
            heading -= 360
            
        roll = (data[3] << 8 | data[2]) / 16.0
        if roll > 180:
            roll -= 360
            
        pitch = (data[5] << 8 | data[4]) / 16.0
        if pitch > 180:
            pitch -= 360
            
        return {'heading': heading, 'roll': roll, 'pitch': pitch}
    
    def get_heading(self):
        """Read just the heading Euler angle.
        
        Returns:
            Heading in degrees (0-360)
        """
        data = self.read_registers(self.REG_EUL_HEADING_LSB, 2)
        heading = (data[1] << 8 | data[0]) / 16.0
        if heading < 0:
            heading += 360
            
        return heading
    
    def get_angular_velocity(self):
        """Read angular velocity from the IMU.
        
        Returns:
            Dictionary containing angular velocities in degrees per second:
            {'x': x_rate, 'y': y_rate, 'z': z_rate}
        """
        # Read 6 bytes (3 16-bit values)
        data = self.read_registers(self.REG_GYR_DATA_X_LSB, 6)
        
        # Convert from little-endian 16-bit signed integers to dps
        x_rate = ((data[1] << 8) | data[0])
        if x_rate > 32767:
            x_rate -= 65536
        x_rate = x_rate / 16.0
            
        y_rate = ((data[3] << 8) | data[2])
        if y_rate > 32767:
            y_rate -= 65536
        y_rate = y_rate / 16.0
            
        z_rate = ((data[5] << 8) | data[4])
        if z_rate > 32767:
            z_rate -= 65536
        z_rate = z_rate / 16.0
            
        return {'x': x_rate, 'y': y_rate, 'z': z_rate}
    
    def get_yaw_rate(self):
        """Read just the yaw (z-axis) angular velocity.
        
        Returns:
            Yaw rate in degrees per second
        """
        data = self.read_registers(self.REG_GYR_DATA_Z_LSB, 2)
        z_rate = ((data[1] << 8) | data[0])
        if z_rate > 32767:
            z_rate -= 65536
        return z_rate / 16.0
    
    def save_calibration(self, filename='calibration.txt'):
        """Save calibration coefficients to a file.
        
        Args:
            filename: Name of the file to save calibration data to
        """
        cal_data = self.get_calibration_coefficients()
        with open(filename, 'wb') as f:
            f.write(cal_data)
    
    def load_calibration(self, filename='calibration.txt'):
        """Load calibration coefficients from a file.
        
        Args:
            filename: Name of the file to load calibration data from
            
        Returns:
            True if calibration data was loaded, False otherwise
        """
        try:
            with open(filename, 'rb') as f:
                cal_data = f.read()
                if len(cal_data) == 22:
                    self.set_calibration_coefficients(cal_data)
                    return True
                else:
                    print(f"Invalid calibration data in {filename}")
        except OSError:
            print(f"Could not open {filename}")
        return False
    
    def calibrate_with_file(self, filename='calibration.txt', timeout=30):
        """Calibrate the IMU using a file or manual calibration.
        
        This implements the flowchart from the lab document.
        
        Args:
            filename: Name of the calibration file
            timeout: Timeout in seconds for manual calibration
            
        Returns:
            True if calibration was successful, False otherwise
        """
        # Try to load calibration from file
        if filename in os.listdir('/'):
            print(f"Loading calibration from {filename}")
            if self.load_calibration(filename):
                return True
        
        # Manual calibration needed
        print("Manual calibration required. Please follow these steps:")
        print("1. Place the device in 6 different stable positions for accelerometer")
        print("2. Keep the device still for gyroscope calibration")
        print("3. Move in figure-8 pattern for magnetometer calibration")
        
        # Switch to NDOF mode for calibration
        self.set_operation_mode(self.OPERATION_MODE_NDOF)
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.get_calibration_status()
            print(f"Calibration status: sys={status['sys']}, gyro={status['gyro']}, accel={status['accel']}, mag={status['mag']}")
            
            if self.is_fully_calibrated():
                print("Calibration complete!")
                # Save calibration data
                self.save_calibration(filename)
                return True
                
            time.sleep(1)
        
        print("Calibration timed out")
        return False