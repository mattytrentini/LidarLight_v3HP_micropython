''' 
Author: Dan Napert
Date: Jan 2024
Reference: https://github.com/garmin/LIDARLite_Arduino_Library/blob/master/src/LIDARLite_v3HP.cpp
Datasheet: https://static.garmin.com/pumac/LIDAR-Lite_v3HP_Instructions_EN.pdf

Remember to add a 680uf capacitor across the power and ground pins of the sensor.
If using multiple I2C devices, remember to use pull-up resistors on the SDA and SCL lines.

Wiring:

Blue wire: SDA
Green wire: SCL
Black wire: GND
Red wire: 5V
10k pull-up resistors on SDA and SCL lines

'''



from machine import Pin,I2C
from time import sleep_us


class V3HP:
    def __init__(self,mode=0, i2c=I2C(1,scl=Pin(7),sda=Pin(6),freq=400000),address = 0x62):
        
        self.i2c = i2c
        sleep_us(80) # sleep for 80 microseconds to allow i2c to settle
        
        self.address = address
        self.sig_count_max = 0x80
        self.acq_config_reg = 0x08
        self.ref_count_max = 0x05
        self.threshold_bypass = 0x00
        self.busy_counter = 0
        self.configurations = {
            0: { # Default mode, balanced performance
                'sig_count_max': 0x80,
                'acq_config_reg': 0x08,
                'ref_count_max': 0x05,
                'threshold_bypass': 0x00,
            },
            1: { # Short range, high speed
                'sig_count_max': 0x1d,
                'acq_config_reg': 0x08,
                'ref_count_max': 0x03,
                'threshold_bypass': 0x03,
            },
            2: {    # Default range, higher speed short range
                'sig_count_max': 0x80,
                'acq_config_reg': 0x00,
                'ref_count_max': 0x03,
                'threshold_bypass': 0x00,
            },
            3: {    # Maximum range
                'sig_count_max': 0xff,
                'acq_config_reg': 0x08,
                'ref_count_max': 0x05,
                'threshold_bypass': 0x00,
            },  
            4: {    # High sensitivity detection, high erroneous measurements
                'sig_count_max': 0x80,
                'acq_config_reg': 0x08,
                'ref_count_max': 0x05,
                'threshold_bypass': 0x00,
            },
            5: {    # Low sensitivity detection, low erroneous measurements
                'sig_count_max': 0x80,
                'acq_config_reg': 0x08,
                'ref_count_max': 0x05,
                'threshold_bypass': 0xb0,
            },
            6: {    # Short range, high speed, higher error, not recommended
                'sig_count_max': 0x04,
                'acq_config_reg': 0x01,
                'ref_count_max': 0x03,
                'threshold_bypass': 0x00,
            },
        }
        self.configure(mode)
        
    def configure(self,mode)->None:
        '''
          configuration:  Default 0.
            0: Default mode, balanced performance.
            1: Short range, high speed. Uses 0x1d maximum acquisition count.
            2: Default range, higher speed short range. Turns on quick termination
                detection for faster measurements at short range (with decreased
                accuracy)
            3: Maximum range. Uses 0xff maximum acquisition count.
            4: High sensitivity detection. Overrides default valid measurement detection
                algorithm, and uses a threshold value for high sensitivity and noise.
            5: Low sensitivity detection. Overrides default valid measurement detection
                algorithm, and uses a threshold value for low sensitivity and noise.
            6: Short range, high speed, higher error. Overrides default valid measurement
        '''
          
        self.sig_count_max = self.configurations[mode]['sig_count_max']
        self.acq_config_reg = self.configurations[mode]['acq_config_reg']
        self.ref_count_max = self.configurations[mode]['ref_count_max']
        self.threshold_bypass = self.configurations[mode]['threshold_bypass']
        
        self.write(0x02,bytes([self.sig_count_max]))
        self.write(0x04,bytes([self.acq_config_reg]))
        self.write(0x12,bytes([self.ref_count_max]))
        self.write(0x1c,bytes([self.threshold_bypass]))
        
    def change_i2c_address(self,address,disable_default=False)->None:
        '''
        address: desired secondary I2C device address
        
        disable_default: a  True value here means the default 0x62 I2C device
        address will be disabled.  A False value here means the default 0x62
        address will remain enabled.  The default is False.
        Do this with caution.  The new address will be saved in EEPROM and will
        continue to be used after reset/power cycle.
        
        Default address is 0x62 .
        '''
        #  Read UNIT_ID serial number bytes and write them into I2C_ID byte locations
        databytes = bytearray(self.read(0x16,2))
        self.write(0x18,databytes)
        # Write the new I2C device address to registers
        # left shift by one to work around data alignment issue in v3HP
        databytes[0] = address << 1
        self.write(0x1a,databytes)
        # Enable the new I2C device address
        self.address = address
        databytes = bytearray(self.read(0x1e,1))
        databytes[0] = databytes[0] | 1 << 4
        self.write(0x1e,databytes)
        
        if disable_default:
            databytes = bytearray(self.read(0x1e,1))
            databytes[0] = databytes[0] | 1 << 3
            self.write(0x1e,databytes)
        
    def read_distance(self)->int:
        '''
        Read and return result of distance measurement in centimeters.
        
        1.  Read two bytes from register 0x8f 
        2.  Shift the first value from 0x8f << 8 and add to second value from 0x8f.
        The result is the measured distance in centimeters.
        '''
        databytes = self.read(0x0f,2)
        # Create 16-bit word from bytes
        return databytes[0] << 8 | databytes[1]
    
    def take_range(self)->None:
        '''
        Initiate a distance measurement by writing to register 0x00.
        '''
        self.write(0x00,bytes([0x01]))
    
    def wait_for_busy(self)->None:
        
        busy_counter = 0
        while self.get_busy_flag() == 1:
            busy_counter += 1
            if busy_counter > 9999:
                print("giving up on busy flag")
                break
    
    def get_busy_flag(self)->int:
        '''
        Read BUSY flag from device registers. Function will return 0 if not busy.
        '''
        busy_flag = self.read(0x01,1)
        return busy_flag[0] & 0x01
    
    def write(self,register,value):
        '''
        wrapper for i2c.writeto_mem
        '''
        self.i2c.writeto_mem(self.address,register,value)
        # sleep 10 microseconds for safety with subsequent i2c reads
        sleep_us(10)
        
    def read(self,register,bytes)->bytes:
        '''
        wrapper for i2c.readfrom_mem
        '''
        val = self.i2c.readfrom_mem(self.address,register,bytes)
        
        return val
    
    def reset_reference_filter(self)->None:
        '''
        Quickly resets the reference filter to improve the accuracy of initial measurements. Not normally necessary, usually will rectify itself after a few hundred measurements.
        '''
        
        
        databytes = bytearray(self.read(0x04,1))            # read the current value of the acquisition configuration register
        acq_config_reg = databytes[0]                       # store the current value of the acquisition configuration register
        databytes[0] = databytes[0] | 0x10                  # disable reference filter
        self.write(0x12,databytes)                          # write the new value to the acquisition configuration register   
        databytes = bytearray(self.read(0x12,1))            # read ref integration count
        refCountMax = databytes[0]                          # store the current value of the ref integration count
        databytes[0] = 0xff                                 # reference to overflow faster
        self.write(0x12,databytes)                          # write the new value to the ref integration count
        
        # Take a range measurement
        self.range()
        
        databytes[0] = refCountMax                           # restore the original value of the ref integration count
        self.write(0x12,databytes)                           # write the new value to the ref integration count
        databytes[0] = acq_config_reg                        # restore the original value of the acquisition configuration register
        self.write(0x04,databytes)                           # write the new value to the acquisition configuration register
        
    def range(self)->int:
        '''
        takes a single range measurement and returns the result in centimeters, most accurate.
        stable around 280 measurements per second on pi pico
        '''
        self.wait_for_busy()
        self.take_range()
        self.wait_for_busy()
        return self.read_distance()
    
    def range_fast(self)->int:
        '''
        takes a single range measurement and returns the result in centimeters,
        but does not check to see if the measurement is complete.  The result may be
        stale if the device is busy, less accurate but faster.
        stable around 330 measurements per second on pi pico.
        '''
        self.wait_for_busy()
        self.take_range()
        return self.read_distance()

