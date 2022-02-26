import time
import smbus
import RPi.GPIO as GPIO
import numpy as np
import threading
# pwm naming
PWM1 = 18
PWM2 = 13
PWM3 = 12
PWM4 = 19
i2c_ch = 1
# address of the sensor
i2c_addr = 0x68
i2c_mag_addr = 0x0C
# initialization regs
XG_OFFSET_H_addr = 0x13
XG_OFFSET_L_addr = 0x14
YG_OFFSET_H_addr = 0x15
YG_OFFSET_L_addr = 0x16
ZG_OFFSET_H_addr = 0x17
ZG_OFFSET_L_addr = 0x18
SMPLRT_DIV_addr = 0x19
CONFIG_addr = 0x1A
GYRO_CONFIG_addr = 0x1B
ACCEL_CONFIG_addr = 0x1C
ACCEL_CONFIG_2_addr = 0x1D
LP_ACCEL_ODR_addr = 0x1E
WOM_THR_addr = 0x1F
FIFO_EN_addr = 0x23
INT_PIN_CFG_addr = 0x37

# real regs
## accel related addr
ACCEL_XOUT_H_addr = 0x3B
ACCEL_XOUT_L_addr = 0x3C
ACCEL_YOUT_H_addr = 0x3D
ACCEL_YOUT_L_addr = 0x3E
ACCEL_ZOUT_H_addr = 0x3F
ACCEL_ZOUT_L_addr = 0x40
XA_OFFSET_H_addr = 0x77
XA_OFFSET_L_addr = 0x78 
YA_OFFSET_H_addr = 0x7A
YA_OFFSET_L_addr = 0x7B
ZA_OFFSET_H_addr = 0x7D
ZA_OFFSET_L_addr = 0x7E

TEMP_OUT_H_addr = 0x41
TEMP_OUT_L_addr = 0x42
GYRO_XOUT_H_addr = 0x43
GYRO_XOUT_L_addr = 0x44
GYRO_YOUT_H_addr = 0x45
GYRO_YOUT_L_addr = 0x46
GYRO_ZOUT_H_addr = 0x47
GYRO_ZOUT_L_addr = 0x48
# 
CTRL1_addr = 0x0A
CTRL2_addr = 0x0B
HX_L_addr = 0x03
HX_H_addr = 0x04
HY_L_addr = 0x05
HY_H_addr = 0x06
HZ_L_addr = 0x07
HZ_H_addr = 0x08

# initialise I2C
bus = smbus.SMBus(i2c_ch)
bus.write_i2c_block_data(i2c_addr,INT_PIN_CFG_addr,[0b00000010])
bus.write_i2c_block_data(i2c_addr,XG_OFFSET_H_addr,[0b00000000])
bus.write_i2c_block_data(i2c_addr,XG_OFFSET_L_addr,[0b00000000])
bus.write_i2c_block_data(i2c_addr,YG_OFFSET_H_addr,[0b00000000])
bus.write_i2c_block_data(i2c_addr,YG_OFFSET_L_addr,[0b00000000])
bus.write_i2c_block_data(i2c_addr,ZG_OFFSET_H_addr,[0b00000000])
bus.write_i2c_block_data(i2c_addr,ZG_OFFSET_L_addr,[0b00000000])
bus.write_i2c_block_data(i2c_addr,SMPLRT_DIV_addr ,[0b00000000])
bus.write_i2c_block_data(i2c_addr,GYRO_CONFIG_addr,[0b00000000])
# accel config 
bus.write_i2c_block_data(i2c_addr,XA_OFFSET_H_addr ,[0b00101000])
bus.write_i2c_block_data(i2c_addr,XA_OFFSET_L_addr ,[0b00000000])
bus.write_i2c_block_data(i2c_addr,YA_OFFSET_H_addr ,[0b11110001])
bus.write_i2c_block_data(i2c_addr,YA_OFFSET_L_addr ,[0b00000000])
bus.write_i2c_block_data(i2c_addr,ZA_OFFSET_H_addr ,[0b00101010])

bus.write_i2c_block_data(i2c_addr,ZA_OFFSET_L_addr ,[0000000000])
bus.write_i2c_block_data(i2c_addr,ACCEL_CONFIG_addr,[0b00001000])
# mag config
bus.write_i2c_block_data(i2c_mag_addr,CTRL1_addr,  [0b00011000])

#bus.write_i2c_block_data(i2c_mag_addr,CTRL2_addr,[0b00000000])
# initialise
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM1,GPIO.OUT)
GPIO.setup(PWM2,GPIO.OUT)
GPIO.setup(PWM3,GPIO.OUT)
GPIO.setup(PWM4,GPIO.OUT)
LF = GPIO.PWM(PWM1,1000)
RF = GPIO.PWM(PWM2,1000)
LB = GPIO.PWM(PWM3,1000)
RB = GPIO.PWM(PWM4,1000)



def drive(L_duty_cycle, R_duty_cycle):
    if L_duty_cycle>=0:
        LB.stop()
        LF.start(L_duty_cycle)
    else:
        LF.stop()
        LB.start(-L_duty_cycle)
    if R_duty_cycle>=0:
        RB.stop()
        RF.start(R_duty_cycle)
    else:
        RF.stop()
        RB.start(-R_duty_cycle)

def xyz_direction():
    GYRO_XOUT =  bus.read_i2c_block_data(i2c_addr,GYRO_XOUT_H_addr,2)
    GYRO_YOUT =  bus.read_i2c_block_data(i2c_addr,GYRO_YOUT_H_addr,2)
    GYRO_ZOUT =  bus.read_i2c_block_data(i2c_addr,GYRO_ZOUT_H_addr,2)
    
    ACCEL_XOUT =  bus.read_i2c_block_data(i2c_addr,ACCEL_XOUT_H_addr,2)
    ACCEL_YOUT =  bus.read_i2c_block_data(i2c_addr,ACCEL_YOUT_H_addr,2)
    ACCEL_ZOUT =  bus.read_i2c_block_data(i2c_addr,ACCEL_ZOUT_H_addr,2)
    
    ACCEL_XOUT_tmp = np.int16((ACCEL_XOUT[0]<<8 | ACCEL_XOUT[1]))
    ACCEL_YOUT_tmp = np.int16((ACCEL_YOUT[0]<<8 | ACCEL_YOUT[1]))
    ACCEL_ZOUT_tmp = np.int16((ACCEL_ZOUT[0]<<8 | ACCEL_ZOUT[1]))

    HX_L_OUT = bus.read_i2c_block_data(i2c_mag_addr,HX_L_addr,1)
    HX_H_OUT = bus.read_i2c_block_data(i2c_mag_addr,HX_H_addr,1)
    HY_L_OUT = bus.read_i2c_block_data(i2c_mag_addr,HY_L_addr,1)
    HY_H_OUT = bus.read_i2c_block_data(i2c_mag_addr,HY_H_addr,1)
    HZ_L_OUT = bus.read_i2c_block_data(i2c_mag_addr,HZ_L_addr,1)
    HZ_H_OUT = bus.read_i2c_block_data(i2c_mag_addr,HZ_H_addr,1)
    return ACCEL_XOUT_tmp, ACCEL_YOUT_tmp, ACCEL_ZOUT_tmp
    #
    #return GYRO_XOUT, GYRO_YOUT, GYRO_ZOUT, ACCEL_XOUT, ACCEL_YOUT, ACCEL_ZOUT
    #return GYRO_XOUT, GYRO_YOUT, GYRO_ZOUT, HX_L_OUT, HX_H_OUT, HY_L_OUT, HY_H_OUT, HZ_L_OUT,HZ_H_OUT

while(1):
    bus.write_i2c_block_data(i2c_mag_addr,CTRL1_addr,[0b00000001])
    time.sleep(0.01)
    x,y,z= xyz_direction()
    print(x,y,z)
    #print(bus.read_i2c_block_data(i2c_addr,59,6))
    #drive(100,100)
    time.sleep(1)
