#Import necessary libraries
#RPi.GPIO necessary to use GPIO pins on RPi
import RPi.GPIO as GPIO
#time library for delays
import time
#smbus necessary to handle I2C communication
import smbus
#subprocess necessary to interact with command line
import subprocess
"""
os necessary to handle interactions with the Pi's operating system.
Additionally, the os library will allow us to make directories, change
directories, handle files, etc.
"""
import os
#datetime necessary to put file timestamps in human-readable format
#Can't use datetime.now() with import datetime. Platform-specific?
from datetime import datetime
#re necessary to get search for text and use it
import re
#BMP180 library
import Adafruit_BMP.BMP085 as BMP085
#ctypes for LSM9DS1 library
from ctypes import *
#csv necessary to create csv files
import csv


#SETUP~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#Pin definitions
ledPin = 11
sdaPin = 2
sclPin = 3
piezoPin = 12

#I2C Port Setup (for communication with Lora boards)
i2c = smbus.SMBus(1)

#I2C Addresses
#Barometer
bmp = BMP085.BMP085()

#IMU
#This code snippet from Akimach on Github:
path = "../lib/liblsm9ds1cwrapper.so"
lib = cdll.LoadLibrary(path)

lib.lsm9ds1_create.argtypes = []
lib.lsm9ds1_create.restype = c_void_p

lib.lsm9ds1_begin.argtypes = [c_void_p]
lib.lsm9ds1_begin.restype = None

lib.lsm9ds1_calibrate.argtypes = [c_void_p]
lib.lsm9ds1_calibrate.restype = None

lib.lsm9ds1_gyroAvailable.argtypes = [c_void_p]
lib.lsm9ds1_gyroAvailable.restype = c_int
lib.lsm9ds1_accelAvailable.argtypes = [c_void_p]
lib.lsm9ds1_accelAvailable.restype = c_int
lib.lsm9ds1_magAvailable.argtypes = [c_void_p]
lib.lsm9ds1_magAvailable.restype = c_int

lib.lsm9ds1_readGyro.argtypes = [c_void_p]
lib.lsm9ds1_readGyro.restype = c_int
lib.lsm9ds1_readAccel.argtypes = [c_void_p]
lib.lsm9ds1_readAccel.restype = c_int
lib.lsm9ds1_readMag.argtypes = [c_void_p]
lib.lsm9ds1_readMag.restype = c_int

lib.lsm9ds1_getGyroX.argtypes = [c_void_p]
lib.lsm9ds1_getGyroX.restype = c_float
lib.lsm9ds1_getGyroY.argtypes = [c_void_p]
lib.lsm9ds1_getGyroY.restype = c_float
lib.lsm9ds1_getGyroZ.argtypes = [c_void_p]
lib.lsm9ds1_getGyroZ.restype = c_float

lib.lsm9ds1_getAccelX.argtypes = [c_void_p]
lib.lsm9ds1_getAccelX.restype = c_float
lib.lsm9ds1_getAccelY.argtypes = [c_void_p]
lib.lsm9ds1_getAccelY.restype = c_float
lib.lsm9ds1_getAccelZ.argtypes = [c_void_p]
lib.lsm9ds1_getAccelZ.restype = c_float

lib.lsm9ds1_getMagX.argtypes = [c_void_p]
lib.lsm9ds1_getMagX.restype = c_float
lib.lsm9ds1_getMagY.argtypes = [c_void_p]
lib.lsm9ds1_getMagY.restype = c_float
lib.lsm9ds1_getMagZ.argtypes = [c_void_p]
lib.lsm9ds1_getMagZ.restype = c_float

lib.lsm9ds1_calcGyro.argtypes = [c_void_p, c_float]
lib.lsm9ds1_calcGyro.restype = c_float
lib.lsm9ds1_calcAccel.argtypes = [c_void_p, c_float]
lib.lsm9ds1_calcAccel.restype = c_float
lib.lsm9ds1_calcMag.argtypes = [c_void_p, c_float]
lib.lsm9ds1_calcMag.restype = c_float

if __name__ == "__main__":
    imu = lib.lsm9ds1_create()
    lib.lsm9ds1_begin(imu)
    if lib.lsm9ds1_begin(imu) == 0:
        print("Failed to communicate with LSM9DS1.")
        quit()
    lib.lsm9ds1_calibrate(imu)
    
    #Tell console I2C device list is about to be printed
    print("I2C Device List:")
    #Open terminal, run i2cdetect
    geti2c = subprocess.Popen(['i2cdetect', '-y', '1'], stdout=subprocess.PIPE)
    #For every match for number, number: anything, number, number found, print it
    """
    Add feature that lets RPi check that Lora, baro, and IMU are at certain
    addresses
    """
    for i in range (0,9):
        line = str(geti2c.stdout.readline())
        
        for match in re.finditer("[0-9][0-9]:.*[0-9][0-9]", line):
            #match.group() is all i2c devices from i2cdetect
            i2cDev = match.group()
            #Parse i2cDev so that we get rid of dashes and spaces
            i2cDevParsed = i2cDev.replace("-", "").replace(" ", "") #Bad syntax
            #Print out the result
            print (i2cDevParsed)
            
            
    #Set up LoraWAN board through I2C
    loraAddress = 0x04
    
    def sendByte(x):
        i2c.write_byte(loraAddress, x)
        return -1
    
    def recByte():
        inByte = i2c.read_byte(loraAddress)
        return inByte
    
    #sendByte(0)
    #print ('Sent Lora board "0"')
    
    #loraOpenByte = recByte()
    #print ('Received ', loraOpenByte, ' from Lora board.\n')
    #
    #if loraOpenByte == 1:
    #    print ('Connection with Lora board established.')
    #else:
    #    print ('No connection to Lora board.')
    
    """
    Set RPi GPIO mode - in this case, we are referring to the RPi's GPIO pins
    according to the numbering printed on the motherboard. This is because the
    datalogger doesn't use a Pi Wedge or similar device, so it's just easier
    this way.
    """
    GPIO.setmode(GPIO.BOARD)
    
    print("GPIO mode set to BOARD") #Print GPIO mode to command line
    
    GPIO.setwarnings(False)
    print("No GPIO warnings will be printed")
    
    GPIO.setup(ledPin, GPIO.OUT) #LED pin set to output
    GPIO.setup(piezoPin, GPIO.OUT) #Piezo pin set to output
    
    #Datalogging & Telemetry~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    GPIO.cleanup()
    
    
    asl = bmp.read_altitude()
    #add AGL code here
    
    
    landed = 0 #Rocket landed
    flightComplete = 0 #Flight complete, datalogging can stop
    
    
    calibration = open('calibration' + datetime.now().strftime('%Y-%m-%d-%H-%M'), 'w')
    calibration.write(str(asl) + "\n")
    calibration.close()
    
    #Set up CSV
    timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M')
    filetime = timestamp.replace("'", "")
    dataCSV = "flightdata " + filetime + ".csv" #Set up CSV name for datalogging
    
    titleRow = ['Time', 'Xa', 'Ya', 'Za', 'Xo', 'Yo', 'Zo', 'Mx', 'My', 'Mz', 'Pressure', 'ASL', 
                'AGL', 'Temp']
    
    with open(dataCSV, 'w') as dataCSV:
        writer = csv.writer(dataCSV)
        writer.writerow(titleRow)
    
        while (flightComplete == 0):
            
            while lib.lsm9ds1_gyroAvailable(imu) == 0:
                pass
            lib.lsm9ds1_readGyro(imu)
            while lib.lsm9ds1_accelAvailable(imu) == 0:
                pass
            lib.lsm9ds1_readAccel(imu)
            while lib.lsm9ds1_magAvailable(imu) == 0:
                pass
            lib.lsm9ds1_readMag(imu)
    
            gx = lib.lsm9ds1_getGyroX(imu)
            gy = lib.lsm9ds1_getGyroY(imu)
            gz = lib.lsm9ds1_getGyroZ(imu)
    
            ax = lib.lsm9ds1_getAccelX(imu)
            ay = lib.lsm9ds1_getAccelY(imu)
            az = lib.lsm9ds1_getAccelZ(imu)
    
            mx = lib.lsm9ds1_getMagX(imu)
            my = lib.lsm9ds1_getMagY(imu)
            mz = lib.lsm9ds1_getMagZ(imu)
    
            cgx = lib.lsm9ds1_calcGyro(imu, gx)
            cgy = lib.lsm9ds1_calcGyro(imu, gy)
            cgz = lib.lsm9ds1_calcGyro(imu, gz)
    
            cax = lib.lsm9ds1_calcAccel(imu, ax)
            cay = lib.lsm9ds1_calcAccel(imu, ay)
            caz = lib.lsm9ds1_calcAccel(imu, az)
    
            cmx = lib.lsm9ds1_calcMag(imu, mx)
            cmy = lib.lsm9ds1_calcMag(imu, my)
            cmz = lib.lsm9ds1_calcMag(imu, mz)
    
            temp = bmp.read_temperature()
            pressure = bmp.read_pressure()
            asl = bmp.read_altitude()
            
            ltime = datetime.now().strftime('%H:%M:%S')
            dataRow = [ltime, cax, cay, caz, cgx, cgy, cgz, cmx, cmy, cmz, pressure, asl,' ', temp ]
            writer.writerow(dataRow)
            
            time.sleep(.1)
            
            print('logging')
    
    
    
    
    
    
    
    
    
    
    
