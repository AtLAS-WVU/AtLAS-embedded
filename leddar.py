from __future__ import division
import serial
import minimalmodbus
import time

def Leddar():
    #Set baud rate (device runs @ 115200)
    minimalmodbus.BAUDRATE=115200
    #Designate what port Leddar is plugged into
    leddar = minimalmodbus.Instrument("/dev/tty.usbserial-A800BWGK", 1, 'rtu')
    #Enable to view debugging messages for sensor
    #leddar.debug = True
    leddar.BAUDRATE=115200

    #Gather temperature info
    temperature = leddar.read_register(22, 0, 4, True)
    #print("temp: %f" % (temperature / 256))

    #Sleep, otherwise results in crashing
    time.sleep(1) #Sleep for 1 sec

    #Get # of Detections
    num_detections = leddar.read_register(23, 0, 4)
    print("num_detections: ",num_detections)

    #Get inital time stored in sensor
    timeLSB = leddar.read_register(20, 0, 4)
    timeMSB = leddar.read_register(21, 0, 4)
    startTime = (timeMSB << 16) + timeLSB

    # Clear any existing info before starting
    distance = leddar.read_register(24, 0, 4)

    while True:
        try:
            timeLSB = leddar.read_register(20, 0, 4)
            timeMSB = leddar.read_register(21, 0, 4)
            currentTime = (timeMSB << 16) + timeLSB

            distance = leddar.read_register(24, 0, 4)

            print("Proximity: %f: LEDDAR: %f" % ((currentTime - startTime) / 1000, distance / 1000))
            time.sleep(1)
        except OSError:
            print("No communication with sensor")

Leddar()
