#!/usr/bin/env python

from __future__ import division
import serial
import minimalmodbus
import os
import time
import RPi.GPIO as GPIO

def main():
    minimalmodbus.BAUDRATE=115200
    mmb = minimalmodbus.Instrument("/dev/ttyAMA0", 1, 'rtu')
    mmb.BAUDRATE=115200

    temp = mmb.read_register(22, 0, 4, True)
    print "temp: %f" % (temp / 256)

    time.sleep(0.1) # ARBITRARY SLEEP REQUIRED WHY?

    num_detections = mmb.read_register(23, 0, 4)
    print "num_detections: %d" % num_detections

    time_lss = mmb.read_register(20, 0, 4)
    time_mss = mmb.read_register(21, 0, 4)
    start_time = (time_mss << 16) + time_lss

    #----------------------------------------------------------------------------------
    # Create GPIO 18 as input, pull down
    #----------------------------------------------------------------------------------
    GPIO_INT = 18
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_INT, GPIO.IN, GPIO.PUD_UP)

    #----------------------------------------------------------------------------------
    # Clear the already existing interrupt by reading data from the modbus link
    #----------------------------------------------------------------------------------
    dist_leddar = mmb.read_register(24, 0, 4)

    for ii in range(10):

        #------------------------------------------------------------------------------
        # Add GPIO pin 18 rising edge interrupt and then get new data
        #------------------------------------------------------------------------------
        GPIO.wait_for_edge(GPIO_INT, GPIO.RISING)

        time_lss = mmb.read_register(20, 0, 4)
        time_mss = mmb.read_register(21, 0, 4)
        current_time = (time_mss << 16) + time_lss

        dist_leddar = mmb.read_register(24, 0, 4)

        print "Proximity @ %f: LEDDAR %f;" % ((current_time - start_time) / 1000, dist_leddar / 1000)

if __name__ == "__main__":
    os.system("systemctl stop serial-getty@ttyAMA0.service")
    main()
