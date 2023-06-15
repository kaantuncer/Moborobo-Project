#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 26 23:01:43 2022

@author: ozgur
"""
"""Driver for locosys GPS for moborobo"""

#import numpy as np
#import os, pynmea2, roslib
import rospy, serial
from nmea_msgs.msg import Sentence


def main(robotname="moborobot", portname = '/dev/ttyUSB0',baudrate=57600):
    myser=serial.Serial(portname, baudrate)
    myser.write(b'$PMTK220,200*2C\r\n')#set it to 200Hz!
    #myser.write(b'$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n')#this should turn Off all sentences except GGA and RMC
    myser.write(b'$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n')#this should turn Off all sentences except GGA
    '''
    $GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,-164.0,M,,,,*47
    GGA 	Global Positioning System Fix Data
    123519.00 	Fix taken at 12:35:19 UTC
    4807.038,N 	Latitude 48 deg 07.038' N
    01131.000, E 	Longitude 11 deg 31.000' E
    1 	Fix quality:
    0 = Invalid
    1 = GNSS fix (SPS)
    2 = DGPS fix
    3 = PPS fix
    4 = Real Time Kinematic
    6 = estimated (dead reckoning) (2.3 feature)
    7 = Manual input mode
    8 = Simulation mode
    08 	Number of satellites being tracked
    0.9 	Horizontal dilution of position
    545.4,M 	Altitude, Meters, above mean sea level (geoid)
    -164.0,M 	Height of geoid (mean sea level) above WGS84 ellipsoid
    (empty field) 	(Field not provided in this setup)
    *47 	Checksum data, always begins with *
    '''
    #mysentence = pynmea2.parse(mymes[0:-2].decode("utf-8"))
    gps_pub = rospy.Publisher("/gps",
               Sentence)
    msg = Sentence()
    rospy.sleep(0.1)
    
    while not rospy.is_shutdown():
    #for i in range(100):
        #'''
        try:
          mymes = myser.readline()
        except myser.SerialException as e:
          print('Device error: {}'.format(e))
          break
        #except pynmea2.ParseError as e:
        #  print('Parse error: {}'.format(e))
        #  continue
        #'''
        mymes = mymes[0:-2].decode("utf-8")
        msg.header.stamp = rospy.Time.now()        
        msg.sentence = mymes
        # Publish new msg
        gps_pub.publish(msg)
    
    myser.close()

if __name__ == "__main__":
    rospy.init_node('gps_pub', anonymous=True)
    main()