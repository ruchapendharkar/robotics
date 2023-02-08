#!/usr/bin/python3


import rospy
import serial
from std_msgs.msg import Float64, Header
from gps_rtk.msg import gps_msg
import utm
import sys

rospy.init_node("gps_data")
SENSOR_NAME = "gps"
port = sys.argv[1]
serial_baud = 4800

pub = rospy.Publisher('gps', gps_msg, queue_size=10)

gps_message = gps_msg()
port_data = serial.Serial(port, serial_baud, timeout=3)
	   
counter = 0 
while not rospy.is_shutdown():

    
    line = port_data.readline().decode("utf-8")
    #To ensure that only GNGGA lines are printed
    if "$GNGGA" in line:
        
        
        data = line.split(",")
        #print(data)
        

        #Latitude, Longitude, Altitude and Fix is extracted from the strings
        latitude = (data[2])
        longitude = (data[4])
        altitude = float((data[9]))
        fix = data[6]

        #Converting UTC Time to Seconds 
        header_raw_data = data[1]
        hours = int(header_raw_data[0:2])
        mins = int(header_raw_data[2:4])
        secs = int(header_raw_data[4:6])

        time_secs = (hours * 3600) + (mins * 60) + secs
        time_nsecs = (int(header_raw_data[7:]))*(1000000)
        
        #Converting Latitude and Longitude to Decimals 
        lat_degrees = float(latitude[0:2])
        latitude2 = float(latitude[2:])
        lat_mins = (latitude2 / 60) 
        lat_sum = lat_degrees + lat_mins
        

        long_degrees = float(longitude[0:3])
        longitude2 = float(longitude[3:])
        long_mins = (longitude2 / 60) 
        long_sum = long_degrees + long_mins


        if data[3] == "S":
            lat_final = lat_sum * (-1)
        else:
            lat_final = lat_sum

        if data[5] == "W":
            long_final = long_sum * (-1)
        else:
            long_final = long_sum
    
        #Generating UTM Data from Latitude and Longitude
        data_utm = utm.from_latlon(lat_final,long_final)
        utm_easting = float(data_utm[0])
        utm_northing = float(data_utm[1])
        utm_zone = int(data_utm[2])
        utm_letter = data_utm[3]

        counter = counter + 1 


        #GPS Message
        
        gps_message.Fix = fix
        gps_message.Header.seq = counter
        gps_message.Header.stamp.secs = time_secs 
        gps_message.Header.stamp.nsecs = time_nsecs
        gps_message.Header.frame_id = 'GPS1_Frame'
        gps_message.Latitude = lat_final
        gps_message.Longitude = long_final
        gps_message.Altitude = altitude
        gps_message.UTM_easting = utm_easting
        gps_message.UTM_northing = utm_northing
        gps_message.Zone = utm_zone
        gps_message.Letter = utm_letter

        
        pub.publish(gps_message)
