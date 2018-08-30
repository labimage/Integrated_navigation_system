#!/usr/bin/env python

import rospy
import serial
import serial.tools.list_ports
import time
import numpy
import math


from sensor_msgs.msg import MagneticField
from std_msgs.msg import Header

#a = serial.tools.list_ports.comports()
#print a[0]
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout = 0.2)
data = ''
data = data.encode('utf-8')
ser.close()

def receive():
	Mag = numpy.zeros([3,1])
	flag = -1

	ser.open()
	data = ser.read(13)
	ser.close()

	i = -1
	while i<12:
		i = i + 1
		if ord(data[i]) == 0x0d:
			flag = i

	if flag < 7:
		Mag_X_ori = ord(data[flag + 1]) * 255 + ord(data[flag + 2])
		Mag_Y_ori = ord(data[flag + 3]) * 255 + ord(data[flag + 4])	
		Mag_Z_ori = ord(data[flag + 5]) * 255 + ord(data[flag + 6])
	else:
		Mag_X_ori = ord(data[flag - 6]) * 255 + ord(data[flag - 5])
		Mag_Y_ori = ord(data[flag - 4]) * 255 + ord(data[flag - 3])	
		Mag_Z_ori = ord(data[flag - 2]) * 255 + ord(data[flag - 1])

	if Mag_X_ori > 32767:
		Mag_X = ( Mag_X_ori - 65535 - 1 ) / 15
	else:
		Mag_X = Mag_X_ori / 15

	if Mag_Y_ori > 32767:
		Mag_Y = ( Mag_Y_ori - 65535 - 1 ) / 15
	else:
		Mag_Y = Mag_Y_ori / 15
		
	if Mag_Z_ori > 32767:
		Mag_Z = ( Mag_Z_ori - 65535 - 1 ) / 15
	else:
		Mag_Z = Mag_Z_ori / 15

	Yaw = math.asin(Mag_X / math.sqrt(Mag_X**2 + Mag_Y**2)) * 180 / 3.1415926


	if Mag_X < 0 and Mag_Y < 0:
		Yaw = 180 - Yaw

	if Mag_X < 0 and Mag_Y > 0:
		Yaw = 360 + Yaw

	if Mag_Y < 0 and 0 < Mag_X:
		Yaw = 180 - Yaw

	Yaw = Yaw + 5.9
	
	if Yaw < 0:
		Yaw = Yaw + 360

	if Yaw > 180:
		Yaw = Yaw - 360

	time_now = time.time()
	secs = int(time.time())
	nsecs = (time_now - secs) * 10**8

	return Yaw, Mag_X, Mag_Y, Mag_Z, secs, nsecs

def main():
	rospy.init_node('HMR2300_Publisher', anonymous = True)
	Mag_pub = rospy.Publisher('/imu/mag', MagneticField, queue_size=10)

	rate = rospy.Rate(2)

	mag_msg = MagneticField()
	mag_msg.header = Header()
	mag_msg.header.frame_id = '/Magnet'

	while not rospy.is_shutdown():	
		Yaw, Mag_X, Mag_Y, Mag_Z, secs, nsecs = receive()
		
		mag_msg.header.stamp.secs = secs
		mag_msg.header.stamp.nsecs = nsecs

		mag_msg.magnetic_field.x = 0
		mag_msg.magnetic_field.y = 0
		mag_msg.magnetic_field.z = Yaw

		mag_msg.magnetic_field_covariance[0] = Mag_X
		mag_msg.magnetic_field_covariance[1] = Mag_Y
		mag_msg.magnetic_field_covariance[2] = Mag_Z

		Mag_pub.publish(mag_msg)
		
		#print mag_msg

		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
    main()