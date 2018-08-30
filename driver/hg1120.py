#!/usr/bin/env python

import rospy
import serial
import serial.tools.list_ports
import time
import numpy
import math

from sensor_msgs.msg import Imu
from std_msgs.msg import Header

#a = serial.tools.list_ports.comports()
#print a[0]
ser = serial.Serial('/dev/ttyUSB0', 1000000, timeout = 0.01)
data = ''
data = data.encode('utf-8')
ser.close()

def receive():
	Imu_data = numpy.zeros([6,1])
	Gyro = numpy.zeros([3,1])
	Acce = numpy.zeros([3,1])
	ser.open()
	data = ser.read(105)
	ser.close()

	#print('The data is: ', data)

	i = -1
	flag = 0
	while i < 103:
		i = i + 1
		if ord(data[i]) == 0x0e and ord(data[i+1]) == 0x04 and (i+12 < 105):
			flag = i
			#print i
			break

	#print('Begin byte is :', data[flag])

	Gyro[0] = ord(data[flag + 3]) * 256 + ord(data[flag + 2])
	Gyro[1] = ord(data[flag + 5]) * 256 + ord(data[flag + 4])
	Gyro[2] = ord(data[flag + 7]) * 256 + ord(data[flag + 6])
	Acce[0] = ord(data[flag + 9]) * 256 + ord(data[flag + 8])
	Acce[1] = ord(data[flag + 11]) * 256 + ord(data[flag + 10])
	Acce[2] = ord(data[flag + 13]) * 256 + ord(data[flag + 12])

	j = 0
	while j < 3:
		if Gyro[j] > 32767:
			Gyro[j] = ( Gyro[j] - 65535 - 1 )
			Imu_data[j] = Gyro[j] * 0.001144409
		else:
			Imu_data[j] = Gyro[j] * 0.001144409

		if Acce[j] > 32767:
			Acce[j] = ( Acce[j] - 65535 - 1 )
			Imu_data[j+3] = Acce[j] * 0.02232422
		else:
			Imu_data[j+3] = Acce[j] * 0.02232422
		j = j + 1

	#print Gyro, Acce
	#print('The IMU data is %f.', Imu_data)

	time_now = time.time()
	secs = int(time.time())
	nsecs = (time_now - secs) * 10**8

	return Imu_data, secs, nsecs

def main():
	rospy.init_node('HG1120_Publisher', anonymous = True)
	Imu_pub = rospy.Publisher('imu/data', Imu, queue_size = 10)
	rate = rospy.Rate(75)

	imu_msg = Imu()
	imu_msg.header = Header()
	imu_msg.header.frame_id = '/Imu'

	i = 1
	while not rospy.is_shutdown():	
		Imu_raw, secs, nsecs = receive()

		#print Imu_raw 

		time_now = time.time()
		secs = int(time.time())
		nsecs = (time_now - secs) * 10**8

		imu_msg.header.stamp.secs = secs
		imu_msg.header.stamp.nsecs = nsecs
		imu_msg.header.seq = i
		if i > 65530:
			i = 0

		imu_msg.angular_velocity.x = Imu_raw[1]
		imu_msg.angular_velocity.y = Imu_raw[2]	
		imu_msg.angular_velocity.z = Imu_raw[0]
		imu_msg.linear_acceleration.x = Imu_raw[4]
		imu_msg.linear_acceleration.y = Imu_raw[5]
		imu_msg.linear_acceleration.z = Imu_raw[3]

		Imu_pub.publish(imu_msg)
		i = i + 1

		#print imu_msg

		rate.sleep()

	rospy.spin()

if __name__ == '__main__':
    main()
