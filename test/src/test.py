#!/usr/bin/env python

#import rospy	#python for ros
from parameter import *		# store sensor's data
from INS import *
from KF import *
import matplotlib.pyplot as plt

from sensor_msgs.msg import Imu, NavSatFix	 # msg for imu,mag
from xsens_msgs.msg import IMUDATA
from novatel_msgs.msg import RTKPOS, INSPVAX, CORRIMUDATA

def Callback_Imudata(data):		# ENU
	# hg1120
	INS_Data.HG_Gyro[0] = data.angular_velocity.x  #  orig  data.angular_velocity.x 
	INS_Data.HG_Gyro[1] = data.angular_velocity.y  #  orig  data.angular_velocity.y 
	INS_Data.HG_Gyro[2] = data.angular_velocity.z

	INS_Data.HG_Accel[0] = data.linear_acceleration.x 
	INS_Data.HG_Accel[1] = data.linear_acceleration.y
	INS_Data.HG_Accel[2] = - data.linear_acceleration.z #  orig  data.linear_acceleration.

	INS_Data.seq = data.header.seq

def Callback_Imu(data):		# ENU
	# xsens_driver
	INS_Data.Gyro[0] = data.angular_velocity.x  #  orig  data.angular_velocity.x 
	INS_Data.Gyro[1] = data.angular_velocity.y  #  orig  data.angular_velocity.y 
	INS_Data.Gyro[2] = data.angular_velocity.z

	INS_Data.Accel[0] = data.linear_acceleration.x 
	INS_Data.Accel[1] = data.linear_acceleration.y
	INS_Data.Accel[2] = - data.linear_acceleration.z #  orig  data.linear_acceleration.


def Callback_GPS_Service(data):
	GPS_Data.Service = data.status.service

def Callback_GPS_Xsens_Pos(data):
	GPS_Data.Xsens_Pos[0] = data.longitude
	GPS_Data.Xsens_Pos[1] = data.latitude
	GPS_Data.Xsens_Pos[2] = data.altitude

def Callback_GPS_Pos(data):
	GPS_Data.Pos[0] = data.longitude
	GPS_Data.Pos[1]	= data.latitude
	GPS_Data.Pos[2] = data.altitude

def Callback_GPS_Vel(data):
	GPS_Data.Vel[0] = data.east_velocity
	GPS_Data.Vel[1] = data.north_velocity
	GPS_Data.Vel[2] = data.up_velocity
	GPS_Data.Ang[0] = data.pitch
	GPS_Data.Ang[1] = data.roll
	GPS_Data.Ang[2] = data.azimuth

def Callback_Novatel_Gyro(data):
	INS_Data.Novatel_Gyro[0] = data.pitch_rate  #  orig  data.angular_velocity.x 
	INS_Data.Novatel_Gyro[1] = data.roll_rate  #  orig  data.angular_velocity.y 
	INS_Data.Novatel_Gyro[2] = data.yaw_rate

def main():
	rospy.init_node('Subscriber_rawimu', anonymous = True)

	rospy.Subscriber('/hg1120/data', Imu, Callback_Imudata)
	#rospy.Subscriber('/xsens_driver/imupos', IMUDATA, Callback_Imudata)
	rospy.Subscriber('/xsens_driver/imu/data', Imu, Callback_Imu)
	rospy.Subscriber('/xsens_driver/imupos', IMUDATA, Callback_GPS_Xsens_Pos)
	rospy.Subscriber('/xsens_driver/fix', NavSatFix, Callback_GPS_Service)

	rospy.Subscriber('/novatel_data/rtkpos', RTKPOS, Callback_GPS_Pos)
	rospy.Subscriber('/novatel_data/inspvax', INSPVAX, Callback_GPS_Vel)
	rospy.Subscriber('/novatel_data/corrimudata', CORRIMUDATA, Callback_Novatel_Gyro)

	Filter_Hz = 10
	rate = rospy.Rate(Filter_Hz)

	
		#print 'Yaw: ', GPS_Data.Ang[2]
		#print 'Xsens_Gyro: ', INS_Data.Gyro
		#print 'Novatel_Gyro: ', INS_Data.Novatel_Gyro
	i = 0
	while not rospy.is_shutdown(): 

		print 'Yaw:         		   ', GPS_Data.Ang[2]
		print 'Xsens_Gyro:  		   ', INS_Data.Gyro[2] * 57.3
		print 'HG1120_Gyro: 		   ', INS_Data.HG_Gyro[2] * 57.3
		print 'Novatel_Gyro: 		   ', INS_Data.Novatel_Gyro[2] * 57.3
		print 'The error of latitude:  ', (GPS_Data.Pos[1] - GPS_Data.Xsens_Pos[1]) * 110000
		print 'The error of longitude: ', (GPS_Data.Pos[0] - GPS_Data.Xsens_Pos[0]) * 110000 * 0.5
		print ''

		#with open('data.txt', 'a+') as f:  
		#	f.write('\nYaw: \n')
		#	f.write(str(GPS_Data.Ang[2]))
		#	f.write('\nXsens_Gyro: \n')
		#	f.write(str(INS_Data.Gyro*57.3))
		#	f.write('\nNovatel_Gyro: \n')
		#	f.write(str(INS_Data.Novatel_Gyro*57.3))
		i = i + 1
		if i >= 100:
			break
		rate.sleep()

	
	rospy.spin()


if __name__ == "__main__":
	main()