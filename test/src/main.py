#!/usr/bin/env python

import rospy	#python for ros
import math
import numpy 
import matplotlib.pyplot as plt

from parameter import *		# store sensor's data
from INS import *
from KF import *

from sensor_msgs.msg import Imu, MagneticField, NavSatFix	 # msg for imu,mag
from xsens_msgs.msg import IMUDATA
from novatel_msgs.msg import RTKPOS, INSPVAX
from geometry_msgs.msg import TwistStamped

INS_Cal = INS()
KF_Cal = KF()

def Callback_HG1120_Imu(data):		# ENU
	# Xsens_driver
	INS_Data.HG_Gyro[0] = data.angular_velocity.x  #  orig  data.angular_velocity.x 
	INS_Data.HG_Gyro[1] = data.angular_velocity.y  #  orig  data.angular_velocity.y 
	INS_Data.HG_Gyro[2] = data.angular_velocity.z

	INS_Data.HG_Accel[0] = data.linear_acceleration.x 
	INS_Data.HG_Accel[1] = data.linear_acceleration.y
	INS_Data.HG_Accel[2] = - data.linear_acceleration.z #  orig  data.linear_acceleration.z

	INS_Data.HG_seq = data.header.seq

def Callback_Xsens_Imu(data):		# ENU
	# Xsens_driver
	INS_Data.Xsens_Gyro[0] = data.angular_velocity.x  #  orig  data.angular_velocity.x 
	INS_Data.Xsens_Gyro[1] = data.angular_velocity.y  #  orig  data.angular_velocity.y 
	INS_Data.Xsens_Gyro[2] = data.angular_velocity.z

	INS_Data.Xsens_Accel[0] = data.linear_acceleration.x 
	INS_Data.Xsens_Accel[1] = data.linear_acceleration.y
	INS_Data.Xsens_Accel[2] = - data.linear_acceleration.z #  orig  data.linear_acceleration.

	INS_Data.Xsens_seq = data.header.seq

def Callback_Xsens_GPS(data):		# ENU
	# Xsens_driver
	GPS_Data.Xsens_Pos[0] = data.longitude * INS.rad  
	GPS_Data.Xsens_Pos[1] = data.latitude * INS.rad 
	GPS_Data.Xsens_Pos[2] = data.altitude
	GPS_Data.Xsens_Time = data.header.stamp.secs + data.header.stamp.nsecs / 10.0 ** 9

def Callback_GPS_Service(data):
	# Xsens_driver
	GPS_Data.Xsens_Service = data.status.service

def Callback_Novatel_GPS_Pos(data):
	GPS_Data.Novatel_Pos[0] = data.longitude * INS.rad
	GPS_Data.Novatel_Pos[1]	= data.latitude * INS.rad
	GPS_Data.Novatel_Pos[2] = data.altitude
	GPS_Data.Novatel_Time = data.header2.stamp.secs + data.header2.stamp.nsecs / 10.0 ** 9

def Callback_Novatel_GPS_Vel(data):
	GPS_Data.Novatel_Vel[0] = data.east_velocity
	GPS_Data.Novatel_Vel[1] = data.north_velocity
	GPS_Data.Novatel_Vel[2] = data.up_velocity
	GPS_Data.Novatel_Ang[0] = data.pitch * INS.rad 
	GPS_Data.Novatel_Ang[1] = data.roll * INS.rad 
	GPS_Data.Novatel_Ang[2] = data.azimuth * INS.rad 

def main():
	rospy.init_node('Subscriber_rawimu', anonymous = True)

	#rospy.Subscriber('/hg1120/data', Imu, Callback_HG1120_Imu)
	rospy.Subscriber('/xsens_driver/imu/data', Imu, Callback_Xsens_Imu)
	rospy.Subscriber('/xsens_driver/imupos', IMUDATA, Callback_Xsens_GPS)
	rospy.Subscriber('/xsens_driver/fix', NavSatFix, Callback_GPS_Service)
	rospy.Subscriber('/novatel_data/rtkpos', RTKPOS, Callback_Novatel_GPS_Pos)
	rospy.Subscriber('/novatel_data/inspvax', INSPVAX, Callback_Novatel_GPS_Vel)

	Filter_Hz = 50

	rate = rospy.Rate(Filter_Hz)	# set update frequency
	Delta_T = 1.0 / Filter_Hz

	# initial alignment
	Align_Flag = False
	Align_Time = 0
	Initial_Time = 0.0
	Initial_Longitude = 0.0
	Initial_Latitude = 0.0
	Initial_Altitude = 0.0
	Initial_Longitude_Vel = 0.0
	Initial_Latitude_Vel = 0.0
	Initial_Altitude_Vel = 0.0
	#GPS_Data.Xsens_Service = 1
	while not Align_Flag:
		if GPS_Data.Xsens_Service == 1:
			Align_Time = Align_Time + 1
			#Ang_align = INS_Cal.Alignment_KF(INS_Data.Gyro, INS_Data.Accel, 39.9675, 0)
			Ang_align = INS_Cal.Alignment_KF(INS_Data.Xsens_Gyro, INS_Data.Xsens_Accel, GPS_Data.Novatel_Pos[0], GPS_Data.Novatel_Ang[2])

			if (Align_Time > 1000) and (Align_Time % 50)  == 0:
				print Ang_align
				Align_Flag = INS_Cal.Initial_Alignment(GPS_Data.Novatel_Pos, GPS_Data.Novatel_Vel, Ang_align, GPS_Data.Xsens_Service)
				
				Initial_Time = GPS_Data.Novatel_Time
			
				Initial_Longitude = float(GPS_Data.Novatel_Pos[0])
				Initial_Latitude = float(GPS_Data.Novatel_Pos[1])
				Initial_Altitude = float(GPS_Data.Novatel_Pos[2])

				Initial_Longitude_Vel = float(GPS_Data.Novatel_Vel[0])
				Initial_Latitude_Vel` = float(GPS_Data.Novatel_Vel[1])
				Initial_Altitude_Vel = float(GPS_Data.Novatel_Vel[2])

				INS_Cal.Initial_Correct()
				Align_Flag = True
				print 'OK'
		rate.sleep()

	#Align_Flag = INS_Cal.Initial_alignment(0, 0, 0, 1)

	i = 0
	j = 1
	Xsens = 0
	Novatel = 0
	Error = numpy.zeros( (15, 1) )

	Number = 30000
	Xsens_GPS_Time = numpy.zeros( (1,Number) )
	Xsens_IMU_Time = numpy.zeros( (1,Number) )
	Novatel_GPS_Time = numpy.zeros( (1,Number) )
	Xsens_Pos = numpy.zeros( (3,Number) )
	Xsens_Vel = numpy.zeros( (3,Number) )
	Xsens_Ang = numpy.zeros( (3,Number) )
	Xsens_GPS_Pos = numpy.zeros( (3,Number) )
	Xsens_GPS_Vel = numpy.zeros( (3,Number) )
	Xsens_Gyro = numpy.zeros( (3,Number) )
	Xsens_Accel = numpy.zeros( (3,Number) )
	
	Novatel_Pos = numpy.zeros( (3,Number) )
	Novatel_Vel = numpy.zeros( (3,Number) )
	Novatel_Ang = numpy.zeros( (3,Number) )

	while not rospy.is_shutdown() and Align_Flag:
		Pos, Vel, Ang, Cb2n, Error = INS_Cal.Update(INS_Data.Xsens_Gyro, INS_Data.Xsens_Accel, Error, 1, Delta_T)

		Xsens_Gyro[0:3, Xsens] = INS_Data.Xsens_Gyro[0:3, 0]
		Xsens_Accel[0:3, Xsens] = INS_Data.Xsens_Accel[0:3, 0]
		Xsens_Pos[0:3, Xsens] = (Pos[0:3, 0] - [Initial_Longitude, Initial_Latitude, Initial_Altitude]) * INS.deg
		Xsens_Vel[0:3, Xsens] = Vel[0:3, 0]# - [Initial_Longitude_Vel, Initial_Latitude_Vel, Initial_Altitude_Vel]
		Xsens_Ang[0:3, Xsens] = Ang[0:3, 0]
		Xsens_IMU_Time[0, Xsens] = GPS_Data.Xsens_Time - Initial_Time
		Xsens = Xsens + 1

		if Novatel_GPS_Time[0, Novatel-1] < (GPS_Data.Novatel_Time - Initial_Time):
			Novatel_Pos[0:3, Novatel] = (GPS_Data.Novatel_Pos[0:3, 0] - [Initial_Longitude, Initial_Latitude, Initial_Altitude]) * INS.deg
			Novatel_Vel[0:3, Novatel] = GPS_Data.Novatel_Vel[0:3, 0]# - [Initial_Longitude_Vel, Initial_Latitude_Vel, Initial_Altitude_Vel]
			Novatel_Ang[0:3, Novatel] = GPS_Data.Novatel_Ang[0:3, 0]
			Novatel_GPS_Time[0, Novatel] = GPS_Data.Novatel_Time - Initial_Time
	
			Novatel = Novatel + 1

		i = i + 1
		if i == 50:	
			Error, Z = KF_Cal.Kalman_Filter(Pos, Vel, Ang, Cb2n, INS_Data.Xsens_Gyro, INS_Data.Xsens_Accel, GPS_Data.Novatel_Pos, GPS_Data.Novatel_Vel, GPS_Data.Novatel_Ang, 1)
			#print 'The error of Angle is :', Error[8] * INS.deg
			j = j + 1			
			i = 0

		if Xsens >= 50 * 60:
			break

		rate.sleep()


	#plt.figure(1)
	#plt.subplot(211)
	#plt.grid(True)
	#plt.plot(Xsens_IMU_Time[0, 0:Xsens], Xsens_Pos[0, 0:Xsens] * 110000, 'r-', label = 'Xsens',)
	#plt.plot(Novatel_GPS_Time[0, 0:Novatel], Novatel_Pos[0, 0:Novatel] * 110000, 'b-.', label = 'Novatel')
	#plt.xlabel('Time(s)')
	#plt.ylabel('East Pos(m)')
	#plt.legend(loc = 0)
	#plt.subplot(212)
	#plt.grid(True)
	#plt.plot(Xsens_IMU_Time[0, 0:Xsens], Xsens_Vel[0, 0:Xsens], 'r-', lw = 1.5, label = 'Xsens')
	#plt.plot(Novatel_GPS_Time[0, 0:Novatel], Novatel_Vel[0, 0:Novatel], 'b-.', lw = 1.5, label = 'Novatel')
	#plt.xlabel('Time(s)')
	##plt.ylabel('East_Vel(m/s)')
	#plt.legend(loc = 0)

	#plt.figure(2)
	#plt.subplot(211)
	#plt.grid(True)
#	plt.plot(Xsens_IMU_Time[0, 0:Xsens], Xsens_Pos[1, 0:Xsens] * 110000, 'r-', lw = 1.5, label = 'Xsens')
	##plt.plot(Novatel_GPS_Time[0, 0:Novatel], Novatel_Pos[1, 0:Novatel] * 110000, 'b-.', lw = 1.5, label = 'Novatel')
#	plt.xlabel('Time(s)')
	#plt.ylabel('North Pos(m)')
#	plt.legend(loc = 0)
	#plt.subplot(212)
	#plt.grid(True)
	#plt.plot(Xsens_IMU_Time[0, 0:Xsens], Xsens_Vel[1, 0:Xsens], 'r-', lw = 1.5, label = 'Xsens',)
	#plt.plot(Novatel_GPS_Time[0, 0:Novatel], Novatel_Vel[1, 0:Novatel], 'b-.', lw = 1.5, label = 'Novatel')
	#plt.xlabel('Time(s)')
	#plt.ylabel('North_Vel(m/s)')
	#plt.legend(loc = 0)

	plt.figure(3)
	#plt.subplot(311)
	#plt.grid(True)
	#plt.plot(Xsens_IMU_Time[0, 0:Xsens], Xsens_Ang[0, 0:Xsens] * INS.deg, 'r-',  lw = 1.5, label = 'Xsens')
	#plt.plot(Novatel_GPS_Time[0, 0:Novatel], Novatel_Ang[0, 0:Novatel] * INS.deg, 'b-.', lw = 1.5, label = 'Novatel')
	#plt.xlabel('Time(s)')
	#plt.ylabel('Pitch(deg)')
	#plt.legend(loc = 0)
	#plt.subplot(312)
	#plt.grid(True)
	#plt.plot(Xsens_IMU_Time[0, 0:Xsens], Xsens_Ang[1, 0:Xsens] * INS.deg, 'r-', lw = 1.5, label = 'Xsens')
	#plt.plot(Novatel_GPS_Time[0, 0:Novatel], Novatel_Ang[1, 0:Novatel] * INS.deg, 'b-.', lw = 1.5, label = 'Novatel')
	#plt.xlabel('Time(s)')
	#plt.ylabel('Roll(deg)')
	#plt.legend(loc = 0)
	#plt.subplot(313)
	plt.grid(True)
	plt.plot(Xsens_IMU_Time[0, 0:Xsens], Xsens_Ang[2, 0:Xsens] * INS.deg, 'r-', lw = 1.5, label = 'Xsens')
	plt.plot(Novatel_GPS_Time[0, 0:Novatel], Novatel_Ang[2, 0:Novatel] * INS.deg, 'b-.', lw = 1.5, label = 'Novatel')
	plt.xlabel('Time(s)')
	plt.ylabel('Yaw(deg)')
	plt.legend(loc = 0)

	plt.show()
	print 'Done'
	#print Xsens_Ang[2, 1:200]
	rospy.spin()

if __name__ == "__main__":
	main()