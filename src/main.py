    #!/usr/bin/env python

import rospy	#python for ros
import math
import matplotlib.pyplot as plt
import time

from parameter import *		# store sensor's data
from INS import *
from KF import *
import time
from sensor_msgs.msg import Imu, MagneticField, NavSatFix	 # msg for imu,mag
from xsens_msgs.msg import IMUDATA
from novatel_msgs.msg import RTKPOS, INSPVAX
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

INS_Cal = INS()
KF_Cal = KF()

def Callback_Imu(data):	# ENU
	# xsens_driver
	INS_Data.Gyro[0] = data.angular_velocity.x  #  orig  data.angular_velocity.x 
	INS_Data.Gyro[1] = data.angular_velocity.y  #  orig  data.angular_velocity.y 
	INS_Data.Gyro[2] = data.angular_velocity.z

	INS_Data.Accel[0] = data.linear_acceleration.x 
	INS_Data.Accel[1] = data.linear_acceleration.y
	INS_Data.Accel[2] = - data.linear_acceleration.z #  orig  data.linear_acceleration.

	INS_Data.seq = data.header.seq
	#print 'Gyro:', INS_Data.Gyro


def Callback_GPS_Service(data):
	GPS_Data.Service = data.status.service

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

def INS_Message(Pos, Vel, Ang, i):
	INS_Msg = INSPVAX()
	INS_Msg.header2 = Header()
	INS_Msg.header2.frame_id = '/INS'

	INS_Msg.latitude = Pos[1]
	INS_Msg.longitude = Pos[0]
	INS_Msg.altitude = Pos[2]

	INS_Msg.north_velocity = Vel[1]
	INS_Msg.east_velocity = Vel[0]
	INS_Msg.up_velocity = Vel[2]

	INS_Msg.roll = Ang[1] * INS.deg
	INS_Msg.pitch = Ang[0] * INS.deg
	INS_Msg.azimuth = Ang[2] * INS.deg

	time_now = time.time()
	secs = int(time.time())
	nsecs = (time_now - secs) * 10**8

	INS_Msg.header2.stamp.secs = secs
	INS_Msg.header2.stamp.nsecs = nsecs
	INS_Msg.header2.seq = i

	return INS_Msg

def main(Filter_Hz = 50, Filter_Numble = 50, Sample_Number = 1):
	rospy.init_node('Subscriber_rawimu', anonymous = True)

	#rospy.Subscriber('/hg1120/data', Imu, Callback_Imu)
	rospy.Subscriber('/xsens_driver/imu/data', Imu, Callback_Imu)
	rospy.Subscriber('/xsens_driver/fix', NavSatFix, Callback_GPS_Service)
	rospy.Subscriber('/novatel_data/rtkpos', RTKPOS, Callback_GPS_Pos)
	rospy.Subscriber('/novatel_data/inspvax', INSPVAX, Callback_GPS_Vel)
	pub = rospy.Publisher('/INS', INSPVAX, queue_size = 10)

	rate = rospy.Rate(Filter_Hz)	# set update frequency
	Delta_T = 1.0 / Filter_Hz
	number = Filter_Numble
	# initial alignment
	Align_Flag = False
	Align_Time = 0
	while not Align_Flag:
		if GPS_Data.Service == 1:
			Align_Time = Align_Time + 1
			Ang_align = INS_Cal.Alignment_KF(INS_Data.Gyro, INS_Data.Accel, GPS_Data.Pos[0], GPS_Data.Ang[2], number)

			if (Align_Time > number) and (Align_Time % 50)  == 0:
				print Ang_align
				Align_Flag = INS_Cal.Initial_Alignment(GPS_Data.Pos, GPS_Data.Vel, Ang_align, GPS_Data.Service)
				INS_Cal.Initial_Correct(number)
				Align_Flag = True
				print 'OK'

		rate.sleep()

	i = 0
	j = 1
	Error = numpy.zeros( (15, 1) )
	while not rospy.is_shutdown() and Align_Flag:
		Pos, Vel, Ang, Cb2n, Error = INS_Cal.Update(INS_Data.Gyro, INS_Data.Accel, Error, Sample_Number, Delta_T)
		pub.publish(INS_Message(Pos, Vel, Ang, i))
		i = i + 1

		if i == 50:	
			Error, Z = KF_Cal.Kalman_Filter(Pos, Vel, Ang, Cb2n, INS_Data.Gyro, INS_Data.Accel, GPS_Data.Pos, GPS_Data.Vel, GPS_Data.Ang, 1)
			
			print ' '
			
			j = j + 1			

			i = 0

		rate.sleep()

	rospy.spin()

if __name__ == "__main__":
	main()