    #!/usr/bin/env python

import rospy
import math
import matplotlib.pyplot as plt
import time

from parameter import *	
from INS import *
from KF import *
import time
from sensor_msgs.msg import Imu, MagneticField, NavSatFix	 
from xsens_msgs.msg import IMUDATA
from novatel_msgs.msg import RTKPOS, INSPVAX
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

INS_Cal = INS()
KF_Cal = KF()

def Callback_Imu(data):	# ENU
	# xsens_driver
	INS_Data.Gyro[0] = data.angular_velocity.x 
	INS_Data.Gyro[1] = data.angular_velocity.y
	INS_Data.Gyro[2] = data.angular_velocity.z

	INS_Data.Accel[0] = data.linear_acceleration.x 
	INS_Data.Accel[1] = data.linear_acceleration.y
	INS_Data.Accel[2] = - data.linear_acceleration.z 

	INS_Data.seq = data.header.seq

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

# Filter_Hz决定了惯导解算周期，Filter_Numble用于初始对准计数，Sample_Number决定了惯导解算中的子样数
def main(Filter_Hz = 50, Filter_Numble = 1000, Sample_Number = 1):
	rospy.init_node('Subscriber_rawimu', anonymous = True)

	# 需要订阅的话题
	rospy.Subscriber('/xsens_driver/imu/data', Imu, Callback_Imu)
	rospy.Subscriber('/xsens_driver/fix', NavSatFix, Callback_GPS_Service)
	rospy.Subscriber('/novatel_data/rtkpos', RTKPOS, Callback_GPS_Pos)
	rospy.Subscriber('/novatel_data/inspvax', INSPVAX, Callback_GPS_Vel)
	# 需要发布的话题
	pub = rospy.Publisher('/INS', INSPVAX, queue_size = 10)

	rate = rospy.Rate(Filter_Hz)	# set update frequency
	Delta_T = 1.0 / Filter_Hz
	number = Filter_Numble
	
	# 初始对准
	Align_Flag = False	# 初始对准标志位
	Align_Time = 0
	while not Align_Flag:
		if GPS_Data.Service == 1:	# 需要GPS有效
			Align_Time = Align_Time + 1
			# 利用卡尔曼滤波进行对准
			Ang_align = INS_Cal.Alignment_KF(INS_Data.Gyro, INS_Data.Accel, GPS_Data.Pos[0], GPS_Data.Ang[2], number)
			
			# 对准一定时间后认为完成，进行赋值。  对于高精度IMU，需考虑是否滤波结果收敛
			if (Align_Time > number) and (Align_Time % 50)  == 0:
				#print Ang_align
				Align_Flag = INS_Cal.Initial_Alignment(GPS_Data.Pos, GPS_Data.Vel, Ang_align, GPS_Data.Service)
				INS_Cal.Initial_Correct(number)
				Align_Flag = True
				print 'OK'

		rate.sleep()

	i = 0
	j = 1
	Error = numpy.zeros( (15, 1) )
	# 组合导航系统开始工作
	while not rospy.is_shutdown() and Align_Flag:
		# 惯导解算
		Pos, Vel, Ang, Cb2n, Error = INS_Cal.Update(INS_Data.Gyro, INS_Data.Accel, Error, Sample_Number, Delta_T)
		# 导航数据发布
		pub.publish(INS_Message(Pos, Vel, Ang, i))
		i = i + 1
		# 滤波修正惯导解算结果，50个解算周期后运行一次
		if i == 50:	
			Error, Z = KF_Cal.Kalman_Filter(Pos, Vel, Ang, Cb2n, INS_Data.Gyro, INS_Data.Accel, GPS_Data.Pos, GPS_Data.Vel, GPS_Data.Ang, Delta_T × 50)
			print ' '
			j = j + 1			
			i = 0

		rate.sleep()

	rospy.spin()

if __name__ == "__main__":
	main()
