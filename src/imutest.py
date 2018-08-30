#!/usr/bin/env python

import rospy	#python for ros
from parameter import *		# store sensor's data
from INS import *
from KF import *

from sensor_msgs.msg import Imu, MagneticField	 # msg for imu,mag
from tbw_shanqi_msgs.msg import ExtraWheelSpeedReportStamped 	# msg for wheelspeed

INS_Cal = INS()
KF_Cal = KF()

def Callback_Imu(data):		# ENU
	INS_Data.Gyro[0] = data.angular_velocity.x
	INS_Data.Gyro[1] = data.angular_velocity.y
	INS_Data.Gyro[2] = - data.angular_velocity.z

	INS_Data.Accel[0] = data.linear_acceleration.x
	INS_Data.Accel[1] = data.linear_acceleration.y
	INS_Data.Accel[2] = - data.linear_acceleration.z

	#gyro = INS_Data.Gyro[0] ** 2 + INS_Data.Gyro[1] ** 2 + INS_Data.Gyro[2] ** 2
	#print 'Gyro: ', math.sqrt(gyro)

	#Accel = INS_Data.Accel[0] ** 2 + INS_Data.Accel[1] ** 2 + INS_Data.Accel[2] ** 2
	#print 'Gyro: ', math.sqrt(Accel)
	#rospy.loginfo("Accelerator:%f", INS_Data.Accel[0])

def main():
	rospy.init_node('Subscriber_rawimu', anonymous = True)
	rospy.Subscriber('/imu/data', Imu, Callback_Imu)
	
	rate = rospy.Rate(50)	# set update frequency
	Delta_T = 1 / 50
	
	# IMU test
	Test_Flag = 1
	data_sum = numpy.zeros( (6, 1) )
	data_sqare = numpy.zeros( (6, 1) )
	j = 0

	while not rospy.is_shutdown():
		j = j + 1
		if Test_Flag == 1:
			data_sum[0] = (data_sum[0] + INS_Data.Accel[0]) 
			data_sum[1] = (data_sum[1] + INS_Data.Accel[1]) 
			data_sum[2] = (data_sum[2] + INS_Data.Accel[2] + 9.7803267714) 

			data_sum[3] = (data_sum[3] + INS_Data.Gyro[0]) 
			data_sum[4] = (data_sum[4] + INS_Data.Gyro[1]) 
			data_sum[5] = (data_sum[5] + INS_Data.Gyro[2]) 

			if j == 50 * 15:
				#print 'data_sum:', data_sum / j
				j = 0
				data_sum = numpy.zeros( (6, 1) )
				Test_Flag = 2

		if Test_Flag == 2:
			data_sqare[0] = data_sqare[0] + ( INS_Data.Accel[0] - data_sum[0] ) ** 2
			data_sqare[1] = data_sqare[1] + ( INS_Data.Accel[1] - data_sum[1] ) ** 2
			data_sqare[2] = data_sqare[2] + ( INS_Data.Accel[2] + 9.7803267714 - data_sum[2] ) ** 2

			data_sqare[3] = data_sqare[3] + ( INS_Data.Gyro[0] - data_sum[3] ) ** 2
			data_sqare[4] = data_sqare[4] + ( INS_Data.Gyro[1] - data_sum[4] ) ** 2
			data_sqare[5] = data_sqare[5] + ( INS_Data.Gyro[2] - data_sum[5] ) ** 2

			if j == 50 * 15:
				#print 'data_sqare pre:', data_sqare
				data_sqare[0] = math.sqrt(data_sqare[0] / j)
				data_sqare[1] = math.sqrt(data_sqare[1] / j)
				data_sqare[2] = math.sqrt(data_sqare[2] / j)
				data_sqare[3] = math.sqrt(data_sqare[3] / j)
				data_sqare[4] = math.sqrt(data_sqare[4] / j)
				data_sqare[5] = math.sqrt(data_sqare[5] / j)
				#=print 'data_sqare:', data_sqare
				j = 0
				data_sqare = numpy.zeros( (6, 1) )
				Test_Flag = 3

		if Test_Flag == 3:
			Test_Flag = 1

		rate.sleep()

	rospy.spin()

if __name__ == "__main__":
	main()