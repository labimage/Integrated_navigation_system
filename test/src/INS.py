import numpy
import math
import rospy

class INS():
	# Constant
	pi = 3.1415923
	deg = 180 / pi
	rad = pi / 180
	wie = 4.167 * 10 ** (-3) * pi / 180 * 2
	e = 1/297
	Re = 6378137
	g = 9.7803267714
	

	def __init__(self):
		self.Ali_KF_Time = 0
		self.Correct_Flag = 0
		self.Correct_Sum = numpy.zeros((3,1))
		self.Correct_Square = numpy.zeros((3,1))

		self.Ang = numpy.zeros((3,1))		# Body to ENU
		self.Vel = numpy.zeros((3,1))		# ENU
		self.Pos = numpy.zeros((3,1))		# longitude, latitude, altitude
		self.Gyro_bias = numpy.zeros((3,1))	# Gyro bias, rad/s
		self.Acc_bias = numpy.zeros((3,1))	# Acce bias, m/s^2
		self.DCM_b2n = numpy.zeros((3,3))	# Direction Cosine Matrix body to navigation axis
		self.Quaternion = numpy.zeros((4,1))# Quaternion of attitude 
		self.wien = numpy.zeros((3,1))
		self.wenn = numpy.zeros((3,1))
		self.Numble = 1000
		self.Initial_Gyro = numpy.zeros((3, self.Numble))

		self.X_prediction = numpy.zeros( (5,1) )
		self.X = numpy.zeros( (5,1) )
		self.P_prediction = numpy.zeros( (5,5) )
		self.P = numpy.zeros( (5,5) )

		self.A = numpy.zeros( (5,5) )
		self.H = numpy.zeros( (2,5) )
		self.Q = numpy.zeros( (5,5) )
		self.R = numpy.eye(2)

		self.H[0, 1] = - self.g 
		self.H[1, 0] = self.g 
		self.X[0] = 1 * self.rad
		self.X[1] = 1 * self.rad
		self.X[2] = 0 * self.rad 
		self.X[3] = 0.0015 * self.rad 
		self.X[4] = 0.0015 * self.rad 
		self.K = numpy.zeros( (5,2) )

		self.DAngle = numpy.zeros( (3,4) )
		self.DVelocity = numpy.zeros( (3,4) )
		self.DAngle_K = 0
		self.Calcu_Num = 0

	def Alignment_KF(self, Gyro, Acce, Pos, Ang):
		Omig_N = self.wie * math.cos(Pos)
		Omig_U = self.wie * math.sin(Pos)

		if self.Ali_KF_Time < 1000:
			self.Initial_Gyro[:, self.Ali_KF_Time] = Gyro.T
			self.Ali_KF_Time = self.Ali_KF_Time + 1
			#print Gyro.T

		self.A[0, 1] = Omig_U
		self.A[0, 2] = Omig_N
		self.A[1, 0] = - Omig_U
		self.A[1, 4] = 1
		self.A[2, 0] = Omig_N
		self.A[2, 1] = 0

		self.Q[0:3, 0:3] = numpy.eye(3) * (0.0005 * self.rad) **2


		self.R = numpy.eye(2) * 0.001 ** 2

		F = numpy.zeros( (5, 5) )
		F = numpy.eye(5) + self.A * 0.1 / 0.02

		Z = numpy.zeros( (2,1) )
		Z[0] = Acce[0]
		Z[1] = Acce[1]

		self.X_prediction = F.dot(self.X)
		self.P_prediction = F.dot(self.P.dot(F.T)) + self.Q
		self.K = self.P_prediction.dot(self.H.T) \
				.dot( numpy.linalg.inv(self.H.dot(self.P_prediction).dot(self.H.T) + self.R) ) 
		self.X = self.X_prediction + self.K.dot( Z - self.H.dot(self.X_prediction) )
		self.P = ( numpy.eye(5) - self.K.dot(self.H) ).dot(self.P_prediction).dot( (numpy.eye(5) - self.K.dot(self.H)).T )\
					+ self.K.dot(self.R).dot(self.K.T)
		
		#self.X[0] = self.Z[0]
		#self.X[1] = self.Z[1]
		self.X[2] = Ang
		return self.X[0:3] * self.deg

	def Initial_Correct(self):
		if self.Correct_Flag == 0:
			for i in range(1000):	
				self.Correct_Sum[0] = self.Correct_Sum[0] + self.Initial_Gyro[0, i]
				self.Correct_Sum[1] = self.Correct_Sum[1] + self.Initial_Gyro[1, i]
				self.Correct_Sum[2] = self.Correct_Sum[2] + self.Initial_Gyro[2, i]

			#print 'dadasdasdassddd', self.Correct_Sum
			self.Correct_Sum = (self.Correct_Sum) / 1000
			for i in range(1000):
				#self.Correct_Square = self.Correct_Square + numpy.sqrt( (self.Correct_Sum[0] - self.Initial_Gyro[0, i]) ** 2 )

				self.Correct_Square[0] = self.Correct_Square[0] + math.sqrt( (self.Correct_Sum[0] - self.Initial_Gyro[0, i]) ** 2 )
				self.Correct_Square[1] = self.Correct_Square[1] + math.sqrt( (self.Correct_Sum[1] - self.Initial_Gyro[1, i]) ** 2 )
				self.Correct_Square[2] = self.Correct_Square[2] + math.sqrt( (self.Correct_Sum[2] - self.Initial_Gyro[2, i]) ** 2 )
			self.Correct_Square = (self.Correct_Square) / 1000

			self.Correct_Flag = 1

			print 'The sum is:', self.Correct_Sum
			print 'The square is:', self.Correct_Square

	def Initial_Alignment(self, Pos, Vel, Ang, flag):#, mag, pos, vel):
		self.Pos[0] = Pos[0]
		self.Pos[1] = Pos[1]
		self.Pos[2] = Pos[2]

		self.Vel[0] = Vel[0]
		self.Vel[1] = Vel[1]
		self.Vel[2] = Vel[2]

		self.Ang[0] = Ang[0] * self.rad
		self.Ang[1] = Ang[1] * self.rad
		self.Ang[2] = Ang[2] * self.rad

		#self.Pos[0] = 116 * self.rad
		#self.Pos[1] = 39 * self.rad
		#self.Pos[2] = 0

		#self.Vel[0] = 0
		#self.Vel[1] = 0
		#self.Vel[2] = 0

		#self.Ang[0] = - Ang[0] * self.rad
		#self.Ang[1] = - Ang[1] * self.rad
		#self.Ang[2] = Ang[2] * self.rad


		self.Gyro_bias[0] = 0
		self.Gyro_bias[1] = 0
		self.Gyro_bias[2] = 0

		self.Acc_bias[0] = 0
		self.Acc_bias[1] = 0
		self.Acc_bias[2] = 0

		a0 = self.Quaternion[0] = math.cos(self.Ang[0] / 2) * math.cos(self.Ang[1] / 2) * math.cos(self.Ang[2] / 2) + \
						math.sin(self.Ang[0] / 2) * math.sin(self.Ang[1] / 2) * math.sin(self.Ang[2] / 2)
		a1 = self.Quaternion[1] = math.sin(self.Ang[0] / 2) * math.cos(self.Ang[1] / 2) * math.cos(self.Ang[2] / 2) + \
						math.cos(self.Ang[0] / 2) * math.sin(self.Ang[1] / 2) * math.sin(self.Ang[2] / 2)
		a2 = self.Quaternion[2] = math.cos(self.Ang[0] / 2) * math.sin(self.Ang[1] / 2) * math.cos(self.Ang[2] / 2) - \
						math.sin(self.Ang[0] / 2) * math.cos(self.Ang[1] / 2) * math.sin(self.Ang[2] / 2)
		a3 = self.Quaternion[3] = math.sin(self.Ang[0] / 2) * math.sin(self.Ang[1] / 2) * math.cos(self.Ang[2] / 2) - \
						math.cos(self.Ang[0] / 2) * math.cos(self.Ang[1] / 2) * math.sin(self.Ang[2] / 2)

		self.DCM_b2n[0,0] = a0 ** 2 + a1 ** 2 - a2 ** 2 + a3 ** 2
		self.DCM_b2n[0,1] = 2 * ( a1 * a2 - a0 * a3)
		self.DCM_b2n[0,2] = 2 * ( a1 * a3 + a0 * a2)

		self.DCM_b2n[1,0] = 2 * ( a1 * a2 + a0 * a3 )
		self.DCM_b2n[1,1] = a0 ** 2 - a1 ** 2 + a2 ** 2 - a3 ** 2
		self.DCM_b2n[1,2] = 2 * ( a2 * a3 - a0 * a1 )

		self.DCM_b2n[2,0] = 2 * ( a1 * a3 - a0 * a2 )
		self.DCM_b2n[2,1] = 2 * ( a2 * a3 + a0 * a1 )
		self.DCM_b2n[2,2] = a0 ** 2 - a1 ** 2 - a2 ** 2 + a3 ** 2

		if flag == 1:
			Flag_Out = True
			print 'Alignment is succeed!'
			print 'The alignment Pos is:' , self.Pos[0:2] * self.deg
			print 'The alignment Vel is:' , self.Vel[0:2]
			print 'The alignment Ang is:' , self.Ang * self.deg
		else:
			Flag_Out = False
			print 'Alignment is failed! GPS is not online!'

		return Flag_Out

	def Correct(self, gyro, acc, Error):
		if gyro[0] < self.Correct_Sum[0] + 3 * self.Correct_Square[0] and gyro[0] > self.Correct_Sum[0] - 3 * self.Correct_Square[0]:
			gyro[0] = 0
		elif gyro[0] > self.Correct_Sum[0] + 3 * self.Correct_Square[0]:
			gyro[0] = gyro[0] - (self.Correct_Sum[0] + 3 * self.Correct_Square[0])
		elif gyro[0] < self.Correct_Sum[0] -  self.Correct_Square[0]:
			gyro[0] = gyro[0] + (self.Correct_Sum[0] - 3 * self.Correct_Square[0])
		
		if gyro[1] < self.Correct_Sum[1] + 3 * self.Correct_Square[1] and gyro[1] > self.Correct_Sum[1] - 3 * self.Correct_Square[1]:
			gyro[1] = 0
		elif gyro[1] > self.Correct_Sum[1] + 3 * self.Correct_Square[1]:
			gyro[1] = gyro[1] - (self.Correct_Sum[1] + 3 * self.Correct_Square[1])
		elif gyro[1] < self.Correct_Sum[1] - 3 * self.Correct_Square[1]:
			gyro[1] = gyro[1] + (self.Correct_Sum[1] - 3 * self.Correct_Square[1])
		
		if gyro[2] < self.Correct_Sum[2] + 3 * self.Correct_Square[2] and gyro[2] > self.Correct_Sum[2] - 3 * self.Correct_Square[2]:
			gyro[2] = 0
		#elif gyro[2] > self.Correct_Sum[2] + 1 * self.Correct_Square[2]:
		#	gyro[2] = gyro[2] - (self.Correct_Sum[2] + 0 * self.Correct_Square[2])
		#elif gyro[2] < self.Correct_Sum[2] - 1 * self.Correct_Square[2]:
		#	gyro[2] = gyro[2] + (self.Correct_Sum[2] - 0 * self.Correct_Square[2])

		self.Pos[0] = self.Pos[0] - Error[1]
		self.Pos[1] = self.Pos[1] - Error[0]
		self.Pos[2] = self.Pos[2] - Error[2]

		self.Vel = self.Vel - Error[3:6]
		#print 'The filtered Vel is :', self.Vel
		self.Ang = self.Ang - Error[6:9]
		#
		#print 'The filtered Ang is :', self.Ang * self.deg
		#print 'Before', self.Quaternion
		self.Quaternion[0] = math.cos(self.Ang[0] / 2) * math.cos(self.Ang[1] / 2) * math.cos(self.Ang[2] / 2) + \
						math.sin(self.Ang[0] / 2) * math.sin(self.Ang[1] / 2) * math.sin(self.Ang[2] / 2)
		self.Quaternion[1] = math.sin(self.Ang[0] / 2) * math.cos(self.Ang[1] / 2) * math.cos(self.Ang[2] / 2) + \
						math.cos(self.Ang[0] / 2) * math.sin(self.Ang[1] / 2) * math.sin(self.Ang[2] / 2)
		self.Quaternion[2] = math.cos(self.Ang[0] / 2) * math.sin(self.Ang[1] / 2) * math.cos(self.Ang[2] / 2) - \
						math.sin(self.Ang[0] / 2) * math.cos(self.Ang[1] / 2) * math.sin(self.Ang[2] / 2)
		self.Quaternion[3] = math.sin(self.Ang[0] / 2) * math.sin(self.Ang[1] / 2) * math.cos(self.Ang[2] / 2) - \
						math.cos(self.Ang[0] / 2) * math.cos(self.Ang[1] / 2) * math.sin(self.Ang[2] / 2)
		#print 'Before Ang', self.Ang
		#print 'Before', self.Quaternion
		
		#print 'The input attitude is :', Input_Ang * self.deg
		#print 'The CF_Ang is:', CF_Ang * self.deg
		#print 'The resault is: ', resault * self.deg
		error = numpy.zeros( (15, 1) )

		return gyro, acc, error

	def Update(self, gyro_origin, acce_origin, error, Number, Delta_T):
		gyro, acce, Error = self.Correct(gyro_origin, acce_origin, error)
		#print 'The IMU Ang is :', self.Ang * self.deg
		#
		self.wien[0] = 0
		self.wien[1] = self.wie * math.cos( self.Pos[1] )
		self.wien[2] = self.wie * math.sin( self.Pos[1] )

		#
		self.Rm = self.Re * (1 - 2 * self.e + 3 *self. e * math.sin( self.Pos[1] )**2)
		self.Rn = self.Re * (1 + self.e * math.sin( self.Pos[1] )**2)
		self.wenn[0] = self.Vel[1] / (self.Rm + self.Pos[2])
		self.wenn[1] = self.Vel[0] / (self.Rn + self.Pos[2])
		self.wenn[2] = self.Vel[0] * math.tan( self.Pos[1]) / (self.Rn + self.Pos[2])

		#DAng_rate = ( gyro - ( self.DCM_b2n.T.dot( (self.wien + self.wenn) ) ) ) * Delta_T 
		#print 'Orign DAng_rate is :', DAng_rate


		# Accelerate
		Accen = self.DCM_b2n.dot(acce)
		g_vector = numpy.zeros((3,1))
		g_vector[2] = - self.g
		V_cross_w = numpy.zeros((3,1))
		V_cross_w[0] = (2 * self.wien[1] + self.wenn[1]) * self.Vel[2] - (2 * self.wien[2] + self.wenn[2]) * self.Vel[1]
		V_cross_w[1] = (2 * self.wien[2] + self.wenn[2]) * self.Vel[0] - (2 * self.wien[0] + self.wenn[0]) * self.Vel[2]
		V_cross_w[2] = (2 * self.wien[0] + self.wenn[0]) * self.Vel[1] - (2 * self.wien[1] + self.wenn[1]) * self.Vel[0]
		DV = (Accen - g_vector - V_cross_w) * Delta_T
		
		# Update
		self.Update_and_Correct(gyro, DV, Number, Delta_T)
		#print 'however: ', self.Ang * self.deg
		self.Ang[0:2] = self.Complement_Filter(self.Ang, acce)
		#print 'The resault is: ', self.Ang * self.deg
		return [self.Pos, self.Vel, self.Ang, self.DCM_b2n, Error]

	def Update_and_Correct(self, Gyro, Acce, Number, Delta_T):
		if self.DAngle_K < Number:
			self.DAngle[0:3, self.DAngle_K] = Gyro.T * Delta_T#( Gyro - ( self.DCM_b2n.T.dot( (self.wien + self.wenn) ) ) ).T * Delta_T
			self.DVelocity[0:3, self.DAngle_K] = Acce.T	
			self.DAngle_K = self.DAngle_K + 1
			self.Calcu_Num = self.Calcu_Num + 1

		if self.DAngle_K == Number:
			self.DAngle_K = 0

			if Number == 1:
				gyro = self.DAngle[0:3, 0]
				acce = 0
				pos = 0
				DAng_rate = gyro
				DVel_rate = 0
				DPos_rate = 0
				#DAng_rate = (( gyro - Number * ( self.DCM_b2n.T.dot( (self.wien + self.wenn) ) ).T ) * Delta_T * Number).T
				#print("Single sample is %s:" %(DAng_rate))
			elif Number == 2:
				gyro = self.DAngle[0:3, 0] + self.DAngle[0:3, 1] + 2 / 3 * self.Cross_Product(self.DAngle[0:3, 0], self.DAngle[0:3, 1])
				acce = 2 / 3 * ( self.Cross_Product(self.DVelocity[0:3, 0], self.DAngle[0:3, 1]) - \
								self.Cross_Product(self.DVelocity[0:3, 1], self.DAngle[0:3, 0]))
				pos = self.Cross_Product(self.DAngle[0:3, 0], 11/90*self.DVelocity[0:3, 0] + 11/90*self.DVelocity[0:3, 1]) + \
						self.Cross_Product(self.DAngle[0:3, 1], 1/90*self.DVelocity[0:3, 1] - 7/30*self.DVelocity[0:3, 0])
				DAng_rate = gyro.T
				DVel_rate = acce.T
				DPos_rate = pos.T
				#DAng_rate = (( gyro - Number * ( self.DCM_b2n.T.dot( (self.wien + self.wenn) ) ).T ) * Delta_T * Number).T
				#print("Two samples are %s:" %DAng_rate)
			elif Number == 3:
				gyro = self.DAngle[0:3, 0] + self.DAngle[0:3, 1] + self.DAngle[0:3, 2]   \
				+ 9 / 20 * self.Cross_Product(self.DAngle[0:3, 0], self.DAngle[0:3, 2]) \
				+ 27 / 40 * self.Cross_Product(self.DAngle[0:3, 1], self.DAngle[0:3, 2] - self.DAngle[0:3, 0])
				acce = 9/20 * ( self.Cross_Product(self.DVelocity[0:3, 0], self.DAngle[0:3, 2]) + \
								self.Cross_Product(self.DVelocity[0:3, 2], self.DAngle[0:3, 0])) + \
						27/40 * (self.Cross_Product(self.DVelocity[0:3, 0], self.DAngle[0:3, 1]) + \
								self.Cross_Product(self.DVelocity[0:3, 1], self.DAngle[0:3, 0]) + \
								self.Cross_Product(self.DVelocity[0:3, 1], self.DAngle[0:3, 2]) + \
								self.Cross_Product(self.DVelocity[0:3, 2], self.DAngle[0:3, 1]))
				pos = self.Cross_Product(self.DAngle[0:3, 0], 17/140*self.DVelocity[0:3, 0] + 16/35*self.DVelocity[0:3, 1] - 51/560*self.DVelocity[0:3, 2]) + \
						self.Cross_Product(self.DAngle[0:3, 1], -227/560*self.DVelocity[0:3, 0] + 69/560*self.DVelocity[0:3, 1] + 2/35*self.DVelocity[0:3, 2]) + \
						self.Cross_Product(self.DAngle[0:3, 2], -9/70*self.DVelocity[0:3, 0] -73/560*self.DVelocity[0:3, 1] - 1/280*self.DVelocity[0:3, 2])
				DAng_rate = gyro.T
				DVel_rate = acce.T
				DPos_rate = pos.T
				#DAng_rate = (( gyro - Number * ( self.DCM_b2n.T.dot( (self.wien + self.wenn) ) ).T ) * Delta_T * Number).T
				#print("Three samples are %s:" %DAng_rate)
			elif Number == 4:
				gyro = self.DAngle[0:3, 0] + self.DAngle[0:3, 1] + self.DAngle[0:3, 2] + self.DAngle[0:3, 3]  \
				+ 214 / 315 * (self.Cross_Product(self.DAngle[0:3, 0], self.DAngle[0:3, 1]) + self.Cross_Product(self.DAngle[0:3, 2], self.DAngle[0:3, 3]))\
				+ 46 / 105 * (self.Cross_Product(self.DAngle[0:3, 0], self.DAngle[0:3, 2]) + self.Cross_Product(self.DAngle[0:3, 1], self.DAngle[0:3, 3]))\
				+ 54/105 * self.Cross_Product(self.DAngle[0:3, 0], self.DAngle[0:3, 3])\
				+ 214/315 * self.Cross_Product(self.DAngle[0:3, 1], self.DAngle[0:3, 2])
				acce =  54/105 * (self.Cross_Product(self.DVelocity[0:3, 0], self.DAngle[0:3, 1]) + \
								self.Cross_Product(self.DVelocity[0:3, 1], self.DAngle[0:3, 0]) + \
								self.Cross_Product(self.DVelocity[0:3, 2], self.DAngle[0:3, 3]) + \
								self.Cross_Product(self.DVelocity[0:3, 3], self.DAngle[0:3, 2])) + \
						46/105 * (self.Cross_Product(self.DVelocity[0:3, 0], self.DAngle[0:3, 2]) + \
								self.Cross_Product(self.DVelocity[0:3, 2], self.DAngle[0:3, 0]) + \
								self.Cross_Product(self.DVelocity[0:3, 1], self.DAngle[0:3, 3]) + \
								self.Cross_Product(self.DVelocity[0:3, 3], self.DAngle[0:3, 1])) + \
						214/315 * (self.Cross_Product(self.DVelocity[0:3, 0], self.DAngle[0:3, 3]) + \
								self.Cross_Product(self.DVelocity[0:3, 3], self.DAngle[0:3, 0]) + \
								self.Cross_Product(self.DVelocity[0:3, 1], self.DAngle[0:3, 2]) + \
								self.Cross_Product(self.DVelocity[0:3, 2], self.DAngle[0:3, 1]))
				pos = self.Cross_Product(self.DAngle[0:3, 0], 797/5670*self.DVelocity[0:3, 0] + 1103/1890*self.DVelocity[0:3, 1] + 47/630*self.DVelocity[0:3, 2] - 47/810*self.DVelocity[0:3, 3]) + \
						self.Cross_Product(self.DAngle[0:3, 1], -307/630*self.DVelocity[0:3, 0] + 43/378*self.DVelocity[0:3, 1] + 629/1890*self.DVelocity[0:3, 2] - 13/270*self.DVelocity[0:3, 3]) + \
						self.Cross_Product(self.DAngle[0:3, 2], -37/3780*self.DVelocity[0:3, 0] - 79/270*self.DVelocity[0:3, 1] + 173/1890*self.DVelocity[0:3, 2] + 61/1890*self.DVelocity[0:3, 3]) + \
						self.Cross_Product(self.DAngle[0:3, 3], -1091/5670*self.DVelocity[0:3, 0] - 59/630*self.DVelocity[0:3, 1] - 187/1890*self.DVelocity[0:3, 2] - 1/5670*self.DVelocity[0:3, 3]) 
				DAng_rate = gyro.T
				DVel_rate = acce.T
				DPos_rate = pos.T
			else:
				gyro = self.DAngle[0:3, 0]
				acce = 0
				pos = 0
				DAng_rate = gyro
				DVel = 0
				DPos = 0
				#DAng_rate = (( gyro - Number * ( self.DCM_b2n.T.dot( (self.wien + self.wenn) ) ).T ) * Delta_T * Number).T	
				#print("Four samples are %s:" %DAng_rate)
		
			#print('The DDDGryo is :', DAng_rate[2])

			# update attitude
			DAng = math.sqrt( DAng_rate[0] ** 2 + DAng_rate[1] ** 2 + DAng_rate[2] ** 2 )

			Delta_Ang = numpy.zeros((4,4))
			Delta_Ang[0,1] = -DAng_rate[0]
			Delta_Ang[0,2] = -DAng_rate[1]
			Delta_Ang[0,3] = -DAng_rate[2]

			Delta_Ang[1,0] = DAng_rate[0]
			Delta_Ang[1,2] = DAng_rate[2]
			Delta_Ang[1,3] = - DAng_rate[1]

			Delta_Ang[2,0] = DAng_rate[1]
			Delta_Ang[2,1] = - DAng_rate[2]
			Delta_Ang[2,3] = DAng_rate[0]

			Delta_Ang[3,0] = DAng_rate[2]
			Delta_Ang[3,1] = DAng_rate[1]
			Delta_Ang[3,2] = - DAng_rate[0]

			I = numpy.eye((4))
			
			self.Quaternion = ( ( 1 - DAng ** 2 / 8 ) * I + ( 0.5 - DAng ** 2 / 48 ) * Delta_Ang ).dot (self.Quaternion )
			D = math.sqrt( self.Quaternion[0] ** 2 + self.Quaternion[1] ** 2 + self.Quaternion[2] ** 2 + self.Quaternion[3] ** 2 )
			self.Quaternion = self.Quaternion / D
			#print 'After', self.Quaternion
			a0 = self.Quaternion[0]
			a1 = self.Quaternion[1]
			a2 = self.Quaternion[2]
			a3 = self.Quaternion[3]

			self.DCM_b2n[0,0] = a0 ** 2 + a1 ** 2 - a2 ** 2 + a3 ** 2
			self.DCM_b2n[0,1] = 2 * ( a1 * a2 - a0 * a3)
			self.DCM_b2n[0,2] = 2 * ( a1 * a3 + a0 * a2)

			self.DCM_b2n[1,0] = 2 * ( a1 * a2 + a0 * a3 )
			self.DCM_b2n[1,1] = a0 ** 2 - a1 ** 2 + a2 ** 2 - a3 ** 2
			self.DCM_b2n[1,2] = 2 * ( a2 * a3 - a0 * a1 )

			self.DCM_b2n[2,0] = 2 * ( a1 * a3 - a0 * a2 )
			self.DCM_b2n[2,1] = 2 * ( a2 * a3 + a0 * a1 )
			self.DCM_b2n[2,2] = a0 ** 2 - a1 ** 2 - a2 ** 2 + a3 ** 2

			# Pitch
			self.Ang[0] = math.asin( self.DCM_b2n[2,1] )

			# Roll
			if self.DCM_b2n[2,2] != 0:
				roll_main = math.atan( -self.DCM_b2n[2,0] / self.DCM_b2n[2,2] )
			else:
				self.Ang[1] = pi / 2

			if -self.DCM_b2n[2,0] > 0:                                               
				if self.DCM_b2n[2,2] > 0: 
					self.Ang[1] = roll_main                             
				else:                                                                   
					self.Ang[1] = roll_main + INS.pi                             
			elif -self.DCM_b2n[2,0] == 0: 
				self.Ang[1] = 0
			else:                                                                    
				if self.DCM_b2n[2,2] > 0:                                                 
					self.Ang[1] = roll_main                                  
				else:                                                                
					self.Ang[1] = roll_main - INS.pi                           

			# Yaw
			if self.DCM_b2n[1,1]  != 0:
				yaw_main = math.atan( self.DCM_b2n[0,1] / self.DCM_b2n[1,1] )
			else:
				self.Ang[2] = pi / 2

			if self.DCM_b2n[0,1] < 0:
				if self.DCM_b2n[1,1] > 0:
					self.Ang[2] = - yaw_main
				else:
					self.Ang[2] = INS.pi - yaw_main
			elif self.DCM_b2n[0,1] == 0:
				self.Ang[2] = 0
			else:
				if self.DCM_b2n[1,1] > 0:
					self.Ang[2] = - yaw_main
				else:
					self.Ang[2] = - yaw_main - INS.pi
			self.Ang[2] = - self.Ang[2]	
			if self.Ang[2] < 0:
				self.Ang[2] = 360 * self.rad + self.Ang[2] 		
			#print('The Angle is :', self.Ang * self.deg)

			# Update velocity
			Vel_pre = self.Vel
			self.Vel = self.Vel + Acce - DVel_rate

			# Update position
			# type 1
			self.Pos[0] = self.Pos[0] + self.Vel[0] * Delta_T / ( ( self.Rn + self.Pos[2] ) * math.cos( self.Pos[1] ) ) * self.rad
			self.Pos[1] = self.Pos[1] + self.Vel[1] * Delta_T / ( self.Rm + self.Pos[2] ) * self.rad
			self.Pos[2] = self.Pos[2] + self.Vel[2] * Delta_T
			#print self.Pos[1]
			# type 2
			self.Pos[0] = self.Pos[0] + 0.5 * ( self.Vel[0] + Vel_pre[0] ) * Delta_T / ( ( self.Rn + self.Pos[2] ) * math.cos( self.Pos[1] ) )
			self.Pos[1] = self.Pos[1] + 0.5 * ( self.Vel[1] + Vel_pre[1] ) * Delta_T / ( self.Rm + self.Pos[2] )
			self.Pos[2] = self.Pos[2] + 0.5 * ( self.Vel[2] + Vel_pre[2] ) * Delta_T 

	def Complement_Filter(self, Input_Ang, CF_Acce):
		CF_Ang = numpy.zeros((2,1))
		resault = numpy.zeros((2,1))

		if CF_Acce[2] != 0:
			CF_Ang[0] = - math.atan( CF_Acce[1] / CF_Acce[2])
			CF_Ang[1] = math.atan( CF_Acce[0] / CF_Acce[2])
		else:
			CF_Ang[0] = 0
			CF_Ang[1] = 0

		if CF_Acce[2] > 0 and CF_Acce[0] > 0:
			CF_Ang[1] = CF_Ang[1] - 180 * self.rad
		elif CF_Acce[2] > 0 and CF_Acce[0] < 0:
			CF_Ang[1] = CF_Ang[1] + 180 * self.rad
		else:
			CF_Ang[1] = CF_Ang[1]
		
		K = 0.0002

		resault[0] = ( 1 - K ) * Input_Ang[0] + K * CF_Ang[0]
		resault[1] = ( 1 - K ) * Input_Ang[1] + K * CF_Ang[1]
		
		if resault[0] > 90 * self.rad:
			resault[0] = resault[0] - 180 * self.rad
		elif resault[0] < -90 * self.rad:
			resault[0] = resault[0] + 180 * self.rad

		if resault[1] > 180 * self.rad:
			resault[1] = resault[1] - 360 * self.rad
		elif resault[1] < -180 * self.rad:
			resault[1] = resault[1] + 360 * self.rad

		self.Ang[0] = resault[0]
		self.Ang[1] = resault[1]	

		#print 'Before Ang', self.Ang
		#print 'Before', self.Quaternion
		#print 'After', self.Quaternion
		#print 'The input attitude is :', Input_Ang * self.deg
		#print 'The CF_Ang is:', CF_Ang * self.deg
		#print 'The resault is: ', resault * self.deg
		return resault


	def Cross_Product(self, a, b):
		c = numpy.zeros((3,1))
		c[0] = a[1] * b[2] - b[1] * a[2]
		c[1] = a[2] * b[0] - a[0] * b[2]
		c[2] = a[0] * b[1] - a[1] * b[0]

		return c.T