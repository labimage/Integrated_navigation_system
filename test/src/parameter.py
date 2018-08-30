import numpy 

class INS_Data():
	Xsens_Accel = numpy.zeros( (3,1) )
	Xsens_Gyro = numpy.zeros( (3,1) )
	Xsens_seq = 0

	HG_Accel = numpy.zeros( (3,1) )
	HG_Gyro = numpy.zeros( (3,1) )
	HG_seq = 0

	Novatel_Gyro = numpy.zeros( (3,1) )
	
class GPS_Data():
	Novatel_Pos = numpy.zeros( (3,1) )
	Novatel_Vel = numpy.zeros( (3,1) )
	Novatel_Ang = numpy.zeros( (3,1) )
	Novatel_Time = 0.0

	Xsens_Pos = numpy.zeros( (3,1) )
	Xsens_Vel = numpy.zeros( (3,1) )
	Xsens_Ang = numpy.zeros( (3,1) )
	Xsens_Time = 0.0

	Xsens_Service = 0

class Wheel_Data():
	Vel = numpy.zeros( (3,1) )

class Magnet_Data():
	Mag = numpy.zeros( (3,1) )
	Ang = numpy.zeros( (3,1) )
	