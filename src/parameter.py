import numpy 

class INS_Data():
	Accel = numpy.zeros( (3,1) )
	Gyro = numpy.zeros( (3,1) )
	HG_Accel = numpy.zeros( (3,1) )
	HG_Gyro = numpy.zeros( (3,1) )
	seq = 0
	Novatel_Gyro = numpy.zeros( (3,1) )
	
class GPS_Data():
	Pos = numpy.zeros( (3,1) )
	Vel = numpy.zeros( (3,1) )
	Ang = numpy.zeros( (3,1) )
	Xsens_Pos = numpy.zeros( (3,1) )
	Xsens_Vel = numpy.zeros( (3,1) )
	Xsens_Ang = numpy.zeros( (3,1) )
	Service = 0

class Wheel_Data():
	Vel = numpy.zeros( (3,1) )

class Magnet_Data():
	Mag = numpy.zeros( (3,1) )
	Ang = numpy.zeros( (3,1) )

	