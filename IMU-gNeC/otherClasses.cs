using System;

namespace IMU_gNeC
{
	public class dataIMU
	{
		public float yaw;
		public float pitch;
		public float roll;
		public string timeStamp;
		public string X0;
		public string X1;
		public string X2;
		public string Y0;
		public string Y1;
		public string Y2;
		public string Z0;
		public string Z1;
		public string Z2;
		public bool calibrationTimeStamp;
	}
		
	public class setupParameters
	{
		public float mainAngleROMrange = 30;
		public float secundaryAngle1ROMrange = 10;
		public float secundaryAngle2ROMrange = 10;
		public string mainAngleROMangle = "Y";
		public string secundaryAngle1ROMangle = "P";
		public string secundaryAngle2ROMangle = "R";
		public float timePerm = 1;
		public float thetaPerm = 5;
        public int or_init = 4;
        public int rotation = 2;
        public int rom = 1;

	}
}

