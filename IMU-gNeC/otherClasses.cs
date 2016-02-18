using System;

namespace IMU_gNeC
{
	public class anglesIMU
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
		
	public class IMUComand
	{
		public float mainAngleROMrange = 30;
		public float secundaryAngle1ROMrange = 10;
		public float secundaryAngle2ROMrange = 10;
		public string mainAngleROMangle = "Y";
		public string secundaryAngle1ROMangle = "P";
		public string secundaryAngle2ROMangle = "R";
		public float timePerm = 1;
		public float thetaPerm = 5;

	}








	#region clase generadora ImuYPR

	public class IMUEventArgs : EventArgs
	{
		private readonly float yaw = 0;
		private readonly float pitch = 0;
		private readonly float roll = 0;
		// Constructor.
		public IMUEventArgs(float yaw, float pitch, float roll)
		{
			this.yaw = yaw;
			this.pitch = pitch;
			this.roll = roll;
		}
		//Properties.
		public float Yaw { get { return yaw; } }
		public float Pitch { get { return pitch; } }
		public float Roll { get { return roll; } }
	}

	public delegate void IMUEventHandler(object sender, IMUEventArgs e);

	// The ImuYPR class that raises the IMU event.
	//
	public class IMUAngle
	{
		private float yaw = 0;
		private float pitch = 0;
		private float roll = 0;

		public float Yaw
		{
			get { return yaw; }
			set { yaw = value; }
		}
		public float Pitch
		{
			get { return pitch; }
			set { pitch = value; }
		}
		public float Roll
		{
			get { return roll; }
			set { roll = value; }
		}

		public event IMUEventHandler ImuYPR;

		protected virtual void OnImuYPR(IMUEventArgs e)
		{
			if (ImuYPR != null)
			{
				// Invokes the delegates. 
				ImuYPR(this, e);
			}
		}

		public void SendAngle()
		{
			IMUEventArgs e = new IMUEventArgs(yaw, pitch, roll);
			OnImuYPR(e);
		}
	}

	#endregion 







}

