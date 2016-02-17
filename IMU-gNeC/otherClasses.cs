using System;

namespace IMU_gNeC
{
	#region other classess

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

	public class posIMU
	{
		public int posX;
		public int posY;
		public bool calibrationTimeStamp;
	}

	public class EnlazaControlInfo
	{
		public byte typeOfControl;
		public byte kalmanOn;
		public float kalman_b;
		public float sampFreq;
		public bool lateralControl;
		public float horizontalRange;
		public float verticalRange;
		public float screenWidth;
		public float screenheight;
	}



	#region eventos YPR

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

	#region clase lectora ImuYPR

	public class readImuYPR
	{
		public void IMUrecieved(object sender, IMUEventArgs e)
		{
		}
	}

	#endregion

	#endregion 

	#region INTERPLAY: eventos Orientación 

	#region clase generadora ImuOR

	public class ImuOREventArgs : EventArgs
	{
		private readonly float mainAngle = 0;
		// Constructor.
		public ImuOREventArgs(float mainAngle)
		{
			this.mainAngle = mainAngle;
		}
		//Properties.
		public float MainAngle { get { return mainAngle; } }
	}

	public delegate void ImuOREventHandler(object sender, ImuOREventArgs e);

	// The ImuYPR class that raises the IMU event.
	//
	public class ImuOrientation
	{
		private float mainAngle = 0;

		public float MainAngle
		{
			get { return mainAngle; }
			set { mainAngle = value; }
		}

		public event ImuOREventHandler ImuOR;

		protected virtual void OnImuOR(ImuOREventArgs e)
		{
			if (ImuOR != null)
			{
				// Invokes the delegates. 
				ImuOR(this, e);
			}
		}

		public void SendAngle()
		{
			ImuOREventArgs e = new ImuOREventArgs(mainAngle);
			OnImuOR(e);
		}
	}

	#endregion

	#endregion

	#region INTERPLAY: eventos permanencia 

	#region clase generadora ImuPerm

	public class ImuPermEventArgs : EventArgs
	{
		public float Alpha;

		public ImuPermEventArgs(float alpha)
		{
			this.Alpha= alpha;
		}
	}

	public delegate void ImuPermEventHandler(object sender, ImuPermEventArgs e);

	// The ImuYPR class that raises the IMU event.
	//
	public class ImuPermanencia
	{
		private float alpha = 0;

		public float Alpha
		{
			get { return alpha; }
			set { alpha = value; }
		}

		public event ImuPermEventHandler ImuPerm;

		protected virtual void OnImuPerm(ImuPermEventArgs e)
		{
			if (ImuPerm != null)
			{
				// Invokes the delegates. 
				ImuPerm(this, e);
			}
		}

		public void SendPerm()
		{
			ImuPermEventArgs e = new ImuPermEventArgs(alpha);
			OnImuPerm(e);
		}
	}

	#endregion

	#endregion

	#region INTERPLAY: salida/vuelta del/al ROM requerido

	#region clase generadora ImuROM

	public class ImuROMEventArgs : EventArgs
	{
		private readonly float mainAngle = 0;
		private readonly bool romOK = false;
		// Constructor.
		public ImuROMEventArgs(float mainAngle, bool romOK)
		{
			this.mainAngle = mainAngle;
			this.romOK = romOK;
		}
		//Properties.
		public float MainAnglea { get { return mainAngle; } }
		public bool RomOK { get { return romOK; } }
	}

	public delegate void ImuROMEventHandler(object sender, ImuROMEventArgs e);

	public class ImuROMChanged
	{
		private float mainAngle = 0;
		private bool romOK = false;

		public float MainAngle
		{
			get { return mainAngle; }
			set { mainAngle = value; }
		}

		public bool RomOK
		{
			get { return romOK; }
			set { romOK = value; }
		}

		public event ImuROMEventHandler ImuROM;

		protected virtual void OnImuROM(ImuROMEventArgs e)
		{
			if (ImuROM != null)
			{
				// Invokes the delegates. 
				ImuROM(this, e);
			}
		}

		public void SendROM()
		{
			ImuROMEventArgs e = new ImuROMEventArgs(mainAngle, romOK);
			OnImuROM(e);
		}
	}

	#endregion

	#endregion


	#region INTERPLAY: parámetros configuracion

	#region clase generadora parámetros configuracion

	public class IMUComandEventArgs : EventArgs
	{
		private readonly float mainAngleROMrange = 30;
		private readonly float secundaryAngle1ROMrange = 10;
		private readonly float secundaryAngle2ROMrange = 10;
		private readonly string mainAngleROMangle = "Y";
		private readonly string secundaryAngle1ROMangle = "P";
		private readonly string secundaryAngle2ROMangle = "R";
		private readonly float timePerm = 1;
		private readonly float thetaPerm = 5;

		// Constructor.
		public IMUComandEventArgs(
			float mainAngleROMrange, float secundaryAngle1ROMrange, float secundaryAngle2ROMrange,
			string mainAngleROMangle, string secundaryAngle1ROMangle, string secundaryAngle2ROMangle, 
			float timePerm, float thetaPerm)
		{
			this.mainAngleROMrange = mainAngleROMrange;
			this.secundaryAngle1ROMrange = secundaryAngle1ROMrange;
			this.secundaryAngle1ROMrange = secundaryAngle2ROMrange;
			this.mainAngleROMangle = mainAngleROMangle;
			this.secundaryAngle1ROMangle = secundaryAngle1ROMangle;
			this.secundaryAngle1ROMangle = secundaryAngle2ROMangle; 
			this.timePerm = timePerm;
			this.thetaPerm = thetaPerm;
		}

		//Properties.
		public float MainAngleROMrange { get { return mainAngleROMrange; } }
		public float SecundaryAngle1ROMrange { get { return secundaryAngle1ROMrange; } }
		public float SecundaryAngle2ROMrange { get { return secundaryAngle2ROMrange; } }
		public string MainAngleROMangle { get { return mainAngleROMangle; } }
		public string SecundaryAngle1ROMangle { get { return secundaryAngle1ROMangle; } }
		public string SecundaryAngle2ROMangle { get { return secundaryAngle2ROMangle; } }
		public float TimePerm { get { return timePerm; } }
		public float ThetaPerm { get { return thetaPerm; } }
	}

	public delegate void IMUComandEventHandler(object sender, IMUComandEventArgs e);

	// The ImuYPR class that raises the IMU event.
	//
	public class IMUComand
	{
		private float mainAngleROMrange = 30;
		private float secundaryAngle1ROMrange = 10;
		private float secundaryAngle2ROMrange = 10;
		private string mainAngleROMangle = "Y";
		private string secundaryAngle1ROMangle = "P";
		private string secundaryAngle2ROMangle = "R";
		private float timePerm = 1;
		private float thetaPerm = 5;


		public float MainAngleROMrange
		{
			get { return mainAngleROMrange; }
			set { mainAngleROMrange = value; }
		}
		public float SecundaryAngle1ROMrange
		{
			get { return secundaryAngle1ROMrange; }
			set { secundaryAngle1ROMrange = value; }
		}
		public float SecundaryAngle2ROMrange
		{
			get { return secundaryAngle2ROMrange; }
			set { secundaryAngle2ROMrange = value; }
		}
		public string MainAngleROMangle
		{
			get { return mainAngleROMangle; }
			set { mainAngleROMangle = value; }
		}
		public string SecundaryAngle1ROMangle
		{
			get { return secundaryAngle1ROMangle; }
			set { secundaryAngle1ROMangle = value; }
		}
		public string SecundaryAngle2ROMangle
		{
			get { return secundaryAngle2ROMangle; }
			set { secundaryAngle2ROMangle = value; }
		}
		public float TimePerm
		{
			get { return timePerm; }
			set { timePerm = value; }
		}
		public float ThetaPerm
		{
			get { return thetaPerm; }
			set { thetaPerm = value; }
		}
		public event IMUComandEventHandler ImuConfig;

		protected virtual void OnImuComand(IMUComandEventArgs e)
		{
			if (ImuConfig != null)
			{
				// Invokes the delegates. 
				ImuConfig(this, e);
			}
		}

		public void SendSetUp()
		{
			IMUComandEventArgs e = new IMUComandEventArgs(mainAngleROMrange, secundaryAngle1ROMrange, secundaryAngle2ROMrange,
				mainAngleROMangle, secundaryAngle1ROMangle, secundaryAngle2ROMangle,
				timePerm, thetaPerm);
			OnImuComand(e);
		}
	}
	#endregion

	#endregion






	#region INTERPLAY: evento imuReady

	#region clase generadora imuReady

	public class ImuReadyEventArgs : EventArgs
	{
		private readonly float alpha = 0;
		// Constructor.
		public ImuReadyEventArgs(float alpha)
		{
			this.alpha = alpha;
		}
		//Properties.
		public float Alpha { get { return alpha; } }
	}

	public delegate void ImuReadyEventHandler(object sender, ImuReadyEventArgs e);

	public class ImuReady
	{
		private float alpha = 0;

		public float Alpha
		{
			get { return alpha; }
			set { alpha = value; }
		}

		public event ImuReadyEventHandler ImuRea;

		protected virtual void OnImuReady(ImuReadyEventArgs e)
		{
			if (ImuRea != null)
			{
				// Invokes the delegates. 
				ImuRea(this, e);
			}
		}

		public void SendReady()
		{
			ImuReadyEventArgs e = new ImuReadyEventArgs(alpha);
			OnImuReady(e);
		}
	}

	#endregion

	#endregion

	#endregion
}

