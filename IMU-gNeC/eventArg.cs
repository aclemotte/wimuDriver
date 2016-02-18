using System;

namespace IMU_gNeC
{
	public delegate void seCumplePermanenciaHandler(seCumplePermanenciaArg e);
	public class seCumplePermanenciaArg : EventArgs
	{
		public float Alpha;
	}



	public delegate void orientaciónIMUArgHandler(orientacionIMUArg e);
	public class orientacionIMUArg: EventArgs
	{
		public float mainAngle = 0;
	}
}

