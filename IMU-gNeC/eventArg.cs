using System;

namespace IMU_gNeC
{
	public delegate void seCumplePermanenciaHandler(seCumplePermanenciaArg e);

	public class seCumplePermanenciaArg : EventArgs
	{
		public float Alpha;
	}
}

