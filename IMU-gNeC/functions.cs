using System;

namespace IMU_gNeC
{
	public static class functions
	{
		public static int maximo(double a, double b, double c)
		{

			a = Math.Abs(a);
			b = Math.Abs(b);
			c = Math.Abs(c);

			if (a > b)
			{
				if (a > c)
					return 1;
				else
					return 3;
			}
			else
			{
				if (b > c)
					return 2;
				else
					return 3;
			}
		}

		public static Single[] calculate_YPR(int or_init, Single alfa, Single beta, Single gamma)
		{

			Single[] YPR = { 0, 0, 0 };

			alfa = alfa * Convert.ToSingle(180 / Math.PI);
			beta = beta * Convert.ToSingle(180 / Math.PI);
			gamma = gamma * Convert.ToSingle(180 / Math.PI);

			switch (or_init)
			{
			case 1:
				YPR[0] = beta; // YAW
				YPR[1] = gamma; // PITCH
				YPR[2] = alfa;  // ROLL

				break;
			case 2:
				YPR[0] = -beta; // YAW
				YPR[1] = -gamma; // PITCH
				YPR[2] = alfa;  // ROLL

				break;
			case 3:
				YPR[0] = gamma; // YAW
				YPR[1] = beta; // PITCH
				YPR[2] = alfa;  // ROLL


				break;
			case 4:
				YPR[0] = -gamma; // YAW
				YPR[1] = -beta; // PITCH
				YPR[2] = alfa;  // ROLL



				break;
			case 5:
				YPR[0] = alfa; // YAW
				YPR[1] = beta; // PITCH
				YPR[2] = gamma;  // ROLL




				break;
			case 6:
				YPR[0] = -alfa; // YAW
				YPR[1] = -beta; // PITCH
				YPR[2] = -gamma;  // ROLL


				break;
			}

			return YPR;
		}
	}
}

