using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using System.Threading;
using CSML;


namespace IMU_gNeC
{
	public class IMUClass
	{        
		static SerialPort serialPort;
		Thread workerIMU;

		List<anglesIMU> IMU = new List<anglesIMU>();
		anglesIMU imuAux = new anglesIMU();
		Matrix RCAL = new Matrix(new double[,] { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } });
		static Single Yaw0;
		static Single Roll0;
		static Single Pitch0;
		int or_init = 0;
		float rom1, rom2, rom3;
		int counter = 0;
		int rotation;
		bool Calib = true;
		int Rot = 0;
		List<float> rot1 = new List<float>();
		List<float> rot2 = new List<float>();
		List<float> rot3 = new List<float>();

		float mainAngleROMrange = 30;
		float secundaryAngle1ROMrange = 10;
		float secundaryAngle2ROMrange = 10;
		float timePerm = 1;
		float thetaPerm = 5;

		float angle1 = 5;
		float angle2 = 5;

		bool VHRgameStarted = false;
		bool VHRgameConfigured = false;
		bool VHRgameInRange = true;

		List<float> dwellAngle = new List<float>();
		bool dwell = false;

		string iFeelLuckyPortSetup;
		int rotationSetup;

		bool workerImuReading;

		public float globalYaw, globalPitch, globalRoll;






		// ImuYPR: Instantiates the event receiver.
		//private readImuYPR rYPR = new readImuYPR();


		// ImuYPR: Instantiates the event source.
		public IMUAngle IMUAng = new IMUAngle();
		
		// Instantiates the event source.
		public IMUComand ImuConfigPar = new IMUComand();

		// InitEnd: Instantiates the event source.
		public game ImuInitgame = new game();

		// ImuROM: Instantiates the event source.
		public ImuROMChanged imuROM = new ImuROMChanged();

		// ImuOR: Instantiates the event source.
		public ImuOrientation imuOR = new ImuOrientation();

		public ImuReady imuReady = new ImuReady();





		// ImuPerm: Instantiates the event source.
		public ImuPermanencia imuPerm = new ImuPermanencia();
















		//verificado
		public bool startReadingIMU(string iFeelLuckyPort, int rotation)
		{
			//UnitySystemConsoleRedirector.Redirect();

			this.iFeelLuckyPortSetup = iFeelLuckyPort;
			this.rotationSetup = rotation;

			try
			{
				workerIMU = new Thread(threadReadingIMU);
				workerIMU.Name = "threadReadingIMU";            
				workerIMU.Start();
			}
			catch (Exception e) 
			{ 
				Console.WriteLine(e.ToString());
				return false; 
			}

			try
			{
				//IMUAng.ImuYPR += new IMUEventHandler(rYPR.IMUrecieved);
				ImuInitgame.InitEndGame += new InitEndGameEventHandler(InitEndGamerecieved);
				ImuConfigPar.ImuConfig += new IMUComandEventHandler(IMUComandrecieved);
				return true;
			}
			catch (Exception e) 
			{
				Console.WriteLine(e.ToString());
				return false; 
			}
			//}
		}

		//verificado
		private void threadReadingIMU()
		{
			bool conectionOk = connectIMUkownCOM(iFeelLuckyPortSetup, rotationSetup);

			if (conectionOk) 
			{
				workerImuReading = true;
				//while (workerIMU.IsAlive) {
				//workerImuReading se cambia a false mediante el metodo publico
				while(workerImuReading)
				{
					readIMU();
				}
				if(serialPort.IsOpen)
					serialPort.Close();
			}
		}

		//verificado
		private bool connectIMUkownCOM(string iFeelLuckyPort, int rotation)
		{
			bool setPortOk = setPort(iFeelLuckyPort);

			if (setPortOk) 
			{
				bool openPortOk = openPort(iFeelLuckyPort);

				if (openPortOk) 
				{	
					serialPort.WriteLine("#om");
					return true;
				} 
				else 
				{
					return false;
				}
			} 
			else 
			{
				return false;
			}
		}      			
				
				










		//verificado
		private bool setPort(string portName)
		{
			try
			{
				serialPort = new SerialPort(@"\\.\" + portName, 57600, Parity.None, 8, StopBits.One);
//				serialPort.ReadTimeout = 500;
//				serialPort.WriteTimeout = 500;
//				serialPort.Handshake = Handshake.None;
				return true;
			}
			catch (Exception e)
			{
				Console.WriteLine(e.ToString());
				return false;
			}
		}

		//verificado
		private bool openPort(string portName)
		{
			if (!serialPort.IsOpen) 
			{
				try 
				{
					serialPort.Open ();
					Console.WriteLine ("Port opened");
					return true; // Success
				} 
				catch (Exception e) 
				{
					Console.WriteLine (e.ToString ());
					return false;
				}
			}
			else 
			{
				return false;
			}
		}










		private void readIMU()
		{
			#region recibe datos del IMU

			string[] words = { "0", "0", "0", "0", "0", "0", "0" };
			bool first_char;//, last_char;

			try
			{
				String message = null;
				do
				{
					try
					{
						message = serialPort.ReadLine();

					}
					catch (Exception e) 
					{ 
						Console.WriteLine(e.ToString()); 
						return; 
					}

					first_char = message.StartsWith("#");
					//last_char = message.EndsWith("\r");

				} while (!first_char);// || !last_char);


				words = message.Split('=');

				string[] words2 = words[1].Split(',');
				Single[] angle = { 0, 0, 0 };
				Single[] YPR = { 0, 0, 0 };

				Single yaw = 0, roll = 0, pitch = 0;

				double X0, X1, X2, Y0, Y1, Y2, Z0, Z1, Z2;
				anglesIMU imuAuxA = new anglesIMU();
				X0 = 0; X1 = 0; X2 = 0; Y0 = 0; Y1 = 0; Y2 = 0; Z0 = 0; Z1 = 0; Z2 = 0;

				if (words[0] == "#DCM")
				{
					// DCM
					X0 = Convert.ToSingle(words2[0]);
					X1 = Convert.ToSingle(words2[1]);
					X2 = Convert.ToSingle(words2[2]);

					Y0 = Convert.ToSingle(words2[3]);
					Y1 = Convert.ToSingle(words2[4]);
					Y2 = Convert.ToSingle(words2[5]);

					Z0 = Convert.ToSingle(words2[6]);
					Z1 = Convert.ToSingle(words2[7]);
					Z2 = Convert.ToSingle(words2[8]);
				}
				Matrix DCM_A = new Matrix(new double[,] { { X0, X1, X2 }, { Y0, Y1, Y2 }, { Z0, Z1, Z2 } });

				imuAuxA.calibrationTimeStamp = false;

				if (Calib == true)
				{
					Calib = false;
					imuAuxA.calibrationTimeStamp = true;
					// RCAL
					RCAL = new Matrix(new double[,] { { X0, X1, X2 }, { Y0, Y1, Y2 }, { Z0, Z1, Z2 } });

					// Calcular el máximo Z0, Z1 y Z2
					or_init = maximo(Z0, Z1, Z2);

					if (or_init == 1)
					{
						if (Z0 < 0) or_init = 1; else or_init = 2;
					}
					else if (or_init == 2)
					{
						if (Z1 < 0) or_init = 3; else or_init = 4;
					}
					else if (or_init == 3)
					{
						if (Z2 < 0) or_init = 5; else or_init = 6;
					}

					// EULER
					Yaw0 = angle[0];
					Roll0 = angle[1];
					Pitch0 = angle[2];
				}

				Matrix RT_A;
				RT_A = (DCM_A.Transpose()) * RCAL;

				angle[0] = Convert.ToSingle(Math.Atan2(RT_A[2, 1].Re, RT_A[1, 1].Re)); // YAW                         
				angle[1] = Convert.ToSingle(Math.Atan2(RT_A[3, 2].Re, RT_A[3, 3].Re)); // ROLL

				if (RT_A[3, 1].Re < -1)
					RT_A[3, 1].Re = -1;
				if (RT_A[3, 1].Re > 1)
					RT_A[3, 1].Re = 1;
				angle[2] = Convert.ToSingle(-Math.Asin(RT_A[3, 1].Re)); // PITCH

				YPR = calculate_YPR(or_init, angle[0], angle[1], angle[2]);
				//YPR[2] = Convert.ToSingle(gravity); // Esto es un apaño para la silla de ruedas

				yaw = YPR[0];
				pitch = YPR[1];
				roll = YPR[2];

			#endregion

				#region detecta nuevo eje sagital de rotacion

				if ((Rot > 0) && (Rot < 2))
				{
					rot1.Add(yaw);
					rot2.Add(pitch);
					rot3.Add(roll);

					if (Rot == 1)
					{
						rom1 = rot1[0] + rot1[rot1.Count - 1];
						rom2 = rot2[0] + rot2[rot2.Count - 1];
						rom3 = rot3[0] + rot3[rot3.Count - 1];
						rotation = maximo(rom1, rom2, rom3);

						Rot = 0;
						rot1.Clear();
						rot2.Clear();
						rot3.Clear();
					}
				}
				#endregion

				#region reposiciona ángulos; switch orientación (6x4)


				switch (or_init)
				{
					case 1:

						#region or_init = 1, switch rotation

						switch (rotation)
						{
							case 2:
								if (rom2 > 0)
								{
									globalYaw = yaw;
									globalPitch = pitch;
									globalRoll = -roll;
								}
								else
								{
									globalYaw = yaw;
									globalPitch = -pitch;
									globalRoll = roll;
								}
								break;

							case 3:
								if (rom3 > 0)
								{
									globalYaw = yaw;
									globalPitch = roll;
									globalRoll = pitch;
								}
								else
								{
									globalYaw = yaw;
									globalPitch = -roll;
									globalRoll = -pitch;
								}
								break;

							default:
								globalYaw = yaw;
								globalPitch = roll;
								globalRoll = pitch;
								break;
						}

						#endregion

						//WriteTextBox(textBox_A2, Convert.ToString((int)globalPitch_A));
						//WriteTextBox(textBox_A3, Convert.ToString((int)globalYaw_A));
						//WriteTextBox(textBox_A1, Convert.ToString((int)globalRoll_A));
						break;

					case 2:

						#region or_init = 2, switch rotation

						switch (rotation)
						{
							case 2:
								if (rom2 > 0)
								{
									globalYaw = yaw;
									globalPitch = pitch;
									globalRoll = -roll;
								}
								else
								{
									globalYaw = yaw;
									globalPitch = -pitch;
									globalRoll = roll;
								}
								break;

							case 3:
								if (rom3 > 0)
								{
									globalYaw = yaw;
									globalPitch = roll;
									globalRoll = pitch;
								}
								else
								{
									globalYaw = yaw;
									globalPitch = -roll;
									globalRoll = -pitch;
								}
								break;

							default:
								globalYaw = -yaw;
								globalPitch = roll;
								globalRoll = -pitch;
								break;
						}
						#endregion

						//WriteTextBox(textBox_A2, Convert.ToString((int)globalPitch_A));
						//WriteTextBox(textBox_A3, Convert.ToString((int)globalYaw_A));
						//WriteTextBox(textBox_A1, Convert.ToString((int)globalRoll_A));
						break;

					case 3:

						#region or_init = 3, switch rotation

						switch (rotation)
						{
							case 2:
								if (rom2 > 0)
								{
									globalYaw = yaw;
									globalPitch = pitch;
									globalRoll = roll;
								}
								else
								{
									globalYaw = yaw;
									globalPitch = -pitch;
									globalRoll = -roll;
								}
								break;
							case 3:
								if (rom3 > 0)
								{
									globalYaw = yaw;
									globalPitch = roll;
									globalRoll = -pitch;
								}
								else
								{
									globalYaw = yaw;
									globalPitch = -roll;
									globalRoll = pitch;
								}
								break;
							default:
								globalYaw = yaw;
								globalPitch = roll;
								globalRoll = pitch;
								break;
						}
						#endregion

						//WriteTextBox(textBox_A2, Convert.ToString((int)globalPitch_A));
						//WriteTextBox(textBox_A3, Convert.ToString((int)globalYaw_A));
						//WriteTextBox(textBox_A1, Convert.ToString((int)globalRoll_A));
						break;

					case 4:

						#region or_init = 4, switch rotation

						switch (rotation)
						{
							case 2:
								if (rom2 > 0)
								{
									globalYaw = yaw;
									globalPitch = pitch;
									globalRoll = roll;
								}
								else
								{
									globalYaw = yaw;
									globalPitch = -pitch;
									globalRoll = -roll;
								}
								break;
							case 3:
								if (rom3 > 0)
								{
									globalYaw = yaw;
									globalPitch = roll;
									globalRoll = -pitch;
								}
								else
								{
									globalYaw = yaw;
									globalPitch = -roll;
									globalRoll = pitch;
								}
								break;

							default:
								globalYaw = yaw;
								globalPitch = roll;
								globalRoll = pitch;
								break;
						}
						#endregion

						//WriteTextBox(textBox_A2, Convert.ToString((int)globalPitch_A));
						//WriteTextBox(textBox_A3, Convert.ToString((int)globalYaw_A));
						//WriteTextBox(textBox_A1, Convert.ToString((int)globalRoll_A));
						break;

					case 6:

						#region or_init = 6, , switch rotation

						switch (rotation)
						{
							case 2:
								if (rom2 > 0)
								{
									globalYaw = yaw;
									globalPitch = pitch;
									globalRoll = roll;
								}
								else
								{
									globalYaw = yaw;
									globalPitch = -pitch;
									globalRoll = -roll;
								}
								break;

							case 3:
								if (rom3 > 0)
								{
									globalYaw = yaw;
									globalPitch = roll;
									globalRoll = -pitch;
								}
								else
								{
									globalYaw = yaw;
									globalPitch = -roll;
									globalRoll = pitch;
								}
								break;

							default:
								globalYaw = -yaw;
								globalPitch = -roll;
								globalRoll = -pitch;
								break;
						}

						#endregion

						//WriteTextBox(textBox_A2, Convert.ToString((int)globalPitch_A));
						//WriteTextBox(textBox_A3, Convert.ToString((int)globalYaw_A));
						//WriteTextBox(textBox_A1, Convert.ToString((int)globalRoll_A));
						break;

					default: //or_init == 5

						#region or_init = 5, switch rotation

						switch (rotation)
						{
							case 2:
								if (rom2 > 0)
								{
									globalYaw = yaw;
									globalPitch = pitch;
									globalRoll = -roll;
								}
								else
								{
									globalYaw = yaw;
									globalPitch = -pitch;
									globalRoll = roll;
								}
								break;
							case 3:
								if (rom3 > 0)
								{
									globalYaw = yaw;
									globalPitch = roll;
									globalRoll = pitch;
								}
								else
								{
									globalYaw = yaw;
									globalPitch = -roll;
									globalRoll = -pitch;
								}
								break;
							default:
								globalYaw = yaw;
								globalPitch = roll;
								globalRoll = pitch;
								break;
						}
						#endregion

						//WriteTextBox(textBox_A2, Convert.ToString((int)globalPitch_A));
						//WriteTextBox(textBox_A3, Convert.ToString((int)globalYaw_A));
						//WriteTextBox(textBox_A1, Convert.ToString((int)globalRoll_A));
						break;
				}

				#endregion
										 
				long unixTimestamp = (long)(DateTime.UtcNow.Subtract(new DateTime(1970, 1, 1))).TotalMilliseconds;
				//WriteTextBox(textBox_tA, Convert.ToString(unixTimestamp));

				imuAuxA.pitch = globalPitch;
				imuAuxA.yaw = globalYaw;
				imuAuxA.roll = globalRoll;
				imuAuxA.timeStamp = Convert.ToString(unixTimestamp);
				imuAuxA.X0 = words2[0];
				imuAuxA.X1 = words2[1];
				imuAuxA.X2 = words2[2];
				imuAuxA.Y0 = words2[3];
				imuAuxA.Y1 = words2[4];
				imuAuxA.Y2 = words2[5];
				imuAuxA.Z0 = words2[6];
				imuAuxA.Z1 = words2[7];
				imuAuxA.Z2 = words2[8];
				IMU.Add(imuAux);

				counter++;

				IMUAng.Yaw = globalYaw;
				IMUAng.Pitch = globalPitch;
				IMUAng.Roll = globalRoll;
				IMUAng.SendAngle();

				// analiza el ángulo principal
				if (ImuConfigPar.MainAngleROMangle == "Y")
					imuOR.MainAngle = globalYaw;
				else if (ImuConfigPar.MainAngleROMangle == "P")
					imuOR.MainAngle = globalPitch;
				else
					imuOR.MainAngle = globalRoll;

				// analiza el ángulo secundario 1
				if (ImuConfigPar.SecundaryAngle1ROMangle == "Y")
					angle1 = globalYaw;
				else if (ImuConfigPar.SecundaryAngle1ROMangle == "P")
					angle1 = globalPitch;
				else
					angle1 = globalRoll;

				// analiza el ángulo secundario 2
				if (ImuConfigPar.SecundaryAngle2ROMangle == "Y")
					angle2 = globalYaw;
				else if (ImuConfigPar.SecundaryAngle2ROMangle == "P")
					angle2 = globalPitch;
				else
					angle2 = globalRoll;

				VHRgameInRange = checkROM(angle1, angle2, imuOR.MainAngle);
			 
				if ((VHRgameStarted) && (VHRgameConfigured) && (VHRgameInRange))
				{
					// envía el ángulo principal   
					try
					{
						imuOR.SendAngle();
					}
					catch (Exception e) { 
						Console.WriteLine(e.ToString());
					}

					dwellAngle.Add(globalYaw);
					dwell = checkDwell(thetaPerm, timePerm, counter, imuOR.MainAngle);
					if (dwell)
					{
						imuPerm.Alpha = globalYaw;
						try
						{
							imuPerm.SendPerm();
						}
						catch (Exception e) { 
							Console.WriteLine(e.ToString());
						}
					}
				}
			}
			catch (Exception e) { 
				Console.WriteLine(e.ToString());
			}
		}






















		public void disconnectIMU()
		{
			try
			{
				workerImuReading = false;
			}
			catch (Exception e)
			{ 
				Console.WriteLine (e.ToString ());
			}
		}

		public void IMUComandrecieved(object sender, IMUComandEventArgs e)
		{
			if (e.Calibrate)
				Calib = true;
			if (e.Rotate)
				Rot++;
			if (e.SetUp)
			{
				this.mainAngleROMrange = e.MainAngleROMrange;
				this.secundaryAngle1ROMrange = e.SecundaryAngle1ROMrange;
				this.secundaryAngle2ROMrange = e.SecundaryAngle2ROMrange;
				this.timePerm = e.TimePerm;
				this.thetaPerm = e.ThetaPerm;
				VHRgameConfigured = true;
			}
		}

		public void InitEndGamerecieved(object sender, InitEndGameEventArgs e)
		{
			if (e.InitGame)
			{
				VHRgameStarted = true;
			}
			else
			{
				VHRgameStarted = false;
				VHRgameConfigured = false;
			}
		}

		public bool checkROM(float angle1, float angle2, float mainAngle)
		{
			bool ok;

			if ((secundaryAngle1ROMrange > Math.Abs(angle1)) && (secundaryAngle2ROMrange > Math.Abs(angle2)))// && (2 * mainAngleROMrange > Math.Abs(mainAngle)))
			{
				ok = true;
				if (!VHRgameInRange)
				{
					imuROM.MainAngle = mainAngle;
					imuROM.RomOK = ok;
					imuROM.SendROM();
				}
			}
			else
			{
				ok = false;
				if (VHRgameInRange)
				{
					imuROM.MainAngle = mainAngle;
					imuROM.RomOK = ok;
					imuROM.SendROM();
				}
			}         
			return ok;
		}
		
		public bool checkDwell(float anglePerm, float time, int counter, float mainAngle)
		{
			bool clic;
			int t_permanencia = (int)(time * 50); // T_permanencia en samples
			clic = false;

			if (dwellAngle.Count > t_permanencia - 1)
			{
				try
				{
					clic = check_click_intention(t_permanencia, anglePerm, mainAngle);
				}
				catch 
				{ 
					//Console.WriteLine("ERROR FATAL 1"); 
				}
			}

			return clic;
		}

		public bool check_click_intention(int t_permanencia, float anglePerm, float mainAngle)
		{
			float std_intention_theta;
			bool clic_flag = false;

			// Calculate standard deviation x
			float std_suma = 0;
			float std_media = 0;


			// Cálculo de la media de los datos
			for (int j = 0; j < t_permanencia; j++)
			{
				std_media = (std_media + dwellAngle[dwellAngle.Count-1 - j]);
			}
			std_media = std_media / t_permanencia;

			// Cada muestra menos su la media al cuadrado
			for (int j = 0; j < t_permanencia; j++)
			{
				std_suma = std_suma + ((dwellAngle[dwellAngle.Count - 1 - j] - std_media) * (dwellAngle[dwellAngle.Count - 1 - j] - std_media));
			}

			std_intention_theta = (float)Math.Sqrt(std_suma / t_permanencia); // Standard deviation coordinate x
			int theta_intention = Convert.ToInt16(std_media);

			if (std_intention_theta < anglePerm)
			{
				if (mainAngleROMrange > Math.Abs(mainAngle))
					clic_flag = true;
				dwellAngle.Clear();
			}
			else
			{
				clic_flag = false;
			}
			return clic_flag;
		}
		
		public int maximo(double a, double b, double c)
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

		public Single[] calculate_YPR(int or_init, Single alfa, Single beta, Single gamma)
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

	#region INTERPLAY: inicio juego

		#region clase generadora InitEndGame

		public class InitEndGameEventArgs : EventArgs
		{
			private readonly bool initGame = false;
			// Constructor.
			public InitEndGameEventArgs(bool initGame)
			{
				this.initGame = initGame;
			}
			//Properties.
			public bool InitGame { get { return initGame; } }
		}

		public delegate void InitEndGameEventHandler(object sender, InitEndGameEventArgs e);

		public class game
		{
			private bool initGame = false;

			public bool InitGame
			{
				get { return initGame; }
				set { initGame = value; }
			}

			public event InitEndGameEventHandler InitEndGame;

			protected virtual void OnInitEndGame(InitEndGameEventArgs e)
			{
				if (InitEndGame != null)
				{
					// Invokes the delegates. 
					InitEndGame(this, e);
				}
			}

			public void SendInitEndGame()
			{
				InitEndGameEventArgs e = new InitEndGameEventArgs(initGame);
				OnInitEndGame(e);
			}
		}

		#endregion

	#endregion

	#region INTERPLAY: parámetros configuracion

		#region clase generadora parámetros configuracion

		public class IMUComandEventArgs : EventArgs
		{
			private readonly bool calibrate = false;
			private readonly bool rotate = false;
			private readonly bool setUp = false;
			private readonly float mainAngleROMrange = 30;
			private readonly float secundaryAngle1ROMrange = 10;
			private readonly float secundaryAngle2ROMrange = 10;
			private readonly string mainAngleROMangle = "Y";
			private readonly string secundaryAngle1ROMangle = "P";
			private readonly string secundaryAngle2ROMangle = "R";
			private readonly float timePerm = 1;
			private readonly float thetaPerm = 5;

			// Constructor.
			public IMUComandEventArgs(bool calibrate, bool rotate, bool setUp,
									float mainAngleROMrange, float secundaryAngle1ROMrange, float secundaryAngle2ROMrange,
									string mainAngleROMangle, string secundaryAngle1ROMangle, string secundaryAngle2ROMangle, 
									float timePerm, float thetaPerm)
			{
				this.calibrate = calibrate;
				this.rotate = rotate;
				this.setUp = setUp;
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
			public bool Calibrate { get { return calibrate; } }
			public bool Rotate { get { return rotate; } }
			public bool SetUp { get { return setUp; } }
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
			private bool calibrate = false;
			private bool rotate = false;
			private bool setUp = false;
			private float mainAngleROMrange = 30;
			private float secundaryAngle1ROMrange = 10;
			private float secundaryAngle2ROMrange = 10;
			private string mainAngleROMangle = "Y";
			private string secundaryAngle1ROMangle = "P";
			private string secundaryAngle2ROMangle = "R";
			private float timePerm = 1;
			private float thetaPerm = 5;

			public bool Calibrate
			{
				get { return calibrate; }
				set { calibrate = value; }
			}
			public bool Rotate
			{
				get { return rotate; }
				set { rotate = value; }
			}
			public bool SetUp
			{
				get { return setUp; }
				set { setUp = value; }
			}
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
				IMUComandEventArgs e = new IMUComandEventArgs(calibrate, rotate, setUp, 
															  mainAngleROMrange, secundaryAngle1ROMrange, secundaryAngle2ROMrange,
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