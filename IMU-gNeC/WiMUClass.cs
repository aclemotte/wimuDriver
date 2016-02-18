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

		List<dataIMU> IMU = new List<dataIMU>();
		dataIMU imuAux = new dataIMU();
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

		float angle1 = 5;
		float angle2 = 5;

		bool VHRgameStarted = false;
		bool VHRgameConfigured = false;
		bool VHRgameInRange = true;

		List<float> dwellAngle = new List<float>();
		bool dwell = false;

		string iFeelLuckyPortSetup;
		int rotationSetup;

		bool workerImuReading;// flag to stop the thread
		float globalYaw, globalPitch, globalRoll;
		setupParameters ImuConfigPar = new setupParameters();
		orientacionIMUYPRArg orientacionIMUYPR; 



		public event orientacionIMUYPRHandler orientacionIMUYPRFired;
		public event EventHandler haVueltoAlRangoFired;
		public event EventHandler fueraDeRangoFired;
		public event seCumplePermanenciaHandler seCumplePermanenciaFired;
		public event EventHandler imuListoFired;
		public event orientaciónIMUArgHandler orientacionIMUFired;













		public bool connectIMU(string iFeelLuckyPort, int rotation)
        {
			//UnitySystemConsoleRedirector.Redirect();

            this.iFeelLuckyPortSetup = iFeelLuckyPort;
            this.rotationSetup = rotation;

            try
            {
				workerIMU = new Thread(threadReadingIMU);
				workerIMU.Name = "threadReadingIMU";            
                workerIMU.Start();
				return true;
            }
            catch (Exception e) 
			{ 
				Console.WriteLine(e.ToString());
				return false; 
			}
        }

		void threadReadingIMU()
		{
			Console.WriteLine ("Thread started");
			bool conectionOk = connectIMUkownCOM(iFeelLuckyPortSetup, rotationSetup);

			if (conectionOk) 
			{
				if (imuListoFired != null)
				{
					imuListoFired(this, EventArgs.Empty);
				}

				workerImuReading = true;
				//while (workerIMU.IsAlive) {
				//workerImuReading se cambia a false mediante el metodo publico
				while(workerImuReading)
				{
					readIMU();
				}
				Console.WriteLine ("Thread finished");

				Console.WriteLine ("Closing port");
				if (serialPort.IsOpen) 
				{
					serialPort.Close ();
					Console.WriteLine ("Port closed");
				} 
				else 
				{
					Console.WriteLine ("Port already closed");
				}
			}
			Console.WriteLine ("Thread finished");
		}

        bool connectIMUkownCOM(string iFeelLuckyPort, int rotation)
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
                




		//serial port methods


        bool setPort(string portName)
        {
            try
            {
				serialPort = new SerialPort(@"\\.\" + portName, 57600, Parity.None, 8, StopBits.One);
                return true;
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
                return false;
            }
        }

        bool openPort(string portName)
        {
			if (!serialPort.IsOpen) 
			{
				try 
				{
					Console.WriteLine ("Opening port");
					serialPort.Open ();
					Console.WriteLine ("Port opened");
					return true; // Success
				} 
				catch (Exception e) 
				{
					Console.WriteLine ("Port not opened");
					Console.WriteLine (e.ToString ());
					return false;
				}
			}
			else 
			{
				return false;
			}
        }










        void readIMU()
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
				dataIMU imuAuxA = new dataIMU();
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
                    or_init = functions.maximo(Z0, Z1, Z2);

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

                YPR = functions.calculate_YPR(or_init, angle[0], angle[1], angle[2]);
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
                        rotation = functions.maximo(rom1, rom2, rom3);

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


				orientacionIMUYPR = new orientacionIMUYPRArg();
				orientacionIMUYPR.yaw = globalYaw;
				orientacionIMUYPR.roll = globalRoll;
				orientacionIMUYPR.pitch = globalPitch;

				if(orientacionIMUYPRFired != null)
				{
					orientacionIMUYPRFired(orientacionIMUYPR);
				}

				orientacionIMUArg imuOrientacion = new orientacionIMUArg();

                // analiza el ángulo principal
                if (ImuConfigPar.mainAngleROMangle == "Y")
					imuOrientacion.mainAngle = globalYaw; //imuOR.MainAngle = globalYaw;
                else if (ImuConfigPar.mainAngleROMangle == "P")
					imuOrientacion.mainAngle = globalPitch; //imuOR.MainAngle = globalPitch;
                else
					imuOrientacion.mainAngle = globalRoll; //imuOR.MainAngle = globalRoll;

                // analiza el ángulo secundario 1
                if (ImuConfigPar.secundaryAngle1ROMangle == "Y")
                    angle1 = globalYaw;
                else if (ImuConfigPar.secundaryAngle1ROMangle == "P")
                    angle1 = globalPitch;
                else
                    angle1 = globalRoll;

                // analiza el ángulo secundario 2
                if (ImuConfigPar.secundaryAngle2ROMangle == "Y")
                    angle2 = globalYaw;
                else if (ImuConfigPar.secundaryAngle2ROMangle == "P")
                    angle2 = globalPitch;
                else
                    angle2 = globalRoll;

				VHRgameInRange = checkROM(angle1, angle2, imuOrientacion.mainAngle); //imuOR.MainAngle);
             
                if ((VHRgameStarted) && (VHRgameConfigured) && (VHRgameInRange))
                {
                    // envía el ángulo principal   
                    try
                    {
                        //imuOR.SendAngle()
						if (orientacionIMUFired != null)
						{
							orientacionIMUFired(imuOrientacion);
						}
                    }
                    catch (Exception e) { 
						Console.WriteLine(e.ToString());
					}

                    dwellAngle.Add(globalYaw);
					dwell = checkDwell(ImuConfigPar.thetaPerm, ImuConfigPar.timePerm, counter, imuOrientacion.mainAngle); //imuOR.MainAngle);
                    if (dwell)
                    {
                        //imuPerm.Alpha = globalYaw;
                        try
                        {
                            //imuPerm.SendPerm();
							if (seCumplePermanenciaFired != null)
							{
								seCumplePermanenciaArg eventArg = new seCumplePermanenciaArg();
								eventArg.Alpha = globalYaw;
								seCumplePermanenciaFired(eventArg);
							}
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





















		//segun doc api
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

		//segun doc api
		public void endGame()
		{
			VHRgameStarted = false;
			VHRgameConfigured = false;
		}

		//segun doc api
		public void initGame()
		{
			VHRgameStarted = true;
		}

		//segun doc api
		public void setup(setupParameters setupParametersArg)
		{
			//copiarClase
			ImuConfigPar.mainAngleROMrange = setupParametersArg.mainAngleROMrange;
			ImuConfigPar.secundaryAngle1ROMrange = setupParametersArg.secundaryAngle1ROMrange;
			ImuConfigPar.secundaryAngle2ROMrange = setupParametersArg.secundaryAngle2ROMrange;
			ImuConfigPar.mainAngleROMangle = setupParametersArg.mainAngleROMangle;
			ImuConfigPar.secundaryAngle1ROMangle = setupParametersArg.secundaryAngle1ROMangle;
			ImuConfigPar.secundaryAngle2ROMangle = setupParametersArg.secundaryAngle2ROMangle;
			ImuConfigPar.timePerm = setupParametersArg.timePerm;
			ImuConfigPar.thetaPerm = setupParametersArg.thetaPerm;

			VHRgameConfigured = true;
		}

		//segun doc api
		public void calibrate()
		{
			Calib=true;
		}

		//segun doc api
		public void rotate()
		{
			Rot++;
		}




























        bool checkROM(float angle1, float angle2, float mainAngle)
        {
            bool ok;

			if ((ImuConfigPar.secundaryAngle1ROMrange > Math.Abs(angle1)) && (ImuConfigPar.secundaryAngle2ROMrange > Math.Abs(angle2)))// && (2 * mainAngleROMrange > Math.Abs(mainAngle)))
            {
                ok = true;
                if (!VHRgameInRange)
                {
					if (haVueltoAlRangoFired != null)
					{
						haVueltoAlRangoFired(this, EventArgs.Empty);
					}

                }
            }
            else
            {
                ok = false;
                if (VHRgameInRange)
                {
					if (fueraDeRangoFired != null)
					{
						fueraDeRangoFired(this, EventArgs.Empty);
					}
                }
            }         
            return ok;
        }
        
        bool checkDwell(float anglePerm, float time, int counter, float mainAngle)
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

        bool check_click_intention(int t_permanencia, float anglePerm, float mainAngle)
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
				if (ImuConfigPar.mainAngleROMrange > Math.Abs(mainAngle))
                    clic_flag = true;
                dwellAngle.Clear();
            }
            else
            {
                clic_flag = false;
            }
            return clic_flag;
        }       
    }

}