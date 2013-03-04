using System;
using System.Threading;
using Kinova.API.Jaco;
using Kinova.API.Jaco.Configurations;
using Kinova.API.Jaco.Control.Trajectory;
using Kinova.DLL.Data.Jaco;
using Kinova.DLL.Data.Jaco.Config;
using Kinova.DLL.Data.Util;
using Kinova.DLL.SafeGate;
using Kinova.API.Jaco.Diagnostic;
using Kinova.DLL.Data.Jaco.Diagnostic;

namespace JacoAPIWrapper
{
    /// \brief A Jaco joint state structure.
    public struct JacoJointState
    {
        public double angle;
        public double velocity;
        public double effort;
    }

    /// \brief Basic POD struct to store a Jaco Arm state.
    /// 
    /// NOTE: Dummy fields for now.
    /// NOTE: We're not using arrays for unboxing issues on the native side.
    /// Fixed buffers are an option, but implies unsafe sections.
    public struct JacoArmState
    {
		//Joints information
        public JacoJointState shoulder_yaw;
        public JacoJointState shoulder_pitch;
        public JacoJointState elbow_pitch;
        public JacoJointState elbow_roll;
        public JacoJointState wrist_roll;
        public JacoJointState hand_roll;
		
		//Finger information
		public JacoJointState finger_1;
		public JacoJointState finger_2;
		public JacoJointState finger_3;		
		

		//Hand information
        public double hand_position_x;
        public double hand_position_y;
        public double hand_position_z;
        public double hand_orientation_x;
        public double hand_orientation_y;
        public double hand_orientation_z;
		
	
    }
	

	

    /// \brief A structure containing a joint-space command.
    public struct JacoJointCommand
    {
        public double shoulder_yaw;
        public double shoulder_pitch;
        public double elbow_pitch;
        public double elbow_roll;
        public double wrist_roll;
        public double hand_roll;
    }

    /// \brief A geometric pose in X,Y,Z and Euler angles X,Y,Z.
    public struct JacoPose
    {
        public double x;
        public double y;
        public double z;
        public double e_x;
        public double e_y;
        public double e_z;
    }

    /// \brief Jaco Arm API wrapper class.
    /// 
    /// This class acts as a minimalist wrapper to simplify interop from C code
    /// managing the Mono runtime.
    public class JacoArm
    {
        public JacoArm(string key)
        {
            m_IsEnabled = false;
            m_State = new JacoArmState();
            m_Cmd = new CJoystickValue();
            
            try 
            {
                CCypherMessage pass = Crypto.GetInstance().Encrypt(key);
                m_Arm = new CJacoArm(pass);
  
                
                Console.WriteLine("Wrapper initialized.");
                if (m_Arm.JacoIsReady())
                {
                    m_Arm.ControlManager.StartControlAPI();
                    // Init sequence, place the arm in READY position.
                    InitSequence();

                    Console.WriteLine("The arm is ready.");
                    m_IsEnabled = true;
                }
                else
                {
                    Console.WriteLine("The arm is not ready!");
                    m_IsEnabled = false;
                }
            } catch (Exception e)
            {
                System.Console.WriteLine(
                    "JACO API initialisation failed. Reason: ");
                System.Console.WriteLine(e.ToString());
                m_IsEnabled = false;
            }

            // 200 ms delay.
            m_WatchDogDelay = new TimeSpan(0,0,0,0,200);

        }

        ~JacoArm()
        {
            m_Arm.ControlManager.StopControlAPI();
        }

        public bool IsEnabled()
        {
            return m_IsEnabled;
        }

        public JacoArmState GetState()
        {
            return m_State;
        }

        /// \brief Update the internal state, send commands.
        public void Update()
        {
                    			      
            CVectorAngle jp = m_Arm.ConfigurationsManager.GetJointPositions();
            CVectorEuler hand = m_Arm.ConfigurationsManager.GetHandPosition();
								
			
            // See the DH specs for angle conversions.
            m_State.shoulder_yaw.angle = 
                (180.0 - jp.Angle[0]) / (180.0 / Math.PI);
            m_State.shoulder_pitch.angle = 
                (jp.Angle[1] - 270.0) / (180.0 / Math.PI);
            m_State.elbow_pitch.angle = 
                (90.0 - jp.Angle[2]) / (180.0 / Math.PI);
            m_State.elbow_roll.angle = 
                (180.0 - jp.Angle[3]) / (180.0 / Math.PI);
            m_State.wrist_roll.angle = 
                (180.0 - jp.Angle[4]) / (180.0 / Math.PI);
            m_State.hand_roll.angle = 
                (260.0 - jp.Angle[5]) / (180.0 / Math.PI);
			
			
			//Hand pose & orientation
            m_State.hand_position_x = hand.Position[0];
            m_State.hand_position_y = hand.Position[1];
            m_State.hand_position_z = hand.Position[2];
            m_State.hand_orientation_x = hand.Rotation[0];
            m_State.hand_orientation_y = hand.Rotation[1];
            m_State.hand_orientation_z = hand.Rotation[2];
			
						
			//Update finger information (slow)
			float[] fingerPos = m_Arm.DiagnosticManager.DataManager.GetPositionLogLiveFromJaco().UserPosition.FingerPosition;

			//Update finger angles
			m_State.finger_1.angle = fingerPos[0] * Math.PI / 180.0;
			m_State.finger_2.angle = fingerPos[1] * Math.PI / 180.0;
			m_State.finger_3.angle = fingerPos[2] * Math.PI / 180.0;
			
			
            //if (m_IsRetracting && ((DateTime.Now - m_LastCmd) > m_RetractDelay))
			if(m_IsRetracting)
            {
				try
	            {   
					
					//Getting position Live Only when retracting and wait for 
					CPosition positionLive = new CPosition();
	                positionLive = m_Arm.DiagnosticManager.DataManager.GetPositionLogLiveFromJaco();
	                
	                //System.Console.WriteLine("Retract status ");
	                //System.Console.WriteLine(positionLive.SystemStatus.RetractStatus);
					
					/*					  					
						Mode_Normal_To_Ready	
						Mode_Ready_Standby	
						Mode_Ready_To_Retract	
						Mode_Retract_Standby	
						Mode_Retract_To_Ready	
						Mode_Normal	
						Mode_NoInit_To_Ready	
					    ERROR
					*/	
					
					
					//Are we done retracting 
					if (
                        positionLive.SystemStatus.RetractStatus == 1 ||
                        positionLive.SystemStatus.RetractStatus == 3)
					{
						m_IsRetracting = false;
						// Stop the retract/reset command.
               			m_Cmd.ButtonValue[2] = 0;
                		m_Arm.ControlManager.SendJoystickFunctionnality(m_Cmd); 
						m_LastCmd = DateTime.Now;
					}
					
					
					else
					{
						//Look for timeout
						if ((DateTime.Now - m_LastCmd) > m_RetractDelay)
						{
							System.Console.WriteLine("Retract delay expired");
							m_IsRetracting = false;
							// Stop the retract/reset command.
	               			m_Cmd.ButtonValue[2] = 0;
	                		m_Arm.ControlManager.SendJoystickFunctionnality(m_Cmd); 
							m_LastCmd = DateTime.Now;
						}
					}
					
	                
	            }
	            catch (Exception e)
	            {
	                System.Console.WriteLine(
	                    "JACO API diagnostic API Failed : ");
	                System.Console.WriteLine(e.ToString());
					m_IsRetracting = false;
					// Stop the retract/reset command.
           			m_Cmd.ButtonValue[2] = 0;
            		m_Arm.ControlManager.SendJoystickFunctionnality(m_Cmd); 
					m_LastCmd = DateTime.Now;
	            }
				
				

                return;
            }

			
            // 200 ms watchdog, reset the command if it's too old.
            if ((DateTime.Now - m_LastCmd) > m_WatchDogDelay)
                m_Cmd = new CJoystickValue();
                
                
                
                
            // NOTE: Disabled for now, testing trajectory mode.
            //m_Arm.ControlManager.SendJoystickFunctionnality(m_Cmd);


        }
		
		
		public void SendJointsAngleRads(double j0, double j1, double j2, double j3, double j4, double j5)
		{
			if (m_Arm.JacoIsReady())
			{
				
				try 
				{
					CPointsTrajectory pointsTrajectory = new CPointsTrajectory();
					CVectorAngle vector = new CVectorAngle();
					
					//Change into degrees
					j0 *= (180.0 / Math.PI);
					j1 *= (180.0 / Math.PI);
					j2 *= (180.0 / Math.PI);
					j3 *= (180.0 / Math.PI);
					j4 *= (180.0 / Math.PI);
					j5 *= (180.0 / Math.PI);
					
					//Angle conversion for Jaco according to DH model
				    vector.Angle = new Single[6] {
						Convert.ToSingle(-j0 + 180),
						Convert.ToSingle( j1 + 270),
						Convert.ToSingle(-j2 + 90 ),
						Convert.ToSingle(-j3 + 180),
						Convert.ToSingle(-j4 + 180),
						Convert.ToSingle(-j5 + 260)};
					
					//This is weird, do we have to set this?
					vector.NbAngle = 6;
					
					//Send trajectory
					CTrajectoryInfo trajectory = new CTrajectoryInfo();
					trajectory.UserPosition.AnglesJoints = vector;
					trajectory.UserPosition.PositionType = CJacoStructures.PositionType.AngularPosition;									
					pointsTrajectory.Add(trajectory);
					m_Arm.ControlManager.SendTrajectoryFunctionnality(pointsTrajectory);				
				}
				 catch (Exception e)
	            {
	                System.Console.WriteLine(
	                    "JACO API SendTrajectory Failed (Angular Joint Positions): ");
	                System.Console.WriteLine(e.ToString());					
	            }
			}
			
		}
		
		public void SendFingersAngleRads(double f1, double f2, double f3)
		{
			if (m_Arm.JacoIsReady())
			{
				try
				{
					CPointsTrajectory pointsTrajectory = new CPointsTrajectory();
					//CVectorAngle vector = new CVectorAngle();
			        CTrajectoryInfo trajectory = new CTrajectoryInfo();
			
			        //trajectory.UserPosition.AnglesJoints = vector;
			        trajectory.UserPosition.PositionType = CJacoStructures.PositionType.PositionNoMovements;
			
			        trajectory.UserPosition.HandMode = CJacoStructures.HandMode.PositionMode;
					
					//Trajectory needs to be converted to degrees
			        trajectory.UserPosition.FingerPosition[0] = Convert.ToSingle(f1 * 180.0 / Math.PI);
			        trajectory.UserPosition.FingerPosition[1] = Convert.ToSingle(f2 * 180.0 / Math.PI);
			        trajectory.UserPosition.FingerPosition[2] = Convert.ToSingle(f3 * 180.0 / Math.PI);
			
			        pointsTrajectory.Add(trajectory);
			
			        m_Arm.ControlManager.SendTrajectoryFunctionnality(pointsTrajectory); 
					
				}
				catch (Exception e)
				{
					 System.Console.WriteLine(
	                    "JACO API SendTrajectory Failed (Finger Positions): ");
	                System.Console.WriteLine(e.ToString());		
				}
			}
		}
		
		

        public void SendRelCartCommand(double x, double y, double z)
        {
            m_LastCmd = DateTime.Now;
            m_Arm.ConfigurationsManager.SetCartesianControl();

            if (x < -CMD_EPSILON)
                m_Cmd.InclineLR = -1;
            else if (x > CMD_EPSILON)
                m_Cmd.InclineLR = 1;
            else
                m_Cmd.InclineLR = 0;

            if (y < -CMD_EPSILON)
                m_Cmd.InclineFB = -1;
            else if (y > CMD_EPSILON)
                m_Cmd.InclineFB = 1;
            else
                m_Cmd.InclineFB = 0;

            if (z < -CMD_EPSILON)
                m_Cmd.Rotate = -1;
            else if (y > CMD_EPSILON)
                m_Cmd.Rotate = 1;
            else
                m_Cmd.Rotate = 0;
        }

        public void SendRelJointCommand(JacoJointCommand cmd)
        {
            m_LastCmd = DateTime.Now;
            m_Arm.ConfigurationsManager.SetAngularControl();

            if (cmd.shoulder_pitch < -CMD_EPSILON)
                m_Cmd.InclineLR = -1;
            else if (cmd.shoulder_pitch > CMD_EPSILON)
                m_Cmd.InclineLR = 1;
            else
                m_Cmd.InclineLR = 0;
        }

        public void GoToPose(JacoPose pose)
        {
            CPointsTrajectory traj = new CPointsTrajectory();
            CTrajectoryInfo ti= new CTrajectoryInfo();
            ti.UserPosition.PositionType = 
                CJacoStructures.PositionType.CartesianPosition;
             
             //ti.UserPosition.PositionType = 
             //CJacoStructures.PositionType.AngularPosition;   
             
             
            ti.UserPosition.HandMode =
                CJacoStructures.HandMode.NoMovements;
            CVectorEuler p = new CVectorEuler();
                                           
            p.Position = new Single[3] {
                Convert.ToSingle(pose.x), 
                Convert.ToSingle(pose.y), 
                Convert.ToSingle(pose.z)};
            p.Rotation = new Single[3] {
                Convert.ToSingle(pose.e_x), 
                Convert.ToSingle(pose.e_y), 
                Convert.ToSingle(pose.e_z)};
     
            ti.UserPosition.Position = p;

            traj.Add(ti);
            m_Arm.ControlManager.SendTrajectoryFunctionnality(traj);

        }
        
        

	public void Retract()
	{
			
			System.Console.WriteLine(
	                    "JACO API Retract()");
			
		//Retract again...
		m_Cmd = new CJoystickValue();
		m_Cmd.ButtonValue[2] = 1;
		m_Arm.ControlManager.SendJoystickFunctionnality(m_Cmd);
		m_LastCmd = DateTime.Now;
	    m_IsRetracting = true;
			
	}

        private void InitSequence()
        {

			//Initialize fingers
			//InitFingers();
			
            m_Cmd = new CJoystickValue();
            m_Cmd.ButtonValue[2] = 1; // Retract/Ready button.
            m_Arm.ControlManager.SendJoystickFunctionnality(m_Cmd);
            // The button release will be done in the update() method at the
            // appropriate time.
            m_LastCmd = DateTime.Now;
            m_RetractDelay = new TimeSpan(0,0,0,24,0); // 8 secs.
            m_IsRetracting = true;


        }
		
		
	private void InitFingers()
	{
		System.Console.WriteLine("JACO API InitFingers()");
			
		//An object that represents the state of a joystick.	
		CJoystickValue Joystick = new CJoystickValue();
		CControlMappingCharts mapping = m_Arm.ConfigurationsManager.GetControlMappingCharts();
		
		//We set the mapping system in a mode that we need.
		mapping.ActualControlMapping = 2;
		mapping.ControlMapping[2].ActualModeA = 1;
		mapping.ControlMapping[2].ActualModeB = -1;	
			
			
		//We set our mapping configuration
        m_Arm.ConfigurationsManager.SetControlMappingCharts(mapping);

        //We verify that the change has been made correctly(only for learning purpose :) )
        mapping = m_Arm.ConfigurationsManager.GetControlMappingCharts();

        //We show informations on screen to make that we execute the right functionality
        System.Console.WriteLine("Current mapping = " + mapping.ActualControlMapping);
        System.Console.WriteLine("Actual mode A = " + mapping.ControlMapping[2].ActualModeA);
        System.Console.WriteLine("Mode A 1 - INCLINE LR : " + (CJacoStructures.ControlFunctionnalityValues)mapping.ControlMapping[2].ModeControlsA[1].ControlSticks[(int)CJacoStructures.ControlStickValues.InclineLR].Plus);
        
        //We tell Jaco that from now on, the API is taking control.
        //m_Arm.ControlManager.StartControlAPI();
        
        //We set the INCLINELR stick's Value of our vitual joystick to -1(like someone was fully pressing the stick to the right)
        Joystick.InclineLR = -1f;
			
		//We send the command long enough for the fingers to be initialized.
		for (int i = 0; i < 100; i++ )							
		{
				
			System.Console.WriteLine("Looping for finger init i=" + i);	
				
			try
			{
			    m_Arm.ControlManager.SendJoystickFunctionnality(Joystick);
			}
			catch(Exception ex)
			{
			    System.Console.WriteLine("Exception = " + ex.GetType());
			}
		
			Thread.Sleep(10);
		}
		
		//We reset the joystick value.(like the person release the stick)
		Joystick.InclineLR = 0;
		
		//We send it to Jaco
		m_Arm.ControlManager.SendJoystickFunctionnality(Joystick);	
	}
		
		

        private CJacoArm m_Arm;
        private JacoArmState m_State;
        private bool m_IsEnabled;
        private bool m_IsRetracting;
        private CJoystickValue m_Cmd;
        private TimeSpan m_WatchDogDelay;
        private TimeSpan m_RetractDelay;

        private DateTime m_LastCmd; 
        private const double CMD_EPSILON = 1e-6;
    }
}


