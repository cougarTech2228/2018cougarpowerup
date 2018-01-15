package org.usfirst.frc.team2228.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;

public class CubeManipulator {
	public XboxIF xboxIF;
	public XboxController xbox;
	private WPI_TalonSRX left;
	private WPI_TalonSRX right;
	private WPI_TalonSRX squeeze;
	private boolean collection;
	private boolean expulsion;
	private boolean squeezeIn;
	private boolean release;
	private double cubeCollectionValue = 1.0;
	private double cubeExpulsionValue = -1.0;
	private double cubeGripValue = .1;
	private double cubeReleaseValue = -.1;

	public CubeManipulator(XboxController _xbox) {
		xboxIF = new XboxIF();
		xbox = _xbox;
		collection = false;
		expulsion = false;
		squeezeIn = false;
		release = false;
		left = new WPI_TalonSRX(RobotMap.CAN_ID_5);
		right = new WPI_TalonSRX(RobotMap.CAN_ID_6);
		squeeze = new WPI_TalonSRX(RobotMap.CAN_ID_7);
//		XboxController xbox = new XboxController();
		left.set(ControlMode.PercentOutput, 0);
		right.set(ControlMode.PercentOutput, 0);
		right.setInverted(true);
		squeeze.set(ControlMode.PercentOutput, 0);
	}

	public void teleopPeriodic() {
		//git outta heyah
		//c ya laytah
		collection = xboxIF.X_BUTTON();
		expulsion = xboxIF.B_BUTTON();
		squeezeIn = xboxIF.A_BUTTON();
		release = xboxIF.Y_BUTTON();
		
		right.set(0);
		left.set(0);
		squeeze.set(0);
		
		if (collection) {
			left.set(cubeCollectionValue);
			right.set(cubeCollectionValue);
		}
		if (expulsion){
			left.set(cubeExpulsionValue);
			right.set(cubeExpulsionValue);	
		}
		if (squeezeIn){
			squeeze.set(cubeGripValue);
		}
		if(release){
			squeeze.set(cubeReleaseValue);
		}
	}

}
