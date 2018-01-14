package org.usfirst.frc.team2228.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;

public class CubeManipulator {
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
		xbox = _xbox;
//		collection = false;
//		expulsion = false;
//		squeezeIn = false;
//		release = false;
		WPI_TalonSRX left = new WPI_TalonSRX(RobotMap.CAN_ID_5);
		WPI_TalonSRX right = new WPI_TalonSRX(RobotMap.CAN_ID_6);
		WPI_TalonSRX squeeze = new WPI_TalonSRX(RobotMap.CAN_ID_7);
//		XboxController xbox = new XboxController();
		left.set(ControlMode.PercentOutput, 0);
		right.set(ControlMode.PercentOutput, 0);
		right.setInverted(true);
		squeeze.set(ControlMode.PercentOutput, 0.1);
	}

	public void teleopPeriodic() {
		//git outta heyah
		//c ya laytah
		collection = xbox.getXButton();
		expulsion = xbox.getBButton();
		squeezeIn = xbox.getAButton();
		release = xbox.getYButton();
		
		if (collection) {
			left.set(cubeCollectionValue);
			right.set(cubeCollectionValue);
			
		}
		else if (expulsion){
			left.set(cubeExpulsionValue);
			right.set(cubeExpulsionValue);
			
		}
		else if (squeezeIn){
			squeeze.set(cubeGripValue);
		}
		else if(release){
			squeeze.set(cubeReleaseValue);
		}
	}

}
