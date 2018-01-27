package org.usfirst.frc.team2228.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CubeManipulator {
	public DriverIF driverIf;
	public XboxController xbox;
	private WPI_TalonSRX left;
	private WPI_TalonSRX right;
	private double cubeCollectionValue;
	private double cubeExpulsionValue;
;

	public CubeManipulator(DriverIF _driverIf) {
		driverIf = _driverIf;
		left = new WPI_TalonSRX(RobotMap.CAN_ID_5);
		right = new WPI_TalonSRX(RobotMap.CAN_ID_6);

		// XboxController xbox = new XboxController();
		SmartDashboard.putNumber("CollectionValue", 0.1);
		SmartDashboard.putNumber("ExpulsionValue", -0.1);

		left.set(ControlMode.PercentOutput, 0);
		right.set(ControlMode.PercentOutput, 0);
		right.setInverted(true);

		// Nice code dude
	}

	public void teleopPeriodic() {
		cubeCollectionValue = SmartDashboard.getNumber("CollectionValue", 0.1);
		cubeExpulsionValue = SmartDashboard.getNumber("ExpulsionValue", -0.1);
		right.set(0);
		left.set(0);

		if (driverIf.collection()) {
			left.set(cubeCollectionValue);
			right.set(-cubeCollectionValue);
		}
		if (driverIf.expulsion()) {
			left.set(cubeExpulsionValue);
			right.set(-cubeExpulsionValue);
		}
	}

}
