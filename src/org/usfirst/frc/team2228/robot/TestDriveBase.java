package org.usfirst.frc.team2228.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class TestDriveBase {
	public WPI_TalonSRX right1;
	public WPI_TalonSRX right2;
	public WPI_TalonSRX left1;
	public WPI_TalonSRX left2;
	private DifferentialDrive drive;
	private DriverIF driverIf;

	public TestDriveBase(DriverIF _driverIf) {
		driverIf = _driverIf;
		right1 = new WPI_TalonSRX(RobotMap.CAN_ID_1);
		right2 = new WPI_TalonSRX(RobotMap.CAN_ID_2);
		left1 = new WPI_TalonSRX(RobotMap.CAN_ID_3);
		left2 = new WPI_TalonSRX(RobotMap.CAN_ID_4);
		drive = new DifferentialDrive(left1, right1);

//		right1.setName("Drive Base", "Right Master");
		// right2.setName("Drive Base", "Right Follower");
//		left1.setName("Drive Base", "Left Master");
		// left2.setName("Drive Base", "Left Follower");
		/*
		 * Set right/left masters and right/left followers
		 */
		// Set Right master to percentVbus mode
		right1.set(ControlMode.PercentOutput, 0);

		// Set up right follower
		right2.set(ControlMode.Follower, right1.getDeviceID());

		// Set follower motor to follow master

		// Set left master to percentVbus mode
		left1.set(ControlMode.PercentOutput, 0);

		// Set up left follower
		left2.set(ControlMode.Follower, left1.getDeviceID());

		// Set follower motor to follow master

	}

	public void teleopPeriodic() {
		drive.arcadeDrive(driverIf.Throttle(), driverIf.Turn());
	}
}
