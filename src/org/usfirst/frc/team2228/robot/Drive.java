package org.usfirst.frc.team2228.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drive {
	public WPI_TalonSRX right1;
	public WPI_TalonSRX right2;
	public WPI_TalonSRX left1;
	public WPI_TalonSRX left2;
	private DifferentialDrive drive;
	private DriverIF driverIf;
	private AngleIF navx;
	

	public Drive(DriverIF _driverIf, AngleIF _navx) {
		driverIf = _driverIf;
		right1 = new WPI_TalonSRX(RobotMap.CAN_ID_1);
		right2 = new WPI_TalonSRX(RobotMap.CAN_ID_2);
		left1 = new WPI_TalonSRX(RobotMap.CAN_ID_3);
		left2 = new WPI_TalonSRX(RobotMap.CAN_ID_4);
		drive = new DifferentialDrive(left1, right1);
		navx = _navx;
		navx.zeroYaw();
		
		 right1.setName("Drive Base", "Right Master");
		// right2.setName("Drive Base", "Right Follower");
		 left1.setName("Drive Base", "Left Master");

		right1.set(ControlMode.PercentOutput, 0);
		right2.set(ControlMode.Follower, right1.getDeviceID());

		left1.set(ControlMode.PercentOutput, 0);

		left2.set(ControlMode.Follower, left1.getDeviceID());
		
		// Set follower motor to follow master

	}

	public void teleopPeriodic() {
		Drive(driverIf.Throttle());
		//System.out.println(navx.getAngle());
		
		
		
	}
	public void Drive(double speed) {
		//speed = 0.7;
		double turn = Math.round(driverIf.Turn() * 100) / 100.0;
		double angle = 90 - navx.getAngle();
		double error = 10;
		double cor = Math.abs(angle) / 90 * 0.4 + 0.5;
		
		
		
		if(turn > 1)
			turn = 1;
		if(turn < 0)
			turn = 0;
		
		//if(Math.abs(angle) < 30)
			//speed = 0.55;
		
		if(angle > 0)
			speed = cor;
		if(angle < 0)
			speed = -cor;
		
			
		if(Math.abs(angle) > error / 2)
			drive.arcadeDrive(driverIf.Throttle(), speed);
		System.out.println("RREEE" + navx.getAngle());
	}
	public void turn(double angle, double speed) {
		
	}
	public void rotate(double angle, double speed) {
		
	}
}











