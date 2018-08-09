package org.usfirst.frc.team2228.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class DriveBase {
	
	public Motor RightMaster;
	public Motor RightFollower;
	public Motor LeftMaster;
	public Motor LeftFollower;
	public double DriveSpeedPercentage = 0.8;
	public double TurnSpeedPercentage = 0.5;
	
	public DriveBase() {
		RightMaster = new Motor(RobotMap.RIGHT_MASTER);
		RightFollower = new Motor(RobotMap.RIGHT_FOLLOWER, RightMaster);
		LeftMaster = new Motor(RobotMap.LEFT_MASTER);
		LeftFollower = new Motor(RobotMap.LEFT_FOLLOWER, LeftMaster);
		
		RightMaster.Invert(true);
		RightFollower.Invert(true);
		
		RightMaster.SetBrakeMode(true);
		RightFollower.SetBrakeMode(true);
		LeftMaster.SetBrakeMode(true);
		LeftFollower.SetBrakeMode(true);
		
		//RightMaster.
	}
	private double ZeroLimit(double input) {
		if(Math.abs(input) < 0.1)
			return 0;
		return input;
	}
	public void TeleopInit() {
		RightMaster.TeleopInit();
		LeftMaster.TeleopInit();
	}
	private double Limit(double input) {
		if(input > 1)
			input = 1;
		if(input < -1)
			input = -1;
		return input;
	}
	public void TeleopMove(DriverIF Controller) {
		double Forward = Controller.Throttle();
		double Turn = Controller.Turn();
		double Right, Left;
		
		Forward = ZeroLimit(Forward);
		Turn = ZeroLimit(Turn);

		Turn  *= TurnSpeedPercentage;
		
		System.out.println(LeftMaster.GetSensorVelocity() + " yay " + RightMaster.GetSensorVelocity());
		
		Right = Limit(Forward + Turn);
		Left  = Limit(Forward - Turn);
		
		Right *= DriveSpeedPercentage;
		Left  *= DriveSpeedPercentage;
		/*
		if(Forward == 0 && Turn == 0) {
			double LeftVelocity = LeftMaster.GetSensorVelocity();
			double RightVelocity = RightMaster.GetSensorVelocity();
			if(Math.abs(LeftVelocity) > 10)  {
				Left = -Left;
			}
			if(Math.abs(RightVelocity) > 10) {
				Right = -Right;
			}
		}*/
		
		//System.out.println("Right: " + Right + " Left: " + Left);
		
		RightMaster.Set(ControlMode.Velocity, Right * SRXDriveBaseCfg.MaxVel_VelNativeUnits);
		LeftMaster.Set(ControlMode.Velocity, Left * SRXDriveBaseCfg.MaxVel_VelNativeUnits);
	}
}
