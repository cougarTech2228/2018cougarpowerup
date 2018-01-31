package org.usfirst.frc.team2228.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PWM;

public class ThingsUpHigh {
	boolean on = true, off = false;
	DriverIF driverIF;
	WPI_TalonSRX elevator;
	PWM conveyor1, conveyor2;
	WPI_TalonSRX winch;

	public ThingsUpHigh() {
		elevator = new WPI_TalonSRX(RobotMap.CAN_ID_5);
		winch = new WPI_TalonSRX(RobotMap.CAN_ID_6);
		conveyor1 = new PWM(RobotMap.PWM_PORT_3);
		conveyor2 = new PWM(RobotMap.PWM_PORT_4);
	}

	public void teleopPeriodic() {
		if (driverIF.raiseElevator()) {
			elevator.set(.3);
			System.out.println("Raise");
		} else if (driverIF.lowerElevator()) {
			elevator.set(-.3);
			System.out.println("Lower");
		}
		else{
			elevator.set(0);
		}
	}

}
