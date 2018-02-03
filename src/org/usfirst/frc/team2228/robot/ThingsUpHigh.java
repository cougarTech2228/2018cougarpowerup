package org.usfirst.frc.team2228.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DMC60;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ThingsUpHigh {
	boolean on = true, off = false;
	DriverIF driverIF;
	WPI_TalonSRX elevator;
	DMC60 conveyor1, conveyor2;
	WPI_TalonSRX winch;

	public ThingsUpHigh(DriverIF _driverIF) {
		driverIF = _driverIF;
		elevator = new WPI_TalonSRX(RobotMap.CAN_ID_5);
		winch = new WPI_TalonSRX(RobotMap.CAN_ID_6);
		conveyor1 = new DMC60(RobotMap.PWM_PORT_2);
		conveyor2 = new DMC60(RobotMap.PWM_PORT_3);
		elevator.set(0);
		
		SmartDashboard.putNumber("back conveyor:", 0);
		SmartDashboard.putNumber("Elevator Speed:", 0);
	}

	public void teleopPeriodic() {
		double b = SmartDashboard.getNumber("Elevator Speed:", 0);
		//b is the speed of the 
		
		if (driverIF.LeftTrigger()) {
			elevator.set(b);
		} else if (driverIF.RightTrigger()) {
			elevator.set(-b);
		}
		else{
			elevator.set(0);
		}
		
		
		double d = SmartDashboard.getNumber("back conveyor:", 0);
		//d is the speed of the elevator motors
		
		if(driverIF.BackConveyorForwards()) {
			conveyor1.set(d);
			System.out.println(d);
		}
		else if(driverIF.BackConveyorBackwards()) {
			conveyor1.set(-d);
			System.out.println(-d);
		}
		else {
			conveyor1.set(0);
		}
	}

}
