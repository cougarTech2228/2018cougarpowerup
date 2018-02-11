package org.usfirst.frc.team2228.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DMC60;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Relay;

public class ThingsUpHigh {
	boolean on = true, off = false;
	DriverIF driverIF;
	WPI_TalonSRX elevator;
	DMC60 conveyor1, conveyor2;
	WPI_TalonSRX winch;
	Relay hook;
	Spark hookDown;

	public ThingsUpHigh(DriverIF _driverIF) {
		driverIF = _driverIF;
		elevator = new WPI_TalonSRX(RobotMap.CAN_ID_5);
		winch = new WPI_TalonSRX(RobotMap.CAN_ID_6);
		conveyor1 = new DMC60(RobotMap.PWM_PORT_2);
		conveyor2 = new DMC60(RobotMap.PWM_PORT_3);
		elevator.set(0);
		hook = new Relay(0, Relay.Direction.kForward);
		hook.set(Relay.Value.kForward);
		hookDown = new Spark(RobotMap.PWM_PORT_4);
		hookDown.set(0);
		SmartDashboard.putNumber("back conveyor:", 0);
		SmartDashboard.putNumber("front conveyor:", 0);
		SmartDashboard.putNumber("Elevator Speed:", 0);
		SmartDashboard.putNumber("Launch:", 0);
	}

	public void teleopPeriodic() {
		double b = 1; 
				//SmartDashboard.getNumber("Elevator Speed:", 0);
		//b is the speed of the 
		
		if (driverIF.hookForward()) {
			hookDown.set(b);
		} else if (driverIF.hookBackward()) {
			hookDown.set(-b);
		}
		else{
			hookDown.set(0);
		}
		
		if (driverIF.RaiseElevator()) {
			elevator.set(b);
		} else if (driverIF.LowerElevator()) {
			elevator.set(-b);
		}
		else{
			elevator.set(0);
		}
		
		double d = 1;
				//SmartDashboard.getNumber("back conveyor:", 0);
		//d is the speed of the elevator motors
		
		if(driverIF.BackConveyorForwards()) {
			conveyor1.set(d);
			System.out.println("BACKFOR");
		}
		else if(driverIF.BackConveyorBackwards()) {
			conveyor1.set(-d);
			System.out.println("BACKBACK");
		}
		else {
			conveyor1.set(0);
		}
		
		double e = 1; 
				//SmartDashboard.getNumber("front conveyor:", 0);
		//hya
		
		if(driverIF.FrontConveyorForwards()) {
			conveyor2.set(e);
			System.out.println("FRONTFOR");
		}
		else if(driverIF.FrontConveyorBackwards()) {
			conveyor2.set(-e);
			System.out.println("FRONTBACK");
		}
		else {
			conveyor2.set(0);
		}
		double LaunchValue = SmartDashboard.getNumber("Launch:", 1);
		if (LaunchValue == 1){
			hook.set(Relay.Value.kOff);
		}
			
	}

}
