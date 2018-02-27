package org.usfirst.frc.team2228.robot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CubeManipulator {
	public DriverIF driverIf;
	private Spark left;
	private Spark right;
	private double cubeCollectionValue;
	private double cubeExpulsionValue;
	private boolean triggered = false;
	boolean triggered2 = false;
	private boolean lastButton = false;
	boolean lastButton2 = false;
	private Toggler collect, expult;

	public CubeManipulator(DriverIF _driverIf) {
		driverIf = _driverIf;
		left = new Spark(RobotMap.PWM_PORT_0);
		right = new Spark(RobotMap.PWM_PORT_1);
		
		collect = new Toggler(2);
		expult = new Toggler(2);
		
		SmartDashboard.putNumber("CollectionValue", 0.75);
		SmartDashboard.putNumber("ExpulsionValue", -0.75);
		
		// positive is clockwise, negative is counter clockwise
		left.set(0);
		right.set(0);
		left.setInverted(true);
		// Nice code dude
	}

	public void teleopPeriodic() {
		cubeCollectionValue = SmartDashboard.getNumber("CollectionValue", 0.5);
		cubeExpulsionValue = SmartDashboard.getNumber("ExpulsionValue", -0.5);
//		if (driverIf.collection()) {
//			left.set(cubeCollectionValue);
//			right.set(cubeCollectionValue);
//			triggered = true;
//		}
//		else if (driverIf.expulsion()) {
//			left.set(cubeExpulsionValue);
//			right.set(cubeExpulsionValue);
//			triggered = false;
//		}
//		else{
//			left.set(0);
//			right.set(0);
//		}
		
		if (collect.toggle(driverIf.collectionToggle()) == 0) {
			left.set(cubeCollectionValue);
			right.set(cubeCollectionValue);
			triggered = true;
			System.out.println("Suck in");
		}
		else {
			left.set(0);
			right.set(0);
			triggered = false;
		}
		if (expult.toggle(driverIf.expulsion()) == 0) {
			left.set(cubeExpulsionValue);
			right.set(cubeExpulsionValue);
			triggered2 = true;
		}
		else {
			left.set(0);
			right.set(0);
			triggered2 = false;
		}
		
//		else if(driverIf.expulsion()){
//		left.set(cubeExpulsionValue);
//		right.set(cubeExpulsionValue);
//		}
		lastButton = driverIf.collectionToggle();
		lastButton2 = driverIf.expulsion();
		
	}
			
}


