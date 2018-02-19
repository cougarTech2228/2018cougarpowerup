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

	public CubeManipulator(DriverIF _driverIf) {
		driverIf = _driverIf;
		left = new Spark(RobotMap.PWM_PORT_0);
		right = new Spark(RobotMap.PWM_PORT_1);
		
		SmartDashboard.putNumber("CollectionValue", 0.25);
		SmartDashboard.putNumber("ExpulsionValue", -0.25);
		
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
		
		if (!driverIf.collectionToggle() && lastButton && triggered == false) {
			left.set(cubeCollectionValue);
			right.set(cubeCollectionValue);
			triggered = true;
			System.out.println("Suck in");
		}
		else if (!driverIf.collectionToggle() && lastButton && triggered == true) {
			left.set(0);
			right.set(0);
			triggered = false;
		}
		if (!driverIf.expulsion() && lastButton2 && triggered2 == false) {
			left.set(cubeExpulsionValue);
			right.set(cubeExpulsionValue);
			triggered2 = true;
		}
		else if (!driverIf.expulsion() && lastButton2 && triggered2 == true) {
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


