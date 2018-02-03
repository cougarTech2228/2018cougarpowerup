package org.usfirst.frc.team2228.robot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CubeManipulator {
	public DriverIF driverIf;
	private Spark left;
	private Spark right;
	private double cubeCollectionValue;
	private double cubeExpulsionValue;

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
		cubeCollectionValue = SmartDashboard.getNumber("CollectionValue", 0.1);
		cubeExpulsionValue = SmartDashboard.getNumber("ExpulsionValue", -0.1);

		if (driverIf.collection()) {
			left.set(cubeCollectionValue);
			right.set(cubeCollectionValue);
		}
		else if (driverIf.expulsion()) {
			left.set(cubeExpulsionValue);
			right.set(cubeExpulsionValue);
		}
		else{
			left.set(0);
			right.set(0);
		}
		
	}
			
}


